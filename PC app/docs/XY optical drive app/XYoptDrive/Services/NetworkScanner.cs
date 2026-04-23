using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Linq;
using System.Net;
using System.Net.Http;
using System.Net.NetworkInformation;
using System.Net.Sockets;
using System.Threading;
using System.Threading.Tasks;

namespace XYoptDrive.Services;

public class ScanResult
{
    public string IpAddress { get; set; } = "";
    public bool IsArduino { get; set; }
    public string Info { get; set; } = "";
}

public static class NetworkScanner
{
    /// <summary>
    /// Scans all hosts in the /24 subnets of the given IP addresses.
    /// </summary>
    public static async Task<List<ScanResult>> ScanSubnetAsync(
        IEnumerable<string> baseIps, IProgress<int>? progress = null, CancellationToken ct = default)
    {
        // Collect unique /24 subnets to scan
        var subnets = new HashSet<string>();
        foreach (var ip in baseIps)
        {
            var parts = ip.Split('.');
            if (parts.Length == 4)
                subnets.Add($"{parts[0]}.{parts[1]}.{parts[2]}");
        }

        if (subnets.Count == 0)
            return new List<ScanResult>();

        var allIps = subnets.SelectMany(s => Enumerable.Range(1, 254).Select(i => $"{s}.{i}")).ToList();
        var results = new ConcurrentBag<ScanResult>();
        int completed = 0;
        int total = allIps.Count;

        using var semaphore = new SemaphoreSlim(50);
        var tasks = new List<Task>();

        foreach (var ip in allIps)
        {
            tasks.Add(Task.Run(async () =>
            {
                await semaphore.WaitAsync(ct);
                try
                {
                    ct.ThrowIfCancellationRequested();
                    var result = await ProbeHostAsync(ip, ct);
                    if (result != null)
                        results.Add(result);
                }
                finally
                {
                    semaphore.Release();
                    var c = Interlocked.Increment(ref completed);
                    progress?.Report(c * 100 / total);
                }
            }, ct));
        }

        await Task.WhenAll(tasks);
        return results.OrderBy(r =>
        {
            var octets = r.IpAddress.Split('.');
            return int.TryParse(octets.LastOrDefault(), out var last) ? last : 0;
        }).ToList();
    }

    /// <summary>
    /// Overload accepting a single IP for backward compatibility.
    /// </summary>
    public static Task<List<ScanResult>> ScanSubnetAsync(
        string baseIp, IProgress<int>? progress = null, CancellationToken ct = default)
        => ScanSubnetAsync(new[] { baseIp }, progress, ct);

    /// <summary>
    /// Probes a single host: ping first, then try HTTP /status to identify Arduino.
    /// </summary>
    public static async Task<ScanResult?> ProbeHostAsync(string ip, CancellationToken ct = default)
    {
        using var ping = new Ping();
        try
        {
            var reply = await ping.SendPingAsync(ip, 500);
            if (reply.Status != IPStatus.Success)
                return null;

            var result = new ScanResult { IpAddress = ip, Info = "Active host" };

            // Try HTTP GET /status to detect Arduino web server
            try
            {
                using var client = new HttpClient { Timeout = TimeSpan.FromSeconds(2) };
                var response = await client.GetAsync($"http://{ip}/status", ct);
                if (response.IsSuccessStatusCode)
                {
                    var content = await response.Content.ReadAsStringAsync(ct);
                    result.IsArduino = true;
                    result.Info = $"Arduino detected — {content.Trim()}";
                }
            }
            catch
            {
                // Host is alive but no Arduino web server
            }

            return result;
        }
        catch
        {
            return null;
        }
    }

    /// <summary>
    /// Returns the first usable local IPv4 address.
    /// </summary>
    public static string? GetLocalIpAddress()
    {
        foreach (var ni in NetworkInterface.GetAllNetworkInterfaces())
        {
            if (ni.OperationalStatus != OperationalStatus.Up) continue;
            if (ni.NetworkInterfaceType == NetworkInterfaceType.Loopback) continue;

            foreach (var addr in ni.GetIPProperties().UnicastAddresses)
            {
                if (addr.Address.AddressFamily == AddressFamily.InterNetwork)
                    return addr.Address.ToString();
            }
        }
        return null;
    }

    /// <summary>
    /// Returns all usable local IPv4 addresses.
    /// </summary>
    public static List<string> GetAllLocalIpAddresses()
    {
        var addresses = new List<string>();
        foreach (var ni in NetworkInterface.GetAllNetworkInterfaces())
        {
            if (ni.OperationalStatus != OperationalStatus.Up) continue;
            if (ni.NetworkInterfaceType == NetworkInterfaceType.Loopback) continue;

            foreach (var addr in ni.GetIPProperties().UnicastAddresses)
            {
                if (addr.Address.AddressFamily == AddressFamily.InterNetwork)
                    addresses.Add(addr.Address.ToString());
            }
        }
        return addresses;
    }
}
