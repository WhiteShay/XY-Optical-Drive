using System.Diagnostics;
using System.Net.Http;
using System.Text.Json;
using Xunit.Abstractions;
using XYoptDrive.Models;
using XYoptDrive.Services;

namespace XYoptDrive.UserJourney1.Tests;

public class UnitTest1
{
    private const string TargetIp = "169.254.43.138";
    private const string ScanSubnet = "169.254.43.0";
    private readonly ITestOutputHelper _output;

    public UnitTest1(ITestOutputHelper output)
    {
        _output = output;
    }

    [Fact(Timeout = 180000)]
    public async Task UserJourney1_Scan_Add_OpenEndpointValidation()
    {
        var startedAt = DateTime.UtcNow;
        Log("Starting UserJourney1 sequence");

        Log($"Scanning subnet based on {ScanSubnet}");
        var results = await NetworkScanner.ScanSubnetAsync(new[] { ScanSubnet });
        var target = results.FirstOrDefault(r => r.IpAddress == TargetIp);
        Log($"Scan complete. Hosts found: {results.Count}");

        Assert.NotNull(target);
        Log($"Target {TargetIp} discovered. Arduino flag: {target!.IsArduino}");

        var configured = new List<Device>();
        if (configured.All(d => d.IpAddress != TargetIp))
        {
            configured.Add(new Device
            {
                Name = target.IsArduino ? "Arduino" : "Device",
                Role = target.IsArduino ? "XY Optical Drive" : "",
                IpAddress = target.IpAddress,
                IsOnline = true,
                StatusInfo = target.Info
            });
        }
        Log($"Device list contains target: {configured.Any(d => d.IpAddress == TargetIp)}");

        var probe = await NetworkScanner.ProbeHostAsync(TargetIp);
        Assert.NotNull(probe);
        Log($"Probe response: {probe!.Info}");

        using var http = new HttpClient { Timeout = TimeSpan.FromSeconds(6) };

        var rootStarted = DateTime.UtcNow;
        var rootResponse = await http.GetAsync($"http://{TargetIp}");
        Log($"GET / at {rootStarted:HH:mm:ss.fff} => {(int)rootResponse.StatusCode}");

        var statusStarted = DateTime.UtcNow;
        var statusResponse = await http.GetAsync($"http://{TargetIp}/status");
        var statusBody = await statusResponse.Content.ReadAsStringAsync();
        Log($"GET /status at {statusStarted:HH:mm:ss.fff} => {(int)statusResponse.StatusCode}");

        Assert.True(rootResponse.IsSuccessStatusCode,
            $"Expected root page to be reachable at http://{TargetIp}, got {(int)rootResponse.StatusCode}.");
        Assert.True(statusResponse.IsSuccessStatusCode,
            $"Expected /status to be reachable at http://{TargetIp}/status, got {(int)statusResponse.StatusCode}.");

        using var json = JsonDocument.Parse(statusBody);
        Assert.True(json.RootElement.TryGetProperty("uptime", out _), "Expected /status JSON to contain 'uptime'.");

        var duration = DateTime.UtcNow - startedAt;
        Log($"UserJourney1 completed in {duration.TotalSeconds:F1}s");
    }

    private void Log(string message)
    {
        _output.WriteLine($"[{DateTime.Now:HH:mm:ss.fff}] {message}");
    }

}