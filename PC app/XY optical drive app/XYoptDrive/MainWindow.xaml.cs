using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Linq;
using System.Net;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;
using XYoptDrive.Models;
using XYoptDrive.Services;

namespace XYoptDrive;

public partial class MainWindow : Window
{
    private readonly ObservableCollection<Device> _devices = new();
    private readonly ObservableCollection<ScanResult> _scanResults = new();
    private CancellationTokenSource? _scanCts;

    public MainWindow()
    {
        InitializeComponent();

        DeviceGrid.ItemsSource = _devices;
        ScanResultsList.ItemsSource = _scanResults;

        LoadDevices();

        // Pre-fill subnet field with local IP
        var localIp = NetworkScanner.GetLocalIpAddress();
        if (localIp != null)
            TxtSubnet.Text = localIp;

        UpdateStatus($"{_devices.Count} device(s) configured");
    }

    // ───── Persistence ─────

    private void LoadDevices()
    {
        _devices.Clear();
        foreach (var d in DeviceStore.Load())
            _devices.Add(d);
    }

    private void SaveDevices()
    {
        DeviceStore.Save(_devices);
    }

    private void UpdateStatus(string message)
    {
        TxtStatus.Text = message;
    }

    // ───── Add Device ─────

    private void BtnAdd_Click(object sender, RoutedEventArgs e)
    {
        var name = TxtName.Text.Trim();
        var role = TxtRole.Text.Trim();
        var ip = TxtIp.Text.Trim();

        if (string.IsNullOrEmpty(name) || string.IsNullOrEmpty(ip))
        {
            MessageBox.Show("Name and IP address are required.",
                "Missing Field", MessageBoxButton.OK, MessageBoxImage.Warning);
            return;
        }

        if (!IPAddress.TryParse(ip, out _))
        {
            MessageBox.Show("The IP address entered is not valid.",
                "Invalid Address", MessageBoxButton.OK, MessageBoxImage.Warning);
            return;
        }

        if (_devices.Any(d => d.IpAddress == ip))
        {
            MessageBox.Show("A device with this IP address already exists.",
                "Duplicate", MessageBoxButton.OK, MessageBoxImage.Information);
            return;
        }

        _devices.Add(new Device { Name = name, Role = role, IpAddress = ip });
        SaveDevices();

        TxtName.Clear();
        TxtRole.Clear();
        TxtIp.Clear();

        UpdateStatus($"'{name}' added — {_devices.Count} device(s)");
    }

    // ───── Test Connection ─────

    private async void BtnTest_Click(object sender, RoutedEventArgs e)
    {
        if (DeviceGrid.SelectedItem is not Device device)
        {
            MessageBox.Show("Select a device to test.",
                "No Selection", MessageBoxButton.OK, MessageBoxImage.Information);
            return;
        }

        BtnTest.IsEnabled = false;
        UpdateStatus($"Testing {device.IpAddress}...");

        var probe = await NetworkScanner.ProbeHostAsync(device.IpAddress);
        device.IsOnline = probe != null;
        device.StatusInfo = probe?.Info ?? "Unreachable";

        BtnTest.IsEnabled = true;
        UpdateStatus(device.IsOnline
            ? $"{device.Name} is online"
            : $"{device.Name} is offline");
    }

    // ───── Refresh All ─────

    private async void BtnRefreshAll_Click(object sender, RoutedEventArgs e)
    {
        BtnRefreshAll.IsEnabled = false;
        UpdateStatus("Refreshing all devices...");

        var tasks = _devices.Select(async device =>
        {
            var probe = await NetworkScanner.ProbeHostAsync(device.IpAddress);
            device.IsOnline = probe != null;
            device.StatusInfo = probe?.Info ?? "Unreachable";
        });
        await Task.WhenAll(tasks);

        var online = _devices.Count(d => d.IsOnline);
        BtnRefreshAll.IsEnabled = true;
        UpdateStatus($"{online}/{_devices.Count} device(s) online");
    }

    // ───── Network Scan ─────

    private async void BtnScan_Click(object sender, RoutedEventArgs e)
    {
        var subnet = TxtSubnet.Text.Trim();
        if (string.IsNullOrEmpty(subnet) || !IPAddress.TryParse(subnet, out _))
        {
            MessageBox.Show(
                "Enter a valid IP address to determine the /24 subnet to scan.\n" +
                "Example: 169.254.43.1",
                "Invalid Address", MessageBoxButton.OK, MessageBoxImage.Warning);
            return;
        }

        _scanCts = new CancellationTokenSource();
        BtnScan.IsEnabled = false;
        BtnCancelScan.IsEnabled = true;
        ScanProgress.Visibility = Visibility.Visible;
        ScanProgress.Value = 0;
        _scanResults.Clear();

        // Collect all /24 subnets to scan: entered IP + all configured device IPs
        var ipsToScan = new List<string> { subnet };
        ipsToScan.AddRange(_devices.Select(d => d.IpAddress));
        UpdateStatus("Scanning...");

        var progress = new Progress<int>(v => ScanProgress.Value = v);

        try
        {
            var results = await NetworkScanner.ScanSubnetAsync(ipsToScan, progress, _scanCts.Token);
            foreach (var r in results)
                _scanResults.Add(r);

            var arduinos = results.Count(r => r.IsArduino);
            UpdateStatus($"Scan complete — {results.Count} host(s) found, {arduinos} Arduino(s)");
        }
        catch (OperationCanceledException)
        {
            UpdateStatus("Scan cancelled");
        }
        finally
        {
            BtnScan.IsEnabled = true;
            BtnCancelScan.IsEnabled = false;
            ScanProgress.Visibility = Visibility.Collapsed;
            _scanCts = null;
        }
    }

    private void BtnCancelScan_Click(object sender, RoutedEventArgs e)
    {
        _scanCts?.Cancel();
    }

    // ───── Add from Scan ─────

    private void BtnAddFromScan_Click(object sender, RoutedEventArgs e)
    {
        if (ScanResultsList.SelectedItem is not ScanResult scanResult)
        {
            MessageBox.Show("Select a scan result to add.",
                "No Selection", MessageBoxButton.OK, MessageBoxImage.Information);
            return;
        }

        if (_devices.Any(d => d.IpAddress == scanResult.IpAddress))
        {
            MessageBox.Show("This device is already in the list.",
                "Duplicate", MessageBoxButton.OK, MessageBoxImage.Information);
            return;
        }

        var name = scanResult.IsArduino ? "Arduino" : "Device";
        _devices.Add(new Device
        {
            Name = name,
            Role = scanResult.IsArduino ? "XY Optical Drive" : "",
            IpAddress = scanResult.IpAddress,
            IsOnline = true,
            StatusInfo = scanResult.Info
        });
        SaveDevices();

        UpdateStatus($"'{name}' ({scanResult.IpAddress}) added");
    }
}
