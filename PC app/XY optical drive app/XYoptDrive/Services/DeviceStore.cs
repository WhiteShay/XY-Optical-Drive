using System;
using System.Collections.Generic;
using System.IO;
using System.Text.Json;
using XYoptDrive.Models;

namespace XYoptDrive.Services;

public static class DeviceStore
{
    private static readonly string FilePath = Path.Combine(
        AppDomain.CurrentDomain.BaseDirectory, "devices.json");

    public static List<Device> Load()
    {
        if (!File.Exists(FilePath))
            return new List<Device>();

        var json = File.ReadAllText(FilePath);
        if (string.IsNullOrWhiteSpace(json))
            return new List<Device>();

        return JsonSerializer.Deserialize<List<Device>>(json) ?? new List<Device>();
    }

    public static void Save(IEnumerable<Device> devices)
    {
        var options = new JsonSerializerOptions { WriteIndented = true };
        var json = JsonSerializer.Serialize(devices, options);
        File.WriteAllText(FilePath, json);
    }
}
