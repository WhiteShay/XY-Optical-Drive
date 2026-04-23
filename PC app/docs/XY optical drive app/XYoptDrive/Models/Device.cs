using System.ComponentModel;
using System.Runtime.CompilerServices;

namespace XYoptDrive.Models;

public class Device : INotifyPropertyChanged
{
    private string _name = "";
    private string _role = "";
    private string _ipAddress = "";
    private bool _isOnline;
    private string _statusInfo = "";

    public string Name
    {
        get => _name;
        set { _name = value; OnPropertyChanged(); }
    }

    public string Role
    {
        get => _role;
        set { _role = value; OnPropertyChanged(); }
    }

    public string IpAddress
    {
        get => _ipAddress;
        set { _ipAddress = value; OnPropertyChanged(); }
    }

    public bool IsOnline
    {
        get => _isOnline;
        set { _isOnline = value; OnPropertyChanged(); OnPropertyChanged(nameof(StatusText)); }
    }

    public string StatusInfo
    {
        get => _statusInfo;
        set { _statusInfo = value; OnPropertyChanged(); }
    }

    public string StatusText => IsOnline ? "Online" : "Offline";

    public event PropertyChangedEventHandler? PropertyChanged;

    protected void OnPropertyChanged([CallerMemberName] string? propertyName = null)
    {
        PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(propertyName));
    }
}
