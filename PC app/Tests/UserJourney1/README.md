# XY Optical Drive - UserJourney1 Test Suite

Automated test suite for validating the complete device discovery and webpage access workflow.

## Overview

**UserJourney1** simulates the following user actions:
1. Launch PC application
2. Scan network subnet (169.254.43.0)
3. Discover Arduino device (169.254.43.138)
4. Add device to configured device list
5. Double-click device to open webpage
6. Verify webpage loads successfully

## Files

- **test-config.json** - Test configuration and step definitions
- **Run-UserJourney1.ps1** - Main test runner script
- **test-report.json** - Generated test report (created after run)
- **test-run.log** - Detailed execution log (created after run)

## Quick Start

### From PowerShell (Administrator)
```powershell
cd "C:\Users\WhiteShay_TUF\OneDrive\Bureau\Etudes\INSA GE copy\GE5_25\PFE\QNS\Project XY stage\2-Software\XY Optical Drive\PC app\Tests\UserJourney1"

# Run test with default settings
.\Run-UserJourney1.ps1

# Run test with verbose output
.\Run-UserJourney1.ps1 -Verbose

# Run test and keep application open after completion
.\Run-UserJourney1.ps1 -SkipCleanup
```

### From Command Line
```cmd
powershell -ExecutionPolicy Bypass -File "C:\path\to\Run-UserJourney1.ps1"
```

## Requirements

- Windows 10/11
- PowerShell 5.1+
- Administrator privileges
- .NET 8.0 runtime
- Arduino device at 169.254.43.138 (reachable and web server running)

## Test Steps

| Step | Action | Verification |
|------|--------|--------------|
| 1 | Launch XYoptDrive.exe | Application window appears |
| 2 | Set subnet field to 169.254.43.0 | UI element updated |
| 3 | Click "Scan" button | Network scan initiates |
| 4 | Wait for scan completion | Device 169.254.43.138 found in results |
| 5 | Select device in scan results | Device highlighted |
| 6 | Click "Add from Scan" button | Device added to main list |
| 7 | Verify device in grid | Device shows as "Online" |
| 8 | Double-click device | Browser opens with http://169.254.43.138 |
| 9 | Verify webpage access | HTTP 200 status received |
| 10 | Test /status endpoint | JSON response received |

## Output

### Test Report (test-report.json)
```json
{
  "testName": "UserJourney1 - Device Discovery & Webpage Access",
  "overallStatus": "PASSED",
  "timestamp": "2026-05-13 16:15:30.123",
  "totalSteps": 10,
  "passedSteps": 10,
  "failedSteps": 0,
  "summary": {
    "applicationLaunched": true,
    "deviceScanned": true,
    "deviceAdded": true,
    "webpageAccessible": true
  },
  "steps": [ ... ]
}
```

### Test Log (test-run.log)
Detailed timestamp-logged execution trace with INFO, WARN, and ERROR messages.

## Troubleshooting

### "Application executable not found"
- Verify application is built in Debug mode
- Check path in test-config.json matches your setup

### "Element not found" errors
- Application may not be launching properly
- Ensure no other instance of XYoptDrive is running
- Run as Administrator

### "Device not found in scan results"
- Verify Arduino device is online and at 169.254.43.138
- Check network connectivity: `ping 169.254.43.138`
- Ensure Arduino web server is running

### "Webpage access failed"
- Verify device is reachable: `Test-Connection 169.254.43.138`
- Check Arduino logs for errors
- Ensure firmware has been updated with the fix

## Configuration

Edit **test-config.json** to:
- Change target device IP address
- Modify expected device name/role
- Add or modify test steps
- Adjust timeouts and wait periods

## Integration with CI/CD

To integrate with build pipelines:
```powershell
$report = Get-Content "test-report.json" | ConvertFrom-Json
if ($report.overallStatus -ne "PASSED") {
    exit 1
}
```

## Support

For issues, check the test-run.log for detailed error messages and timestamps.
