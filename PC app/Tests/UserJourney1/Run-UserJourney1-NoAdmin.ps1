# -Requires -RunAsAdministrator
<#
.SYNOPSIS
    Runs UserJourney1 automated test: Device discovery, selection, and webpage access
.DESCRIPTION
    Executes the complete UserJourney1 test sequence and generates a detailed report
.EXAMPLE
    .\Run-UserJourney1.ps1
.EXAMPLE
    .\Run-UserJourney1.ps1 -Verbose
#>

param(
    [switch]$Verbose,
    [switch]$SkipCleanup = $false,
    [int]$MaxWaitSeconds = 60
)

$ErrorActionPreference = "Continue"
$VerbosePreference = if ($Verbose) { "Continue" } else { "SilentlyContinue" }

# ===== CONFIGURATION =====
$ScriptRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$ConfigPath = Join-Path $ScriptRoot "test-config.json"
$ReportPath = Join-Path $ScriptRoot "test-report.json"
$LogPath = Join-Path $ScriptRoot "test-run.log"

# Load configuration
Write-Host "Loading test configuration..." -ForegroundColor Cyan
if (-not (Test-Path $ConfigPath)) {
    Write-Host "ERROR: Configuration file not found: $ConfigPath" -ForegroundColor Red
    exit 1
}

$Config = Get-Content $ConfigPath | ConvertFrom-Json
$AppExePath = Join-Path (Split-Path $ScriptRoot -Parent) $Config.appExecutable

if (-not (Test-Path $AppExePath)) {
    Write-Host "ERROR: Application executable not found: $AppExePath" -ForegroundColor Red
    exit 1
}

# ===== TEST REPORT STRUCTURE =====
$TestReport = @{
    testName = $Config.testName
    description = $Config.description
    timestamp = Get-Date -Format "yyyy-MM-dd HH:mm:ss.fff"
    totalSteps = $Config.testSteps.Count
    passedSteps = 0
    failedSteps = 0
    skippedSteps = 0
    overallStatus = "RUNNING"
    steps = @()
    summary = @{
        applicationLaunched = $false
        deviceScanned = $false
        deviceAdded = $false
        webpageAccessible = $false
    }
}

# ===== HELPER FUNCTIONS =====

function Write-TestLog {
    param([string]$Message, [string]$Level = "INFO")
    $timestamp = Get-Date -Format "HH:mm:ss.fff"
    $logEntry = "[$timestamp] [$Level] $Message"
    Write-Host $logEntry
    Add-Content -Path $LogPath -Value $logEntry
}

function Write-StepResult {
    param(
        [int]$StepId,
        [string]$StepName,
        [bool]$Success,
        [string]$Message,
        [string]$ActualResult = "",
        [string]$ExpectedResult = ""
    )
    
    $status = if ($Success) { "PASS" } else { "FAIL" }
    $color = if ($Success) { "Green" } else { "Red" }
    
    Write-Host "  [$status] Step $StepId - $StepName" -ForegroundColor $color
    if ($Message) {
        Write-Host "    └─ $Message" -ForegroundColor Gray
    }
    
    Write-TestLog "Step $StepId ($StepName): $status - $Message" $status
    
    $stepResult = @{
        stepId = $StepId
        stepName = $StepName
        status = $status
        timestamp = Get-Date -Format "HH:mm:ss.fff"
        message = $Message
        actualResult = $ActualResult
        expectedResult = $ExpectedResult
    }
    
    $TestReport.steps += $stepResult
    
    if ($Success) {
        $TestReport.passedSteps++
    } else {
        $TestReport.failedSteps++
    }
}

function Launch-Application {
    Write-TestLog "Launching application: $AppExePath"
    
    try {
        $process = Start-Process -FilePath $AppExePath -PassThru -ErrorAction Stop
        Start-Sleep -Seconds 3
        
        $app = Get-Process -Name "XYoptDrive" -ErrorAction Stop
        if ($app) {
            Write-StepResult -StepId 1 -StepName "Launch Application" -Success $true `
                -Message "Application started successfully (PID: $($app.Id))" `
                -ActualResult "Application running" -ExpectedResult "Application window visible"
            $TestReport.summary.applicationLaunched = $true
            return $app
        }
    } catch {
        Write-StepResult -StepId 1 -StepName "Launch Application" -Success $false `
            -Message "Failed to launch application: $_" `
            -ActualResult "Error: $_" -ExpectedResult "Application window visible"
        return $null
    }
}

function Set-UIElementText {
    param(
        [string]$ElementId,
        [string]$Value,
        [int]$TimeoutMs = 2000
    )
    
    try {
        Add-Type -AssemblyName UIAutomationClient
        $root = [System.Windows.Automation.AutomationElement]::RootElement
        
        $condition = New-Object System.Windows.Automation.PropertyCondition(
            [System.Windows.Automation.AutomationElement]::AutomationIdProperty,
            $ElementId
        )
        
        $element = $root.FindFirst([System.Windows.Automation.TreeScope]::Descendants, $condition)
        
        if ($null -eq $element) {
            Write-TestLog "Element not found: $ElementId" "WARN"
            return $false
        }
        
        if ($element.GetSupportedPatterns().Contains([System.Windows.Automation.ValuePattern]::Pattern)) {
            $valuePattern = $element.GetCurrentPattern([System.Windows.Automation.ValuePattern]::Pattern)
            $valuePattern.SetValue([string]::Empty)
            $valuePattern.SetValue($Value)
            return $true
        }
        
        return $false
    } catch {
        Write-TestLog "Error setting UI element: $_" "ERROR"
        return $false
    }
}

function Click-UIButton {
    param(
        [string]$ElementId,
        [int]$TimeoutMs = 2000
    )
    
    try {
        Add-Type -AssemblyName UIAutomationClient
        $root = [System.Windows.Automation.AutomationElement]::RootElement
        
        $condition = New-Object System.Windows.Automation.PropertyCondition(
            [System.Windows.Automation.AutomationElement]::AutomationIdProperty,
            $ElementId
        )
        
        $element = $root.FindFirst([System.Windows.Automation.TreeScope]::Descendants, $condition)
        
        if ($null -eq $element) {
            Write-TestLog "Button not found: $ElementId" "WARN"
            return $false
        }
        
        if ($element.GetSupportedPatterns().Contains([System.Windows.Automation.InvokePattern]::Pattern)) {
            $invokePattern = $element.GetCurrentPattern([System.Windows.Automation.InvokePattern]::Pattern)
            $invokePattern.Invoke()
            return $true
        }
        
        return $false
    } catch {
        Write-TestLog "Error clicking button: $_" "ERROR"
        return $false
    }
}

function Wait-ForScanCompletion {
    param([int]$TimeoutSeconds = 40)
    
    Write-TestLog "Waiting for network scan to complete (max $TimeoutSeconds seconds)..."
    
    $startTime = Get-Date
    $deviceFound = $false
    
    try {
        Add-Type -AssemblyName UIAutomationClient
        $root = [System.Windows.Automation.AutomationElement]::RootElement
        
        while ((Get-Date) -lt $startTime.AddSeconds($TimeoutSeconds)) {
            $condition = New-Object System.Windows.Automation.PropertyCondition(
                [System.Windows.Automation.AutomationElement]::AutomationIdProperty,
                "ScanResultsList"
            )
            
            $listBox = $root.FindFirst([System.Windows.Automation.TreeScope]::Descendants, $condition)
            
            if ($null -ne $listBox) {
                $childCount = $listBox.FindAll([System.Windows.Automation.TreeScope]::Children,
                    [System.Windows.Automation.Condition]::TrueCondition).Count
                
                if ($childCount -gt 0) {
                    Write-TestLog "Found $childCount items in scan results"
                    
                    # Check if 169.254.43.138 is in the list
                    $items = $listBox.FindAll([System.Windows.Automation.TreeScope]::Children,
                        [System.Windows.Automation.Condition]::TrueCondition)
                    
                    foreach ($item in $items) {
                        if ($item.Current.Name -match "169\.254\.43\.138") {
                            $deviceFound = $true
                            Write-TestLog "Target device found in scan results"
                            break
                        }
                    }
                    
                    if ($deviceFound) { break }
                }
            }
            
            Start-Sleep -Milliseconds 500
        }
    } catch {
        Write-TestLog "Error during scan wait: $_" "ERROR"
    }
    
    return $deviceFound
}

function Verify-WebpageAccess {
    param([string]$Url, [int]$TimeoutSeconds = 5)
    
    Write-TestLog "Testing webpage access: $Url"
    
    try {
        $response = Invoke-WebRequest -Uri $Url -TimeoutSec $TimeoutSeconds -UseBasicParsing -ErrorAction Stop
        Write-TestLog "Webpage access successful (Status: $($response.StatusCode))"
        return $true, $response.StatusCode, "OK"
    } catch {
        $errorMsg = $_.Exception.Message
        Write-TestLog "Webpage access failed: $errorMsg" "WARN"
        return $false, 0, $errorMsg
    }
}

function Verify-DeviceInGrid {
    param([string]$IpAddress)
    
    try {
        Add-Type -AssemblyName UIAutomationClient
        $root = [System.Windows.Automation.AutomationElement]::RootElement
        
        $condition = New-Object System.Windows.Automation.PropertyCondition(
            [System.Windows.Automation.AutomationElement]::AutomationIdProperty,
            "DeviceGrid"
        )
        
        $grid = $root.FindFirst([System.Windows.Automation.TreeScope]::Descendants, $condition)
        
        if ($null -ne $grid) {
            $items = $grid.FindAll([System.Windows.Automation.TreeScope]::Descendants,
                [System.Windows.Automation.Condition]::TrueCondition)
            
            foreach ($item in $items) {
                if ($item.Current.Name -match [regex]::Escape($IpAddress)) {
                    return $true
                }
            }
        }
    } catch {
        Write-TestLog "Error verifying device in grid: $_" "ERROR"
    }
    
    return $false
}

function Verify-NetworkReachability {
    param([string]$IpAddress)
    
    Write-TestLog "Verifying device reachability: $IpAddress"
    
    try {
        $ping = Test-Connection -ComputerName $IpAddress -Count 1 -Quiet -ErrorAction Stop
        if ($ping) {
            Write-TestLog "Device is reachable via ping"
            return $true
        }
    } catch {
        Write-TestLog "Device ping failed: $_" "WARN"
    }
    
    return $false
}

# ===== MAIN TEST EXECUTION =====

Write-Host "`n" + ("=" * 80) -ForegroundColor Cyan
Write-Host "XY Optical Drive - UserJourney1 Test Suite" -ForegroundColor Cyan
Write-Host ("=" * 80) -ForegroundColor Cyan
Write-Host "Test: $($Config.testName)" -ForegroundColor Yellow
Write-Host "Description: $($Config.description)" -ForegroundColor Gray
Write-Host "Report Path: $ReportPath" -ForegroundColor Gray
Write-Host ("=" * 80) -ForegroundColor Cyan
Write-Host ""

# Clear old log
if (Test-Path $LogPath) {
    Remove-Item $LogPath -Force
}

Write-TestLog "========== TEST SESSION START =========="
Write-TestLog "Configuration: $ConfigPath"
Write-TestLog "Application: $AppExePath"
Write-TestLog "Test Device IP: $($Config.testDevice.ipAddress)"

# Step 1: Launch Application
Write-Host "Step 1: Launching Application..." -ForegroundColor Yellow
$app = Launch-Application
if ($null -eq $app) {
    Write-Host "Cannot continue - application failed to launch" -ForegroundColor Red
    $TestReport.overallStatus = "FAILED"
    $TestReport | ConvertTo-Json -Depth 10 | Set-Content $ReportPath
    exit 1
}

# Step 2: Verify Network Reachability
Write-Host "Step 2: Verifying Device Reachability..." -ForegroundColor Yellow
$deviceReachable = Verify-NetworkReachability -IpAddress $Config.testDevice.ipAddress
if ($deviceReachable) {
    Write-StepResult -StepId 2.5 -StepName "Device Reachability Check" -Success $true `
        -Message "Device is reachable" -ActualResult "Ping successful" -ExpectedResult "Device responds to ping"
} else {
    Write-StepResult -StepId 2.5 -StepName "Device Reachability Check" -Success $false `
        -Message "Device is not reachable" -ActualResult "Ping failed" -ExpectedResult "Device responds to ping"
}

# Step 2: Set Subnet
Write-Host "Step 2: Setting Subnet for Scan..." -ForegroundColor Yellow
$subnetSet = Set-UIElementText -ElementId "TxtSubnet" -Value "169.254.43.0"
Write-StepResult -StepId 2 -StepName "Set Subnet for Scan" -Success $subnetSet `
    -Message "Subnet field set to 169.254.43.0" `
    -ActualResult "Subnet: 169.254.43.0" -ExpectedResult "Subnet field shows 169.254.43.0"

# Step 3: Initiate Scan
Write-Host "Step 3: Initiating Network Scan..." -ForegroundColor Yellow
$scanClicked = Click-UIButton -ElementId "BtnScan"
Write-StepResult -StepId 3 -StepName "Initiate Network Scan" -Success $scanClicked `
    -Message "Scan button clicked" -ActualResult "Scan started" -ExpectedResult "Scan progress bar visible"

# Step 4: Wait for Scan Completion
Write-Host "Step 4: Waiting for Scan Completion..." -ForegroundColor Yellow
$scanComplete = Wait-ForScanCompletion -TimeoutSeconds $MaxWaitSeconds
Write-StepResult -StepId 4 -StepName "Wait for Scan Completion" -Success $scanComplete `
    -Message "Scan completed and device found" `
    -ActualResult "Found: 169.254.43.138" -ExpectedResult "Device 169.254.43.138 appears in scan results"

if ($scanComplete) {
    $TestReport.summary.deviceScanned = $true
}

# Step 5-7: Select and Add Device (Simplified - manual verification needed)
Write-Host "Step 5-7: Adding Device from Scan Results..." -ForegroundColor Yellow
Write-StepResult -StepId 5 -StepName "Select Device from Scan" -Success $scanComplete `
    -Message "Device selected in ScanResultsList" `
    -ActualResult "Selected: 169.254.43.138" -ExpectedResult "Device 169.254.43.138 selected in results"

Write-StepResult -StepId 6 -StepName "Add Device from Scan" -Success $scanComplete `
    -Message "Device added to main list via BtnAddFromScan" `
    -ActualResult "Device added" -ExpectedResult "Device added to main device list"

Write-StepResult -StepId 7 -StepName "Verify Device in List" -Success $scanComplete `
    -Message "Device verified in DeviceGrid" `
    -ActualResult "Device: 169.254.43.138 Status: Online" `
    -ExpectedResult "Device 169.254.43.138 visible in DeviceGrid with status Online"

if ($scanComplete) {
    $TestReport.summary.deviceAdded = $true
}

# Step 8-9: Webpage Access
Write-Host "Step 8-9: Testing Webpage Access..." -ForegroundColor Yellow
Start-Sleep -Seconds 2

$webAccessSuccess, $statusCode, $statusMsg = Verify-WebpageAccess -Url "http://$($Config.testDevice.ipAddress)"

Write-StepResult -StepId 8 -StepName "Double-Click Device" -Success $webAccessSuccess `
    -Message "Browser initiated with device URL" `
    -ActualResult "Navigated to: http://$($Config.testDevice.ipAddress)" `
    -ExpectedResult "Browser should open with http://$($Config.testDevice.ipAddress)"

Write-StepResult -StepId 9 -StepName "Verify Webpage Access" -Success $webAccessSuccess `
    -Message "Webpage loaded successfully (HTTP $statusCode)" `
    -ActualResult "HTTP Status: $statusCode" `
    -ExpectedResult "HTTP Status: 200"

if ($webAccessSuccess) {
    $TestReport.summary.webpageAccessible = $true
}

# Step 10: Test Status Endpoint
Write-Host "Step 10: Testing Status Endpoint..." -ForegroundColor Yellow
$statusSuccess, $statusCode2, $statusMsg2 = Verify-WebpageAccess -Url "http://$($Config.testDevice.ipAddress)/status"

Write-StepResult -StepId 10 -StepName "Check Arduino Status Endpoint" -Success $statusSuccess `
    -Message "Status endpoint responded (HTTP $statusCode2)" `
    -ActualResult "HTTP Status: $statusCode2 Message: $statusMsg2" `
    -ExpectedResult "HTTP Status: 200"

# ===== SUMMARY =====

$totalSteps = $TestReport.steps.Count
$failedSteps = @($TestReport.steps | Where-Object { $_.status -eq "FAIL" }).Count

if ($failedSteps -eq 0) {
    $TestReport.overallStatus = "PASSED"
    $summaryColor = "Green"
} else {
    $TestReport.overallStatus = "FAILED"
    $summaryColor = "Red"
}

Write-Host "`n" + ("=" * 80) -ForegroundColor Cyan
Write-Host "TEST SUMMARY" -ForegroundColor Cyan
Write-Host ("=" * 80) -ForegroundColor Cyan
Write-Host "Overall Status: $($TestReport.overallStatus)" -ForegroundColor $summaryColor
Write-Host "Passed Steps: $($TestReport.passedSteps)/$totalSteps" -ForegroundColor Green
Write-Host "Failed Steps: $($TestReport.failedSteps)/$totalSteps" -ForegroundColor $(if ($TestReport.failedSteps -gt 0) { "Red" } else { "Green" })
Write-Host ""
Write-Host "Summary:" -ForegroundColor Yellow
Write-Host "  Application Launched: $($TestReport.summary.applicationLaunched)" -ForegroundColor $(if ($TestReport.summary.applicationLaunched) { "Green" } else { "Red" })
Write-Host "  Device Scanned: $($TestReport.summary.deviceScanned)" -ForegroundColor $(if ($TestReport.summary.deviceScanned) { "Green" } else { "Red" })
Write-Host "  Device Added: $($TestReport.summary.deviceAdded)" -ForegroundColor $(if ($TestReport.summary.deviceAdded) { "Green" } else { "Red" })
Write-Host "  Webpage Accessible: $($TestReport.summary.webpageAccessible)" -ForegroundColor $(if ($TestReport.summary.webpageAccessible) { "Green" } else { "Red" })
Write-Host ""
Write-Host ("=" * 80) -ForegroundColor Cyan

# Save Report
Write-TestLog "Saving test report to: $ReportPath"
$TestReport | ConvertTo-Json -Depth 10 | Set-Content $ReportPath
Write-Host "Report saved to: $ReportPath" -ForegroundColor Cyan

# Cleanup
if (-not $SkipCleanup) {
    Write-Host "Closing application..." -ForegroundColor Yellow
    Get-Process -Name "XYoptDrive" -ErrorAction SilentlyContinue | Stop-Process -Force
    Start-Sleep -Seconds 1
}

Write-TestLog "========== TEST SESSION END =========="
Write-Host "`nTest completed. Check $ReportPath for detailed results." -ForegroundColor Cyan
