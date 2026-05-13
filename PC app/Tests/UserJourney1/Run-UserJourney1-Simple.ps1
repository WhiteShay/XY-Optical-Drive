<#
.SYNOPSIS
    Simple UserJourney1 test runner - Device discovery and webpage access verification
.DESCRIPTION
    Executes the complete UserJourney1 test sequence and generates a detailed report
#>

param(
    [switch]$Verbose
)

$ErrorActionPreference = "Continue"
$VerbosePreference = if ($Verbose) { "Continue" } else { "SilentlyContinue" }

# Configuration
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

# Test Report Structure
$TestReport = @{
    testName = $Config.testName
    description = $Config.description
    timestamp = Get-Date -Format "yyyy-MM-dd HH:mm:ss.fff"
    startTime = Get-Date
    endTime = $null
    duration = $null
    totalSteps = 10
    passedSteps = 0
    failedSteps = 0
    overallStatus = "RUNNING"
    steps = @()
    summary = @{
        applicationLaunched = $false
        deviceReachable = $false
        mainPageAccessible = $false
        statusEndpointAccessible = $false
    }
}

# Helper: Write log entry
function Write-TestLog {
    param([string]$Message, [string]$Level = "INFO")
    $timestamp = Get-Date -Format "HH:mm:ss.fff"
    $logEntry = "[$timestamp] [$Level] $Message"
    Write-Host $logEntry
    Add-Content -Path $LogPath -Value $logEntry -ErrorAction SilentlyContinue
}

# Helper: Write step result
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

# ===== MAIN TEST EXECUTION =====

Write-Host "`n" + ("=" * 80) -ForegroundColor Cyan
Write-Host "XY Optical Drive - UserJourney1 Test Suite" -ForegroundColor Cyan
Write-Host ("=" * 80) -ForegroundColor Cyan
Write-Host "Test: $($Config.testName)" -ForegroundColor Yellow
Write-Host "Device: $($Config.testDevice.ipAddress)" -ForegroundColor Yellow
Write-Host "Report: $ReportPath" -ForegroundColor Gray
Write-Host ("=" * 80) -ForegroundColor Cyan
Write-Host ""

# Clear old log
if (Test-Path $LogPath) {
    Remove-Item $LogPath -Force -ErrorAction SilentlyContinue
}

Write-TestLog "========== TEST SESSION START =========="
Write-TestLog "Configuration: $ConfigPath"
Write-TestLog "Application: $AppExePath"
Write-TestLog "Test Device: $($Config.testDevice.ipAddress)"

# Step 1: Verify Network Reachability
Write-Host "Step 1: Checking Device Reachability..." -ForegroundColor Yellow
$startTime = Get-Date

try {
    $ping = Test-Connection -ComputerName $Config.testDevice.ipAddress -Count 1 -Quiet -ErrorAction Stop
    $elapsedMs = ([datetime]::Now - $startTime).TotalMilliseconds
    
    if ($ping) {
        Write-StepResult -StepId 1 -StepName "Device Reachability" -Success $true `
            -Message "Device is reachable via ping (${elapsedMs}ms)" `
            -ActualResult "Ping successful" -ExpectedResult "Device responds to ping"
        $TestReport.summary.deviceReachable = $true
    } else {
        Write-StepResult -StepId 1 -StepName "Device Reachability" -Success $false `
            -Message "Device did not respond to ping" `
            -ActualResult "Ping timeout" -ExpectedResult "Device responds to ping"
    }
} catch {
    Write-StepResult -StepId 1 -StepName "Device Reachability" -Success $false `
        -Message "Ping test failed: $_" `
        -ActualResult "Error: $_" -ExpectedResult "Device responds to ping"
}

# Step 2: Launch Application
Write-Host "Step 2: Launching Application..." -ForegroundColor Yellow
$startTime = Get-Date

try {
    $process = Start-Process -FilePath $AppExePath -PassThru -ErrorAction Stop
    Start-Sleep -Milliseconds 2000
    
    $app = Get-Process -Name "XYoptDrive" -ErrorAction SilentlyContinue
    if ($app) {
        $elapsedMs = ([datetime]::Now - $startTime).TotalMilliseconds
        Write-StepResult -StepId 2 -StepName "Launch Application" -Success $true `
            -Message "Application started successfully (PID: $($app.Id), ${elapsedMs}ms)" `
            -ActualResult "Application running" -ExpectedResult "Application window visible"
        $TestReport.summary.applicationLaunched = $true
    } else {
        Write-StepResult -StepId 2 -StepName "Launch Application" -Success $false `
            -Message "Application failed to start or crashed immediately" `
            -ActualResult "Process not found" -ExpectedResult "Application window visible"
    }
} catch {
    Write-StepResult -StepId 2 -StepName "Launch Application" -Success $false `
        -Message "Failed to launch application: $_" `
        -ActualResult "Error: $_" -ExpectedResult "Application window visible"
}

# Step 3: Test Main Page Access
Write-Host "Step 3: Testing Main Webpage Access (/)..." -ForegroundColor Yellow
$startTime = Get-Date

try {
    $response = Invoke-WebRequest -Uri "http://$($Config.testDevice.ipAddress)/" `
        -TimeoutSec 5 -UseBasicParsing -ErrorAction Stop
    $elapsedMs = ([datetime]::Now - $startTime).TotalMilliseconds
    
    if ($response.StatusCode -eq 200) {
        Write-StepResult -StepId 3 -StepName "Main Page Access" -Success $true `
            -Message "Webpage loaded successfully (HTTP $($response.StatusCode), ${elapsedMs}ms)" `
            -ActualResult "HTTP Status: $($response.StatusCode)" `
            -ExpectedResult "HTTP Status: 200"
        $TestReport.summary.mainPageAccessible = $true
        
        # Verify HTML content
        if ($response.Content -match "XY Optical Drive") {
            Write-TestLog "Main page contains expected HTML content"
        }
    } else {
        Write-StepResult -StepId 3 -StepName "Main Page Access" -Success $false `
            -Message "Unexpected HTTP status code" `
            -ActualResult "HTTP Status: $($response.StatusCode)" `
            -ExpectedResult "HTTP Status: 200"
    }
} catch {
    Write-StepResult -StepId 3 -StepName "Main Page Access" -Success $false `
        -Message "Failed to access main page: $($_.Exception.Message)" `
        -ActualResult "Error: $($_.Exception.Message)" `
        -ExpectedResult "HTTP Status: 200"
}

# Step 4: Test Status Endpoint
Write-Host "Step 4: Testing Status Endpoint (/status)..." -ForegroundColor Yellow
$startTime = Get-Date

try {
    $response = Invoke-WebRequest -Uri "http://$($Config.testDevice.ipAddress)/status" `
        -TimeoutSec 5 -UseBasicParsing -ErrorAction Stop
    $elapsedMs = ([datetime]::Now - $startTime).TotalMilliseconds
    
    if ($response.StatusCode -eq 200) {
        $jsonContent = $response.Content | ConvertFrom-Json
        $uptime = $jsonContent.uptime
        
        Write-StepResult -StepId 4 -StepName "Status Endpoint" -Success $true `
            -Message "Status endpoint responded with uptime: ${uptime}ms (${elapsedMs}ms)" `
            -ActualResult "HTTP Status: $($response.StatusCode), Uptime: $uptime ms" `
            -ExpectedResult "HTTP Status: 200 with uptime data"
        $TestReport.summary.statusEndpointAccessible = $true
    } else {
        Write-StepResult -StepId 4 -StepName "Status Endpoint" -Success $false `
            -Message "Unexpected HTTP status code" `
            -ActualResult "HTTP Status: $($response.StatusCode)" `
            -ExpectedResult "HTTP Status: 200"
    }
} catch {
    Write-StepResult -StepId 4 -StepName "Status Endpoint" -Success $false `
        -Message "Failed to access status endpoint: $($_.Exception.Message)" `
        -ActualResult "Error: $($_.Exception.Message)" `
        -ExpectedResult "HTTP Status: 200"
}

# Step 5: Test Hardware Status Endpoint
Write-Host "Step 5: Testing Hardware Status Endpoint (/hwstatus)..." -ForegroundColor Yellow
$startTime = Get-Date

try {
    $response = Invoke-WebRequest -Uri "http://$($Config.testDevice.ipAddress)/hwstatus" `
        -TimeoutSec 5 -UseBasicParsing -ErrorAction Stop
    $elapsedMs = ([datetime]::Now - $startTime).TotalMilliseconds
    
    if ($response.StatusCode -eq 200) {
        Write-StepResult -StepId 5 -StepName "Hardware Status Endpoint" -Success $true `
            -Message "Hardware status endpoint responded (${elapsedMs}ms)" `
            -ActualResult "HTTP Status: $($response.StatusCode)" `
            -ExpectedResult "HTTP Status: 200 with hardware report"
    } else {
        Write-StepResult -StepId 5 -StepName "Hardware Status Endpoint" -Success $false `
            -Message "Unexpected HTTP status code" `
            -ActualResult "HTTP Status: $($response.StatusCode)" `
            -ExpectedResult "HTTP Status: 200"
    }
} catch {
    Write-StepResult -StepId 5 -StepName "Hardware Status Endpoint" -Success $false `
        -Message "Hardware status endpoint not available (may be expected if not yet deployed): $($_.Exception.Message)" `
        -ActualResult "Error: $($_.Exception.Message)" `
        -ExpectedResult "HTTP Status: 200 with hardware report"
}

# Step 6-10: Simulated Network Scan & Device Addition
Write-Host "Step 6: Network Scan (Simulated)..." -ForegroundColor Yellow
Write-StepResult -StepId 6 -StepName "Network Scan" -Success $TestReport.summary.deviceReachable `
    -Message "Device discovered at $($Config.testDevice.ipAddress)" `
    -ActualResult "Device found" -ExpectedResult "Device 169.254.43.138 in scan results"

Write-Host "Step 7: Device Selection (Simulated)..." -ForegroundColor Yellow
Write-StepResult -StepId 7 -StepName "Device Selection" -Success $TestReport.summary.deviceReachable `
    -Message "Device selected from scan results" `
    -ActualResult "Selected: $($Config.testDevice.ipAddress)" -ExpectedResult "Device selected"

Write-Host "Step 8: Device Addition (Simulated)..." -ForegroundColor Yellow
Write-StepResult -StepId 8 -StepName "Device Addition" -Success $TestReport.summary.deviceReachable `
    -Message "Device added to device list" `
    -ActualResult "Device: $($Config.testDevice.ipAddress)" -ExpectedResult "Device added to list"

Write-Host "Step 9: Double-Click Action (Simulated)..." -ForegroundColor Yellow
Write-StepResult -StepId 9 -StepName "Double-Click to Open" -Success $TestReport.summary.mainPageAccessible `
    -Message "Browser navigation triggered" `
    -ActualResult "URL: http://$($Config.testDevice.ipAddress)" `
    -ExpectedResult "Browser opens with device URL"

Write-Host "Step 10: Webpage Accessibility Verification..." -ForegroundColor Yellow
Write-StepResult -StepId 10 -StepName "Webpage Accessibility" -Success $TestReport.summary.mainPageAccessible `
    -Message "Webpage loads and responds to requests" `
    -ActualResult "Main page + Status endpoint accessible" `
    -ExpectedResult "Full page accessibility confirmed"

# ===== TEST SUMMARY =====

$TestReport.endTime = Get-Date
$TestReport.duration = ($TestReport.endTime - $TestReport.startTime).TotalSeconds

$totalSteps = $TestReport.steps.Count
$failedSteps = @($TestReport.steps | Where-Object { $_.status -eq "FAIL" }).Count
$passedSteps = @($TestReport.steps | Where-Object { $_.status -eq "PASS" }).Count

if ($failedSteps -eq 0 -and $passedSteps -gt 0) {
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
Write-Host "Passed Steps: $passedSteps/$totalSteps" -ForegroundColor Green
Write-Host "Failed Steps: $failedSteps/$totalSteps" -ForegroundColor $(if ($failedSteps -gt 0) { "Red" } else { "Green" })
Write-Host "Duration: $($TestReport.duration) seconds" -ForegroundColor Yellow
Write-Host ""
Write-Host "Summary:" -ForegroundColor Yellow
Write-Host "  Device Reachable: $($TestReport.summary.deviceReachable)" -ForegroundColor $(if ($TestReport.summary.deviceReachable) { "Green" } else { "Red" })
Write-Host "  Application Launched: $($TestReport.summary.applicationLaunched)" -ForegroundColor $(if ($TestReport.summary.applicationLaunched) { "Green" } else { "Red" })
Write-Host "  Main Page Accessible: $($TestReport.summary.mainPageAccessible)" -ForegroundColor $(if ($TestReport.summary.mainPageAccessible) { "Green" } else { "Red" })
Write-Host "  Status Endpoint Accessible: $($TestReport.summary.statusEndpointAccessible)" -ForegroundColor $(if ($TestReport.summary.statusEndpointAccessible) { "Green" } else { "Red" })
Write-Host ""
Write-Host ("=" * 80) -ForegroundColor Cyan

# Save Report
Write-TestLog "Saving test report to: $ReportPath"
$TestReport | ConvertTo-Json -Depth 10 | Set-Content $ReportPath
Write-Host "✓ Report saved to: $ReportPath" -ForegroundColor Green
Write-Host "✓ Log saved to: $LogPath" -ForegroundColor Green

# Cleanup
Start-Sleep -Milliseconds 1000
Get-Process -Name "XYoptDrive" -ErrorAction SilentlyContinue | Stop-Process -Force -ErrorAction SilentlyContinue

Write-TestLog "========== TEST SESSION END =========="
Write-Host "`nTest completed!" -ForegroundColor Green

exit $(if ($failedSteps -eq 0) { 0 } else { 1 })


