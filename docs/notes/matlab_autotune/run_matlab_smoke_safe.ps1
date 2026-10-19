param(
    [string]$MatlabExe = "D:\Program Files\MATLAB\R2016a\bin\matlab.exe",
    [int]$TimeoutSeconds = 60
)

$ErrorActionPreference = "Stop"

$RepoWsl = "/home/zoumaguanbei/SmartCar_Project/Loongson_2k300_301_Library-master/Loongson_2K300_301_LIB"
$WorkDir = "C:\tmp\lq_matlab_autotune_smoke"
$LogPath = "C:\tmp\lq_matlab_smoke.log"

if (-not (Test-Path -LiteralPath $MatlabExe)) {
    throw "MATLAB executable not found: $MatlabExe"
}

Remove-Item -LiteralPath $WorkDir -Recurse -Force -ErrorAction SilentlyContinue
New-Item -ItemType Directory -Path (Join-Path $WorkDir "samples") -Force | Out-Null

$Files = @(
    "docs/matlab_autotune/lq_autotune.m",
    "docs/matlab_autotune/run_autotune_smoke.m",
    "docs/matlab_autotune/samples/t11_sample.log",
    "docs/matlab_autotune/samples/t06_sample.log"
)

foreach ($File in $Files) {
    $Relative = $File -replace "^docs/matlab_autotune/", ""
    $Target = Join-Path $WorkDir $Relative
    $TargetDir = Split-Path -Parent $Target
    New-Item -ItemType Directory -Path $TargetDir -Force | Out-Null
    $Content = & wsl bash -lc "cd '$RepoWsl' && cat '$File'"
    if ($LASTEXITCODE -ne 0) {
        throw "Failed to copy from WSL: $File"
    }
    Set-Content -LiteralPath $Target -Value $Content -Encoding ASCII
}
Remove-Item -LiteralPath $LogPath -Force -ErrorAction SilentlyContinue

$MatlabCommand = "try, cd('$WorkDir'); run_autotune_smoke; catch ME, disp(getReport(ME,'extended')); exit(1); end; exit(0);"
$Args = @(
    "-nosplash",
    "-nodesktop",
    "-logfile", $LogPath,
    "-r", $MatlabCommand
)

Write-Host "[MATLAB_SAFE] starting $MatlabExe"
$StartTime = Get-Date
$Process = Start-Process -FilePath $MatlabExe -ArgumentList $Args -WindowStyle Hidden -PassThru
$Deadline = $StartTime.AddSeconds($TimeoutSeconds)

while ((Get-Date) -lt $Deadline) {
    Start-Sleep -Seconds 2
    $Process.Refresh()
    if ($Process.HasExited) {
        break
    }
    if (Test-Path -LiteralPath $LogPath) {
        $Text = Get-Content -LiteralPath $LogPath -Raw -ErrorAction SilentlyContinue
        if ($Text -match "AUTOTUNE_SMOKE" -or $Text -match "Error using|Undefined function|MATLAB:") {
            break
        }
    }
}

$Process.Refresh()
if (-not $Process.HasExited) {
    Write-Host "[MATLAB_SAFE] timeout or log-ready; stopping MATLAB process $($Process.Id)"
    Stop-Process -Id $Process.Id -Force -ErrorAction SilentlyContinue
    Get-Process MATLAB -ErrorAction SilentlyContinue |
        Where-Object { $_.StartTime -ge $StartTime } |
        Stop-Process -Force -ErrorAction SilentlyContinue
} else {
    Write-Host "[MATLAB_SAFE] MATLAB exited with code $($Process.ExitCode)"
}

if (Test-Path -LiteralPath $LogPath) {
    Write-Host "[MATLAB_SAFE] logfile: $LogPath"
    Get-Content -LiteralPath $LogPath -Tail 160
} else {
    Write-Host "[MATLAB_SAFE] no logfile was created"
}
