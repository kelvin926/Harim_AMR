param(
    [switch]$Headless,
    [switch]$AcceptEula,
    [int]$Cycles = 0,
    [int]$StackCols = 2,
    [int]$StackRows = 2,
    [int]$StackLayers = 2,
    [int]$SelfTestFrames = 0,
    [switch]$SelfTestForceStackComplete,
    [double]$MoveSpeed = 0.65,
    [double]$PickupX = 0.82,
    [double]$PickupY = -0.31,
    [double]$DropX = 11.42,
    [double]$DropY = -0.31,
    [string]$CapturePath = "",
    [string]$PythonExe = ""
)

$ErrorActionPreference = "Stop"

$ProjectRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
if ([string]::IsNullOrWhiteSpace($PythonExe)) {
    $PythonExe = Join-Path $ProjectRoot ".conda\env_isaacsim_5_1_0\python.exe"
}
$DemoScript = Join-Path $ProjectRoot "isaac_sim\scripts\run_harim_pallet_demo.py"

if (-not (Test-Path -LiteralPath $PythonExe)) {
    throw "Isaac Sim conda environment was not found: $PythonExe. Run setup first: powershell -ExecutionPolicy Bypass -File .\setup_isaacsim_env.ps1"
}

if (-not (Test-Path -LiteralPath $DemoScript)) {
    throw "Demo script was not found: $DemoScript"
}

if ($AcceptEula) {
    $env:OMNI_KIT_ACCEPT_EULA = "YES"
}

$ArgsList = @(
    $DemoScript,
    "--cycles", $Cycles,
    "--stack-cols", $StackCols,
    "--stack-rows", $StackRows,
    "--stack-layers", $StackLayers,
    "--self-test-frames", $SelfTestFrames,
    "--move-speed", $MoveSpeed,
    "--pickup-x", $PickupX,
    "--pickup-y", $PickupY,
    "--drop-x", $DropX,
    "--drop-y", $DropY
)

if ($Headless) {
    $ArgsList += "--headless"
}

if ($SelfTestForceStackComplete) {
    $ArgsList += "--self-test-force-stack-complete"
}

if (-not [string]::IsNullOrWhiteSpace($CapturePath)) {
    $ArgsList += "--capture-path"
    $ArgsList += $CapturePath
}

& $PythonExe @ArgsList
