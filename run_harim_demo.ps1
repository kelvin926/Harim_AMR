param(
    [switch]$Headless,
    [switch]$AcceptEula,
    [int]$Cycles = 0,
    [int]$StackCols = 2,
    [int]$StackRows = 2,
    [int]$StackLayers = 2,
    [int]$SelfTestFrames = 0,
    [switch]$SelfTestForceStackComplete,
    [switch]$SelfTestDebugBins,
    [int]$SelfTestMinPlacedBins = 0,
    [double]$MoveSpeed = 0.65,
    [double]$PickupX = 0.82,
    [double]$PickupY = -0.31,
    [double]$DropX = 11.42,
    [double]$DropY = -0.31
)

$ErrorActionPreference = "Stop"

$ProjectRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$PythonExe = Join-Path $ProjectRoot ".conda\env_isaacsim_5_1_0\python.exe"
$DemoScript = Join-Path $ProjectRoot "isaac_sim\scripts\run_harim_pallet_demo.py"

if (-not (Test-Path -LiteralPath $PythonExe)) {
    throw "Isaac Sim conda environment was not found: $PythonExe"
}

if (-not (Test-Path -LiteralPath $DemoScript)) {
    throw "Demo script was not found: $DemoScript"
}

if ($AcceptEula) {
    $env:OMNI_KIT_ACCEPT_EULA = "YES"
}

$env:PYTHONUNBUFFERED = "1"

$ArgsList = @(
    $DemoScript,
    "--cycles", $Cycles,
    "--stack-cols", $StackCols,
    "--stack-rows", $StackRows,
    "--stack-layers", $StackLayers,
    "--self-test-frames", $SelfTestFrames,
    "--self-test-min-placed-bins", $SelfTestMinPlacedBins,
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

if ($SelfTestDebugBins) {
    $ArgsList += "--self-test-debug-bins"
}

& $PythonExe -u @ArgsList
