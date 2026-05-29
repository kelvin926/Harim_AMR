param(
    [switch]$Headless,
    [switch]$AcceptEula,
    [int]$Cycles = 0,
    [int]$StackCols = 2,
    [int]$StackRows = 2,
    [int]$StackLayers = 2,
    [int]$SelfTestFrames = 0
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

$ArgsList = @(
    $DemoScript,
    "--cycles", $Cycles,
    "--stack-cols", $StackCols,
    "--stack-rows", $StackRows,
    "--stack-layers", $StackLayers,
    "--self-test-frames", $SelfTestFrames
)

if ($Headless) {
    $ArgsList += "--headless"
}

& $PythonExe @ArgsList
