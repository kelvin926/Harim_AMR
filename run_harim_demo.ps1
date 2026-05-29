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
    [int]$SelfTestMinTransferCycles = 0,
    [double]$SelfTestMaxPreGripOffset = 0.0,
    [double]$SelfTestMaxReturnReadyError = 0.0,
    [double]$SelfTestMaxReleaseDrift = 0.0,
    [switch]$SelfTestRequireGripperOpenAfterRelease,
    [double]$SelfTestMaxStackLateralGap = 0.0,
    [double]$SelfTestMaxStackSupportGap = 0.0,
    [double]$SelfTestMinPayloadLift = 0.0,
    [double]$SelfTestMaxDroppedPayloadDrift = 0.0,
    [double]$SelfTestMaxLiftContactGap = 0.0,
    [double]$SelfTestMinPalletTunnelClearance = 0.0,
    [double]$SelfTestMinLiftForkInnerGap = 0.0,
    [double]$SelfTestMaxDropSupportGap = 0.0,
    [double]$SelfTestMinDropLaneClearance = 0.0,
    [double]$SelfTestMinDropRunnerClearance = 0.0,
    [double]$SelfTestMinDropForkClearance = 0.0,
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
    "--self-test-min-transfer-cycles", $SelfTestMinTransferCycles,
    "--self-test-max-pre-grip-offset", $SelfTestMaxPreGripOffset,
    "--self-test-max-return-ready-error", $SelfTestMaxReturnReadyError,
    "--self-test-max-release-drift", $SelfTestMaxReleaseDrift,
    "--self-test-max-stack-lateral-gap", $SelfTestMaxStackLateralGap,
    "--self-test-max-stack-support-gap", $SelfTestMaxStackSupportGap,
    "--self-test-min-payload-lift", $SelfTestMinPayloadLift,
    "--self-test-max-dropped-payload-drift", $SelfTestMaxDroppedPayloadDrift,
    "--self-test-max-lift-contact-gap", $SelfTestMaxLiftContactGap,
    "--self-test-min-pallet-tunnel-clearance", $SelfTestMinPalletTunnelClearance,
    "--self-test-min-lift-fork-inner-gap", $SelfTestMinLiftForkInnerGap,
    "--self-test-max-drop-support-gap", $SelfTestMaxDropSupportGap,
    "--self-test-min-drop-lane-clearance", $SelfTestMinDropLaneClearance,
    "--self-test-min-drop-runner-clearance", $SelfTestMinDropRunnerClearance,
    "--self-test-min-drop-fork-clearance", $SelfTestMinDropForkClearance,
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

if ($SelfTestRequireGripperOpenAfterRelease) {
    $ArgsList += "--self-test-require-gripper-open-after-release"
}

& $PythonExe -u @ArgsList
$PythonExitCode = $LASTEXITCODE
if ($PythonExitCode -ne 0) {
    exit $PythonExitCode
}
exit 0
