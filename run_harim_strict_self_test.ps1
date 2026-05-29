param(
    [switch]$AcceptEula,
    [switch]$ShowGui,
    [switch]$SelfTestDebugBins,
    [int]$SelfTestFrames = 12000,
    [int]$Cycles = 1
)

$ErrorActionPreference = "Stop"

$ProjectRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$Runner = Join-Path $ProjectRoot "run_harim_demo.ps1"

if (-not (Test-Path -LiteralPath $Runner)) {
    throw "Harim demo runner was not found: $Runner"
}

$RunnerArgs = @{
    Cycles = $Cycles
    SelfTestFrames = $SelfTestFrames
    SelfTestMinPlacedBins = 8
    SelfTestMinTransferCycles = 1
    SelfTestMaxPreGripOffset = 0.05
    SelfTestMaxReturnReadyError = 0.05
    SelfTestMaxReleaseDrift = 0.005
    SelfTestMinReleaseRetreatLift = 0.20
    SelfTestRequireGripperOpenAfterRelease = $true
    SelfTestMaxStackLateralGap = 0.03
    SelfTestMaxStackSupportGap = 0.02
    SelfTestMinStackPalletMargin = 0.08
    SelfTestMinPayloadLift = 0.10
    SelfTestMaxDroppedPayloadDrift = 0.005
    SelfTestMinAmrExitClearance = 0.60
    SelfTestMaxLiftContactGap = 0.01
    SelfTestMinPalletTunnelClearance = 0.10
    SelfTestMinLiftForkInnerGap = 0.30
    SelfTestMaxDropSupportGap = 0.01
    SelfTestMinDropLaneClearance = 0.03
    SelfTestMinDropRunnerClearance = 0.05
    SelfTestMinDropForkClearance = 0.03
}

if (-not $ShowGui) {
    $RunnerArgs.Headless = $true
}

if ($AcceptEula) {
    $RunnerArgs.AcceptEula = $true
}

if ($SelfTestDebugBins) {
    $RunnerArgs.SelfTestDebugBins = $true
}

& $Runner @RunnerArgs
exit $LASTEXITCODE
