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
    SelfTestMinScriptedPlaceCount = 8
    SelfTestMaxScriptedPlaceError = 0.005
    SelfTestMinReleaseSeparation = 0.20
    SelfTestMinReleaseVerticalClearance = 0.35
    SelfTestRequireGripperOpenAfterRelease = $true
    SelfTestMaxStackLateralGap = 0.03
    SelfTestMaxStackSupportGap = 0.02
    SelfTestMinStackPalletMargin = 0.08
    SelfTestMinLoadRestraintCount = 6
    SelfTestMinLoadRestraintPalletMargin = 0.06
    SelfTestMinInfeedConveyorLength = 0.80
    SelfTestMinInfeedSpawnMargin = 0.30
    SelfTestMinInfeedGuideClearance = 0.40
    SelfTestMaxInfeedBeltSupportGap = 0.02
    SelfTestMinInfeedMotionMarkerCount = 6
    SelfTestMinInfeedMotionObservedTravel = 0.10
    SelfTestMinSafetyFencePartCount = 20
    SelfTestMinSafetyFenceAmrGateClearance = 0.25
    SelfTestMinSafetyFenceInfeedGateClearance = 0.20
    SelfTestMinAmrSafetyPartCount = 8
    SelfTestMinAmrSafetyBeaconHeight = 0.60
    SelfTestMinAmrSafetyScannerClearance = 0.10
    SelfTestMaxAmrSafetyPoseError = 0.005
    SelfTestMinAmrWarningIndicatorCount = 3
    SelfTestMinAmrIdleIndicatorCount = 2
    SelfTestMinAmrWarningObserved = 1
    SelfTestMinAmrIdleObserved = 1
    SelfTestMaxAmrIndicatorVisibilityMismatches = 0
    SelfTestMinAmrDrivePartCount = 6
    SelfTestMaxAmrDrivePoseError = 0.005
    SelfTestMaxAmrWheelFloorGap = 0.010
    SelfTestMaxAmrWheelFloorPenetration = 0.005
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
    SelfTestMinDropDockStopCount = 2
    SelfTestMaxDropDockStopGap = 0.05
    SelfTestMinDropDockGuideClearance = 0.10
    SelfTestMinDropDockForkClearance = 0.03
    SelfTestMinPickupDockStopCount = 2
    SelfTestMaxPickupDockStopGap = 0.05
    SelfTestMinPickupDockGuideClearance = 0.10
    SelfTestMinPickupDockForkClearance = 0.03
    SelfTestMinPickupDockRunnerClearance = 0.05
    SelfTestMinCameraCount = 4
    SelfTestMinCameraRoleCount = 4
    SelfTestMinCameraHeight = 1.25
    SelfTestMinCameraTargetDistance = 1.0
    SelfTestMinCameraDirectorSwitchCount = 4
    SelfTestMinCameraDirectorRoleCount = 4
    SelfTestMinWarehouseLightCount = 4
    SelfTestMinWarehouseLightRoleCount = 3
    SelfTestMinWarehouseLightHeight = 3.20
    SelfTestMinWarehouseLightRouteSpan = 8.00
    SelfTestMinWarehouseLightIntensity = 3000.0
    SelfTestMinAmrRouteGuardPartCount = 14
    SelfTestMinAmrRouteGuardSpan = 8.00
    SelfTestMinAmrRouteGuardClearance = 0.30
    SelfTestMinAmrRouteBollardHeight = 0.65
    SelfTestMinDropApproachStandoff = 0.90
    SelfTestMinDropDockArrivalCount = 1
    SelfTestMaxDropDockFinalError = 0.03
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
