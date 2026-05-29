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
    [switch]$NoGif,
    [string]$GifOutputDir = "",
    [int]$GifFrameStride = 80,
    [int]$GifMaxFrames = 180,
    [int]$SelfTestMinPlacedBins = 0,
    [int]$SelfTestMinTransferCycles = 0,
    [double]$SelfTestMaxPreGripOffset = 0.0,
    [double]$SelfTestMaxReturnReadyError = 0.0,
    [double]$SelfTestMaxReleaseDrift = 0.0,
    [double]$SelfTestMinReleaseRetreatLift = 0.0,
    [int]$SelfTestMinScriptedPlaceCount = 0,
    [double]$SelfTestMaxScriptedPlaceError = 0.0,
    [double]$SelfTestMinReleaseSeparation = 0.0,
    [double]$SelfTestMinReleaseVerticalClearance = 0.0,
    [switch]$SelfTestRequireGripperOpenAfterRelease,
    [double]$SelfTestMaxStackLateralGap = 0.0,
    [double]$SelfTestMaxStackSupportGap = 0.0,
    [double]$SelfTestMinStackPalletMargin = 0.0,
    [int]$SelfTestMinLoadRestraintCount = 0,
    [double]$SelfTestMinLoadRestraintPalletMargin = 0.0,
    [double]$SelfTestMinInfeedConveyorLength = 0.0,
    [double]$SelfTestMinInfeedSpawnMargin = 0.0,
    [double]$SelfTestMinInfeedGuideClearance = 0.0,
    [double]$SelfTestMaxInfeedBeltSupportGap = 0.0,
    [int]$SelfTestMinInfeedMotionMarkerCount = 0,
    [double]$SelfTestMinInfeedMotionObservedTravel = 0.0,
    [int]$SelfTestMinSafetyFencePartCount = 0,
    [double]$SelfTestMinSafetyFenceAmrGateClearance = 0.0,
    [double]$SelfTestMinSafetyFenceInfeedGateClearance = 0.0,
    [int]$SelfTestMinAmrSafetyPartCount = 0,
    [double]$SelfTestMinAmrSafetyBeaconHeight = 0.0,
    [double]$SelfTestMinAmrSafetyScannerClearance = 0.0,
    [double]$SelfTestMaxAmrSafetyPoseError = 0.0,
    [int]$SelfTestMinAmrWarningIndicatorCount = 0,
    [int]$SelfTestMinAmrIdleIndicatorCount = 0,
    [int]$SelfTestMinAmrWarningObserved = 0,
    [int]$SelfTestMinAmrIdleObserved = 0,
    [int]$SelfTestMaxAmrIndicatorVisibilityMismatches = -1,
    [int]$SelfTestMinAmrDrivePartCount = 0,
    [double]$SelfTestMaxAmrDrivePoseError = 0.0,
    [double]$SelfTestMaxAmrWheelFloorGap = 0.0,
    [double]$SelfTestMaxAmrWheelFloorPenetration = 0.0,
    [double]$SelfTestMinPayloadLift = 0.0,
    [double]$SelfTestMaxDroppedPayloadDrift = 0.0,
    [int]$SelfTestMinDroppedStackItemCount = 0,
    [double]$SelfTestMaxDroppedStackPoseError = 0.0,
    [double]$SelfTestMaxDroppedStackSupportGap = 0.0,
    [double]$SelfTestMinDroppedStackPalletMargin = 0.0,
    [int]$SelfTestMinDroppedPalletPartCount = 0,
    [double]$SelfTestMaxDroppedPalletPartPoseError = 0.0,
    [double]$SelfTestMinAmrExitClearance = 0.0,
    [double]$SelfTestMaxLiftContactGap = 0.0,
    [double]$SelfTestMinPalletTunnelClearance = 0.0,
    [double]$SelfTestMinLiftForkInnerGap = 0.0,
    [double]$SelfTestMaxPickupHandoffXyError = 0.0,
    [double]$SelfTestMaxPickupHandoffLiftGap = 0.0,
    [double]$SelfTestMaxPickupHandoffLiftPenetration = 0.0,
    [double]$SelfTestMaxSlideOutYError = 0.0,
    [double]$SelfTestMaxSlideOutLiftGap = 0.0,
    [double]$SelfTestMaxSlideOutLiftPenetration = 0.0,
    [double]$SelfTestMaxDropSupportGap = 0.0,
    [double]$SelfTestMaxDropHandoffXyError = 0.0,
    [double]$SelfTestMaxDropHandoffSupportGap = 0.0,
    [double]$SelfTestMaxDropHandoffSupportPenetration = 0.0,
    [double]$SelfTestMinDropLaneClearance = 0.0,
    [double]$SelfTestMinDropRunnerClearance = 0.0,
    [double]$SelfTestMinDropForkClearance = 0.0,
    [int]$SelfTestMinDropDockStopCount = 0,
    [double]$SelfTestMaxDropDockStopGap = 0.0,
    [double]$SelfTestMinDropDockGuideClearance = 0.0,
    [double]$SelfTestMinDropDockForkClearance = 0.0,
    [int]$SelfTestMinPickupDockStopCount = 0,
    [double]$SelfTestMaxPickupDockStopGap = 0.0,
    [double]$SelfTestMinPickupDockGuideClearance = 0.0,
    [double]$SelfTestMinPickupDockForkClearance = 0.0,
    [double]$SelfTestMinPickupDockRunnerClearance = 0.0,
    [int]$SelfTestMinCameraCount = 0,
    [int]$SelfTestMinCameraRoleCount = 0,
    [double]$SelfTestMinCameraHeight = 0.0,
    [double]$SelfTestMinCameraTargetDistance = 0.0,
    [int]$SelfTestMinCameraDirectorSwitchCount = 0,
    [int]$SelfTestMinCameraDirectorRoleCount = 0,
    [int]$SelfTestMinWarehouseLightCount = 0,
    [int]$SelfTestMinWarehouseLightRoleCount = 0,
    [double]$SelfTestMinWarehouseLightHeight = 0.0,
    [double]$SelfTestMinWarehouseLightRouteSpan = 0.0,
    [double]$SelfTestMinWarehouseLightIntensity = 0.0,
    [int]$SelfTestMinAmrRouteGuardPartCount = 0,
    [double]$SelfTestMinAmrRouteGuardSpan = 0.0,
    [double]$SelfTestMinAmrRouteGuardClearance = 0.0,
    [double]$SelfTestMinAmrRouteBollardHeight = 0.0,
    [double]$SelfTestMaxLoadedRouteYError = 0.0,
    [double]$SelfTestMinLoadedRouteGuardClearance = 0.0,
    [double]$SelfTestMaxCarriedPalletPoseError = 0.0,
    [double]$SelfTestMaxCarriedPayloadPoseError = 0.0,
    [double]$SelfTestMinDropApproachStandoff = 0.0,
    [int]$SelfTestMinDropDockArrivalCount = 0,
    [double]$SelfTestMaxDropDockFinalError = 0.0,
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
    "--gif-frame-stride", $GifFrameStride,
    "--gif-max-frames", $GifMaxFrames,
    "--self-test-frames", $SelfTestFrames,
    "--self-test-min-placed-bins", $SelfTestMinPlacedBins,
    "--self-test-min-transfer-cycles", $SelfTestMinTransferCycles,
    "--self-test-max-pre-grip-offset", $SelfTestMaxPreGripOffset,
    "--self-test-max-return-ready-error", $SelfTestMaxReturnReadyError,
    "--self-test-max-release-drift", $SelfTestMaxReleaseDrift,
    "--self-test-min-release-retreat-lift", $SelfTestMinReleaseRetreatLift,
    "--self-test-min-scripted-place-count", $SelfTestMinScriptedPlaceCount,
    "--self-test-max-scripted-place-error", $SelfTestMaxScriptedPlaceError,
    "--self-test-min-release-separation", $SelfTestMinReleaseSeparation,
    "--self-test-min-release-vertical-clearance", $SelfTestMinReleaseVerticalClearance,
    "--self-test-max-stack-lateral-gap", $SelfTestMaxStackLateralGap,
    "--self-test-max-stack-support-gap", $SelfTestMaxStackSupportGap,
    "--self-test-min-stack-pallet-margin", $SelfTestMinStackPalletMargin,
    "--self-test-min-load-restraint-count", $SelfTestMinLoadRestraintCount,
    "--self-test-min-load-restraint-pallet-margin", $SelfTestMinLoadRestraintPalletMargin,
    "--self-test-min-infeed-conveyor-length", $SelfTestMinInfeedConveyorLength,
    "--self-test-min-infeed-spawn-margin", $SelfTestMinInfeedSpawnMargin,
    "--self-test-min-infeed-guide-clearance", $SelfTestMinInfeedGuideClearance,
    "--self-test-max-infeed-belt-support-gap", $SelfTestMaxInfeedBeltSupportGap,
    "--self-test-min-infeed-motion-marker-count", $SelfTestMinInfeedMotionMarkerCount,
    "--self-test-min-infeed-motion-observed-travel", $SelfTestMinInfeedMotionObservedTravel,
    "--self-test-min-safety-fence-part-count", $SelfTestMinSafetyFencePartCount,
    "--self-test-min-safety-fence-amr-gate-clearance", $SelfTestMinSafetyFenceAmrGateClearance,
    "--self-test-min-safety-fence-infeed-gate-clearance", $SelfTestMinSafetyFenceInfeedGateClearance,
    "--self-test-min-amr-safety-part-count", $SelfTestMinAmrSafetyPartCount,
    "--self-test-min-amr-safety-beacon-height", $SelfTestMinAmrSafetyBeaconHeight,
    "--self-test-min-amr-safety-scanner-clearance", $SelfTestMinAmrSafetyScannerClearance,
    "--self-test-max-amr-safety-pose-error", $SelfTestMaxAmrSafetyPoseError,
    "--self-test-min-amr-warning-indicator-count", $SelfTestMinAmrWarningIndicatorCount,
    "--self-test-min-amr-idle-indicator-count", $SelfTestMinAmrIdleIndicatorCount,
    "--self-test-min-amr-warning-observed", $SelfTestMinAmrWarningObserved,
    "--self-test-min-amr-idle-observed", $SelfTestMinAmrIdleObserved,
    "--self-test-max-amr-indicator-visibility-mismatches", $SelfTestMaxAmrIndicatorVisibilityMismatches,
    "--self-test-min-amr-drive-part-count", $SelfTestMinAmrDrivePartCount,
    "--self-test-max-amr-drive-pose-error", $SelfTestMaxAmrDrivePoseError,
    "--self-test-max-amr-wheel-floor-gap", $SelfTestMaxAmrWheelFloorGap,
    "--self-test-max-amr-wheel-floor-penetration", $SelfTestMaxAmrWheelFloorPenetration,
    "--self-test-min-payload-lift", $SelfTestMinPayloadLift,
    "--self-test-max-dropped-payload-drift", $SelfTestMaxDroppedPayloadDrift,
    "--self-test-min-dropped-stack-item-count", $SelfTestMinDroppedStackItemCount,
    "--self-test-max-dropped-stack-pose-error", $SelfTestMaxDroppedStackPoseError,
    "--self-test-max-dropped-stack-support-gap", $SelfTestMaxDroppedStackSupportGap,
    "--self-test-min-dropped-stack-pallet-margin", $SelfTestMinDroppedStackPalletMargin,
    "--self-test-min-dropped-pallet-part-count", $SelfTestMinDroppedPalletPartCount,
    "--self-test-max-dropped-pallet-part-pose-error", $SelfTestMaxDroppedPalletPartPoseError,
    "--self-test-min-amr-exit-clearance", $SelfTestMinAmrExitClearance,
    "--self-test-max-lift-contact-gap", $SelfTestMaxLiftContactGap,
    "--self-test-min-pallet-tunnel-clearance", $SelfTestMinPalletTunnelClearance,
    "--self-test-min-lift-fork-inner-gap", $SelfTestMinLiftForkInnerGap,
    "--self-test-max-pickup-handoff-xy-error", $SelfTestMaxPickupHandoffXyError,
    "--self-test-max-pickup-handoff-lift-gap", $SelfTestMaxPickupHandoffLiftGap,
    "--self-test-max-pickup-handoff-lift-penetration", $SelfTestMaxPickupHandoffLiftPenetration,
    "--self-test-max-slide-out-y-error", $SelfTestMaxSlideOutYError,
    "--self-test-max-slide-out-lift-gap", $SelfTestMaxSlideOutLiftGap,
    "--self-test-max-slide-out-lift-penetration", $SelfTestMaxSlideOutLiftPenetration,
    "--self-test-max-drop-support-gap", $SelfTestMaxDropSupportGap,
    "--self-test-max-drop-handoff-xy-error", $SelfTestMaxDropHandoffXyError,
    "--self-test-max-drop-handoff-support-gap", $SelfTestMaxDropHandoffSupportGap,
    "--self-test-max-drop-handoff-support-penetration", $SelfTestMaxDropHandoffSupportPenetration,
    "--self-test-min-drop-lane-clearance", $SelfTestMinDropLaneClearance,
    "--self-test-min-drop-runner-clearance", $SelfTestMinDropRunnerClearance,
    "--self-test-min-drop-fork-clearance", $SelfTestMinDropForkClearance,
    "--self-test-min-drop-dock-stop-count", $SelfTestMinDropDockStopCount,
    "--self-test-max-drop-dock-stop-gap", $SelfTestMaxDropDockStopGap,
    "--self-test-min-drop-dock-guide-clearance", $SelfTestMinDropDockGuideClearance,
    "--self-test-min-drop-dock-fork-clearance", $SelfTestMinDropDockForkClearance,
    "--self-test-min-pickup-dock-stop-count", $SelfTestMinPickupDockStopCount,
    "--self-test-max-pickup-dock-stop-gap", $SelfTestMaxPickupDockStopGap,
    "--self-test-min-pickup-dock-guide-clearance", $SelfTestMinPickupDockGuideClearance,
    "--self-test-min-pickup-dock-fork-clearance", $SelfTestMinPickupDockForkClearance,
    "--self-test-min-pickup-dock-runner-clearance", $SelfTestMinPickupDockRunnerClearance,
    "--self-test-min-camera-count", $SelfTestMinCameraCount,
    "--self-test-min-camera-role-count", $SelfTestMinCameraRoleCount,
    "--self-test-min-camera-height", $SelfTestMinCameraHeight,
    "--self-test-min-camera-target-distance", $SelfTestMinCameraTargetDistance,
    "--self-test-min-camera-director-switch-count", $SelfTestMinCameraDirectorSwitchCount,
    "--self-test-min-camera-director-role-count", $SelfTestMinCameraDirectorRoleCount,
    "--self-test-min-warehouse-light-count", $SelfTestMinWarehouseLightCount,
    "--self-test-min-warehouse-light-role-count", $SelfTestMinWarehouseLightRoleCount,
    "--self-test-min-warehouse-light-height", $SelfTestMinWarehouseLightHeight,
    "--self-test-min-warehouse-light-route-span", $SelfTestMinWarehouseLightRouteSpan,
    "--self-test-min-warehouse-light-intensity", $SelfTestMinWarehouseLightIntensity,
    "--self-test-min-amr-route-guard-part-count", $SelfTestMinAmrRouteGuardPartCount,
    "--self-test-min-amr-route-guard-span", $SelfTestMinAmrRouteGuardSpan,
    "--self-test-min-amr-route-guard-clearance", $SelfTestMinAmrRouteGuardClearance,
    "--self-test-min-amr-route-bollard-height", $SelfTestMinAmrRouteBollardHeight,
    "--self-test-max-loaded-route-y-error", $SelfTestMaxLoadedRouteYError,
    "--self-test-min-loaded-route-guard-clearance", $SelfTestMinLoadedRouteGuardClearance,
    "--self-test-max-carried-pallet-pose-error", $SelfTestMaxCarriedPalletPoseError,
    "--self-test-max-carried-payload-pose-error", $SelfTestMaxCarriedPayloadPoseError,
    "--self-test-min-drop-approach-standoff", $SelfTestMinDropApproachStandoff,
    "--self-test-min-drop-dock-arrival-count", $SelfTestMinDropDockArrivalCount,
    "--self-test-max-drop-dock-final-error", $SelfTestMaxDropDockFinalError,
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

if ($NoGif) {
    $ArgsList += "--no-gif"
}

if ($GifOutputDir -ne "") {
    $ArgsList += @("--gif-output-dir", $GifOutputDir)
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
