# Harim AMR Isaac Sim Demo

이 폴더는 `E:\Harim_AMR` 내부에 설치한 Isaac Sim 5.1.0 pip 환경만 사용합니다.
기존 `E:\isaac-sim-5.1.0`, `E:\IsaacLab`, 기존 conda env는 사용하지 않습니다.

## 실행

PowerShell에서 프로젝트 루트로 이동한 뒤 실행합니다.

```powershell
cd E:\Harim_AMR
powershell -ExecutionPolicy Bypass -File .\run_harim_demo.ps1
```

Headless 실행:

```powershell
powershell -ExecutionPolicy Bypass -File .\run_harim_demo.ps1 -Headless
```

NVIDIA Omniverse Kit EULA를 아직 수락하지 않은 환경에서는 첫 실행 때 확인 프롬프트가 뜹니다.
headless 또는 자동 실행에서 프롬프트를 받을 수 없으면, 내용을 확인하고 동의하는 경우에만 `-AcceptEula`를 명시적으로 붙입니다.

```powershell
powershell -ExecutionPolicy Bypass -File .\run_harim_demo.ps1 -AcceptEula
```

초기화만 짧게 확인하려면 `-SelfTestFrames`를 사용합니다.

```powershell
powershell -ExecutionPolicy Bypass -File .\run_harim_demo.ps1 -Headless -AcceptEula -SelfTestFrames 2
```

기본값은 `2 x 2 x 2` 적재 패턴이며, 한 사이클이 끝나면 계속 반복합니다.
`-Cycles 1`처럼 지정하면 해당 횟수만 완료한 뒤 시뮬레이션은 열린 상태로 대기합니다.
기본 하역 위치는 pickup 위치에서 X 방향으로 10.6 m 떨어진 지점입니다.

```powershell
powershell -ExecutionPolicy Bypass -File .\run_harim_demo.ps1 -Cycles 1 -StackCols 3 -StackRows 2 -StackLayers 2
```

검증용으로 AMR 이송 시퀀스를 빠르게 보고 싶으면 적재 완료를 강제로 넣고 이동 속도를 높일 수 있습니다.

```powershell
powershell -ExecutionPolicy Bypass -File .\run_harim_demo.ps1 -Headless -AcceptEula -SelfTestFrames 260 -SelfTestForceStackComplete -Cycles 1 -MoveSpeed 20
```

## 구현 구조

- 공식 UR10 palletizing Cortex 동작을 기반으로 컨베이어 유입, suction gripper pick, 팔레트 위 적재를 수행합니다.
- pip 설치 환경에서 interactive sample import에 의존하지 않도록 필요한 UR10 asset 경로와 bin 공급 task를 이 스크립트 안에 최소 구현했습니다.
- bin은 처음부터 upside-down orientation으로 스폰합니다. 공식 behavior의 `needs_flip` 경로를 타지 않도록 `NoFlipDispatch`로 `pick_bin -> place_bin`만 사용하고, stage 안의 flip 관련 prim은 invisible 처리합니다.
- UR10/background 및 `iw_hub` USD reference 뒤에는 stage asset loading이 끝날 때까지 기다립니다.
- Cortex behavior의 `stack_complete` 상태를 감시합니다.
- 적재 완료 후 custom orchestrator가 `iw_hub`를 pickup pose로 이동시킵니다.
- LiftUp 단계에서 팔레트와 적재물을 들어 올리는 연출을 수행합니다.
- `iw_hub/chassis/lift` prim이 있으면 실제 asset lift prim도 함께 움직이고, 없으면 visual lift plate만 사용합니다.
- 이동 중에는 팔레트와 적재된 bin assembly를 `iw_hub` 기준 offset으로 따라가게 합니다.
- Drop pose에서 LiftDown 후 팔레트/박스 assembly pose를 고정하고, `iw_hub`만 전방으로 슬라이드 이탈합니다.
- visual pallet은 중앙 하부 통로가 비어 있도록 배치해 AMR이 팔레트 밑으로 들어갈 때 뚫려 보이는 문제를 줄였습니다.
- `--cycles 0` 기본값은 무한 반복입니다.

## 주의

첫 실행 시 NVIDIA Isaac asset 다운로드와 shader cache 생성 때문에 시간이 오래 걸릴 수 있습니다.
NVIDIA EULA 확인이 필요한 환경에서는 Isaac Sim 첫 실행 단계에서 사용자 확인이 필요할 수 있습니다.

## 로컬 검증

Isaac Sim을 띄우지 않고 custom orchestrator FSM만 검증하려면 다음 unittest를 실행합니다.
현재 검증 범위는 단일 사이클, 무한 반복 reset, 실제 `iw_hub` lift prim 연동, stage loading wait helper, extension/import 의존성, headless self-test loop, 10 m 이상 이동, no-flip dispatch, 슬라이드 하역입니다.
또한 `CortexUr10` import 전에 `isaacsim.robot.surface_gripper` extension을 enable하는 순서를 검사합니다.

```powershell
cd E:\Harim_AMR
.\.conda\env_isaacsim_5_1_0\python.exe -m unittest .\isaac_sim\tests\test_harim_transfer_orchestrator.py
```

문법 검사:

```powershell
cd E:\Harim_AMR
.\.conda\env_isaacsim_5_1_0\python.exe -m py_compile .\isaac_sim\scripts\run_harim_pallet_demo.py .\isaac_sim\tests\test_harim_transfer_orchestrator.py
```

실제 Isaac Sim 5.1.0 headless 초기화 검증:

```powershell
cd E:\Harim_AMR
powershell -ExecutionPolicy Bypass -File .\run_harim_demo.ps1 -Headless -AcceptEula -SelfTestFrames 2 -Cycles 1
```

AMR transfer 시퀀스까지 빠르게 검증:

```powershell
cd E:\Harim_AMR
powershell -ExecutionPolicy Bypass -File .\run_harim_demo.ps1 -Headless -AcceptEula -SelfTestFrames 260 -SelfTestForceStackComplete -Cycles 1 -MoveSpeed 20
```

2026-05-29 기준 위 검증은 통과했습니다. headless 로그에서 `MOVE_TO_DROP`, `slide-released pallet assembly at drop pose`, `SLIDE_OUT_FROM_PALLET`, `completed transfer cycle 1`을 확인했습니다.

## 2026-05-29 경로/드롭 작업대 변경

- AMR은 이제 로봇팔 테이블 쪽에서 출발하지 않고 pickup 위치 기준 `+X` 방향의 먼 위치에서 접근합니다. 테이블 아래를 뚫고 지나가는 것처럼 보이는 경로를 피하기 위한 변경입니다.
- UR10 기본 예제에 포함된 `flip`, `pallet`, `pallet_holder` 계열 프림은 stage 로드 직후 비활성화합니다. 새로 만든 visual pallet만 남도록 하여 팔레트 겹침을 줄였습니다.
- 목표 위치에는 슬라이드형 팔레트 작업대를 추가했습니다. `DropSlideRail`, `DropSlideRoller`, `DropSlideLeg` 프림으로 구성되며, AMR이 팔레트를 내려놓고 앞으로 빠져나갈 때 팔레트가 작업대 위에 남는 장면을 보여줍니다.
- 현재 검증 범위는 unittest 16개, Python compile, 260-frame headless transfer self-test입니다.

## 2026-05-29 현실감 보강

- 팔레트 상판/러너/블록을 `FixedCuboid` 기반으로 바꾸고, 보이지 않는 `PalletTopSupport` 충돌 지지면을 추가했습니다. 기본 예제 팔레트를 제거해도 박스가 팔레트를 뚫고 떨어지는 현상을 줄이기 위한 변경입니다.
- 적재 완료 직후 stacked bin pose를 잠그고 속도를 0으로 유지합니다. 설명용 데모에서 박스가 물리적으로 무너지거나 바닥으로 빠지는 것보다, 팔레트 위에 안정적으로 적재된 상태를 우선합니다.
- AMR 기본 높이를 warehouse floor 기준 `WORLD_FLOOR_Z`에 맞췄습니다.
- 실제 `iw_hub/chassis/lift` prim이 존재하면 보조 visual lift plate는 숨깁니다. 리프트가 실제 asset과 따로 공중에 떠 보이는 문제를 줄이기 위한 처리입니다.
- 드롭 작업대도 레일/다리/상단 지지면에 고정 충돌체를 추가했습니다.
- 현재 검증 범위는 unittest 20개, Python compile, 260-frame headless realism self-test입니다.

## 2026-05-29 UR10 흡착 해제 보강

- 공식 suction close 판정에 무한 대기하지 않도록 데모용 `DemoAttachBin` / `DemoReleaseBin`을 추가했습니다.
- pick과 place를 별도 decision 전환에 맡기지 않고 `DemoPickAndPlaceBin` 하나의 locked sequence로 묶었습니다. 집기, 들어올리기, place 방향 이동, open gripper, 완료 표시가 한 흐름으로 진행됩니다.
- 이동 중 active bin이 새 박스로 바뀌지 않도록 실제로 집은 박스를 `demo_carried_bin`으로 고정합니다.
- headless 확인용 `-SelfTestMinPlacedBins` 옵션을 추가했습니다.

UR10이 실제로 놓는지 확인:

```powershell
cd E:\Harim_AMR
powershell -ExecutionPolicy Bypass -File .\run_harim_demo.ps1 -Headless -AcceptEula -SelfTestFrames 1800 -SelfTestMinPlacedBins 1 -Cycles 1
```

확인된 로그:

- `<close gripper>`
- `[HarimDemo] demo-attached bin_0`
- `[HarimDemo] reach_place timed release`
- `<open gripper>`
- `[HarimDemo] demo-placed bin_0 at [1.05, -0.62, -0.51]`
- `[HarimDemo] self-test completed after 1800 frames`

## 2026-05-29 다중 박스 적재와 GUI release 보강

GUI에서 로봇팔이 박스를 놓지 않는 것처럼 보이는 문제를 줄이기 위해 release를 한 프레임 명령이 아니라 0.35초 동안 유지되는 state로 바꿨습니다. 이 동안 suction gripper `open()`을 반복 호출하고 박스를 목표 적재 좌표에 고정합니다.

또한 1번째 박스 완료 직후 다음 active bin이 이미 잡혀도 `DemoPickAndPlaceBin` state machine이 다시 시작되도록 수정했습니다. 컨베이어는 로봇팔이 active/carried bin을 처리 중일 때 추가 박스를 계속 흘려 보내지 않고, active bin은 고정 픽업 스테이션에 정렬됩니다.

다중 박스 확인:

```powershell
cd E:\Harim_AMR
powershell -ExecutionPolicy Bypass -File .\run_harim_demo.ps1 -Headless -AcceptEula -SelfTestFrames 4200 -SelfTestMinPlacedBins 2 -SelfTestDebugBins -Cycles 1
```

확인된 결과:

- `bin_0`, `bin_1`이 순차적으로 active bin이 됨
- 각 박스에 대해 `reach_pick`, `demo-attached`, `reach_place`, `<open gripper>`, `demo-placed` 로그가 이어짐
- 4200프레임 검증에서 `placed_bins=4`까지 확인됨

참고: 현재 데모는 GUI release 안정성을 우선해서 실제 `suction_gripper.close()` joint를 만들지 않고, `demo_carried_bin` 기반 scripted suction attach/release로 진행합니다.

추가 보강: GUI에서 박스가 여전히 그리퍼에 붙어 보이는 경우를 줄이기 위해 release 시점에 `force_open_suction_gripper()`를 호출합니다. 이 함수는 high-level `suction_gripper.open()`뿐 아니라 surface gripper internal interface의 `open_gripper()`도 직접 호출합니다. 또한 `DemoReleaseBin.enter()`에서 release 대상의 `demo_attached`, `demo_attach_T`, `is_attached`를 먼저 끊고, 그 뒤 목표 stack pose로 고정합니다.

최신 검증:

```powershell
cd E:\Harim_AMR
.\.conda\env_isaacsim_5_1_0\python.exe -m py_compile isaac_sim\scripts\run_harim_pallet_demo.py
.\.conda\env_isaacsim_5_1_0\python.exe -m unittest isaac_sim.tests.test_harim_transfer_orchestrator
powershell -ExecutionPolicy Bypass -File .\run_harim_demo.ps1 -Headless -AcceptEula -SelfTestFrames 12000 -SelfTestMinPlacedBins 8 -SelfTestMinTransferCycles 1 -SelfTestMaxPreGripOffset 0.05 -SelfTestMaxReturnReadyError 0.05 -SelfTestMaxReleaseDrift 0.005 -SelfTestMinPayloadLift 0.10 -SelfTestMaxDroppedPayloadDrift 0.005 -SelfTestDebugBins -Cycles 1
```

확인 결과: `placed_bins=8`, `transfer_cycles=1`, `max_pre_grip_offset=0.0049`, `max_return_ready_error=0.0396`, `max_release_drift=0.0000`, `max_payload_lift=0.1100`, `max_dropped_payload_drift=0.0000`.

추가 안정화:

- release 순간 gripper 상태를 직접 self-test gate로 확인합니다.
  - PowerShell: `-SelfTestRequireGripperOpenAfterRelease`
  - Python: `--self-test-require-gripper-open-after-release`
- release 후에는 `DemoTimedArmJointSettle`이 0.65초 동안 기본 관절 자세 쪽으로 부드럽게 보간한 뒤 `return_ready`로 들어갑니다. 후반부 박스에서 arm 자세가 꼬여 다음 pick offset이 커지는 run을 줄이기 위한 동작입니다.

최신 full 검증 결과: `placed_bins=8`, `transfer_cycles=1`, `max_pre_grip_offset=0.0046`, `max_return_ready_error=0.0399`, `max_release_drift=0.0000`, `release_gripper_samples=8`, `release_gripper_not_open=0`, `release_gripped_object_max=0`, `release_gripper_probe_failures=0`, `joint_settle_count=8`, `max_payload_lift=0.1100`, `max_dropped_payload_drift=0.0000`.

stack geometry gate도 추가했습니다. `-SelfTestMaxStackLateralGap`은 인접 박스 간 X/Y air gap을 제한하고, `-SelfTestMaxStackSupportGap`은 박스가 팔레트 또는 아래층 박스 위에서 과도하게 떠 있는지 제한합니다. 최신 full 검증에서는 `max_stack_lateral_gap=0.0200`, `min_stack_lateral_gap=0.0100`, `max_stack_support_gap=0.0100`, `min_stack_support_gap=0.0025`로 통과했습니다.

## 2026-05-29 pre-grip 정렬 추가 보강

`ReachToPick` 직후 `DemoSettleBinAtGripper`가 active bin의 grasp frame을 UR10 end-effector frame에 맞춰 최소 0.30초 동안 보간합니다. GUI에서 박스가 그리퍼에 계속 붙어 보이거나 release 뒤 다시 끌려가는 현상을 줄이기 위해 실제 surface gripper close는 기본 경로에서 제외하고 scripted attach/release만 사용합니다.

추가 확인:

```powershell
cd E:\Harim_AMR
powershell -ExecutionPolicy Bypass -File .\run_harim_demo.ps1 -Headless -AcceptEula -SelfTestFrames 4200 -SelfTestMinPlacedBins 4 -SelfTestDebugBins -Cycles 1
```

2026-05-29 확인 결과: `placed_bins=4`까지 통과했고, AMR transfer self-test도 `completed transfer cycle 1`까지 통과했습니다.

## 2026-05-29 GUI release 최종 보강

GUI에서 로봇팔이 박스를 놓지 않는 것처럼 보이는 문제를 기준으로 attach/release를 더 보수적으로 바꿨습니다.

- pick 직후 settle 중인 bin을 `demo_pre_grip_bin`으로 고정해 중간에 `active_bin`이 비거나 다른 bin으로 바뀌어도 release 대상이 바뀌지 않게 했습니다.
- 실제 `suction_gripper.close()`는 호출하지 않습니다. surface gripper joint가 release 뒤에도 박스를 붙잡아 보이는 경로를 없애고, scripted suction attach/release로만 박스를 이동합니다.
- attach 전에는 `suction_gripper.open()`을 호출하고, release 중에는 0.35초 동안 `open()`을 반복 호출합니다.
- 박스는 컨베이어 pick window 근처에서 같은 upside-down orientation으로 정렬 스폰됩니다.
- `return_ready`는 full pose가 아니라 position-only 명령과 기본 posture bias를 사용합니다. place 자세의 orientation에 묶여 다음 pick으로 복귀하지 못하는 문제를 줄이기 위한 변경입니다.

확인 명령:

```powershell
cd E:\Harim_AMR
.\.conda\env_isaacsim_5_1_0\python.exe -m unittest .\isaac_sim\tests\test_harim_transfer_orchestrator.py
.\.conda\env_isaacsim_5_1_0\python.exe -m py_compile .\isaac_sim\scripts\run_harim_pallet_demo.py .\isaac_sim\tests\test_harim_transfer_orchestrator.py
powershell -ExecutionPolicy Bypass -File .\run_harim_demo.ps1 -Headless -AcceptEula -SelfTestFrames 5600 -SelfTestMinPlacedBins 4 -SelfTestDebugBins -Cycles 1
powershell -ExecutionPolicy Bypass -File .\run_harim_demo.ps1 -Headless -AcceptEula -SelfTestFrames 260 -SelfTestForceStackComplete -Cycles 1 -MoveSpeed 20
```

확인 결과:

- unittest 21개 통과
- Python compile 통과
- UR10 5600-frame self-test 통과: `self-test completed after 5600 frames; placed_bins=4`
- AMR 260-frame transfer self-test 통과: `completed transfer cycle 1`

## 2026-05-29 전체 적재 후 AMR 이송 검증

기본 `2 x 2 x 2` 적재를 모두 끝낸 뒤 AMR이 팔레트를 가져가는 전체 흐름을 검증하도록 self-test를 보강했습니다.

- UR10 state timer는 wall-clock이 아니라 simulation dt 기반 `demo_sim_time`을 사용합니다. headless 실행 부하에 따라 같은 frame 수에서 결과가 달라지는 문제를 줄이기 위한 변경입니다.
- `-SelfTestMinTransferCycles` 옵션을 추가했습니다. 이제 placed bin 개수뿐 아니라 AMR transfer 완료 횟수도 self-test gate로 확인할 수 있습니다.
- pick/place/return timing을 줄여 전체 팔레타이징 사이클이 더 빠르고 일관되게 진행됩니다.
- `stack_complete` 이후에는 컨베이어가 추가 박스를 스폰하지 않습니다. 적재 완료 뒤 AMR 접근 중 불필요한 새 박스가 나오는 장면을 제거했습니다.

전체 end-to-end 확인:

```powershell
cd E:\Harim_AMR
powershell -ExecutionPolicy Bypass -File .\run_harim_demo.ps1 -Headless -AcceptEula -SelfTestFrames 7000 -SelfTestMinPlacedBins 8 -SelfTestMinTransferCycles 1 -SelfTestDebugBins -Cycles 1
```

확인 결과:

- `stack-count 8/8 after bin_7`
- `stack_complete detected`
- `attached 8 stacked items and 12 pallet parts`
- `slide-released pallet assembly at drop pose`
- `completed transfer cycle 1`
- `self-test completed after 7000 frames; placed_bins=8; transfer_cycles=1`

## 2026-05-29 AMR 이동 보간 보강

AMR waypoint 이동은 이제 시작점과 목표점을 저장한 뒤 `smoothstep()` S-curve로 보간합니다. 기존처럼 매 frame 같은 거리만 전진하는 방식보다 출발과 정지가 부드럽게 보입니다.

- `move_start_pose`, `move_target`, `move_duration`을 waypoint 전환 시 저장합니다.
- 평균 이동 시간은 `distance / move_speed`를 유지합니다.
- UR10 pre-grip settle도 같은 `smoothstep()` helper를 사용합니다.
- 단위 테스트에서 이동 25% 시점의 AMR 위치가 선형 이동보다 덜 진행되는지 확인합니다.

확인 결과:

- unittest 22개 통과
- Python compile 통과
- 7000-frame full end-to-end self-test 통과: `placed_bins=8; transfer_cycles=1`

## 2026-05-29 적재 좌표 정렬 보강

공식 `ReachToPlace`는 아래 박스와 맞추기 위해 `context.stack_coordinates`를 조금씩 수정할 수 있습니다. 이 때문에 위층 일부 박스가 canonical grid에서 몇 mm 정도 벗어나는 로그가 나왔습니다. 데모에서는 팔레트 위 격자 적재가 깔끔하게 보여야 하므로 release 좌표를 별도로 보존한 canonical grid에 스냅하도록 바꿨습니다.

- `clone_stack_coordinates()`로 stack coordinate를 deep-copy합니다.
- `context.stack_coordinates`는 Cortex behavior가 사용하고, `demo_stack_coordinates`는 release 스냅 기준으로 사용합니다.
- reset cycle 때 두 좌표 목록을 모두 새 copy로 복구합니다.
- 단위 테스트에서 deep-copy가 유지되는지 확인합니다.

확인 결과:

- unittest 23개 통과
- Python compile 통과
- 7000-frame full end-to-end self-test 통과
- `bin_0`부터 `bin_7`까지 모두 canonical grid 좌표에 place됨

## 2026-05-29 GUI release 상태 정리 및 박스 외형 보강

GUI에서 로봇팔이 박스를 놓지 않는 것처럼 보이는 경로를 줄이기 위해 `DemoReleaseBin`이 place 직후 carry 상태를 즉시 끊도록 보강했습니다.

- release 직후 `active_bin`, `demo_carried_bin`, `demo_pre_grip_bin`을 비웁니다.
- 완료 마킹용으로만 `demo_released_bin`을 잠시 보존하고, `DemoMarkCarriedBinComplete` 이후 비웁니다.
- `demo_released_bin`이 남아 있는 동안 새 박스 스폰을 막아 arm retreat 전에 다음 박스가 끼어들지 않게 했습니다.
- release 후 팔을 `POST_RELEASE_CLEARANCE_LIFT = 0.22` m 올려 GUI에서 분리가 더 명확히 보이게 했습니다.
- KLT collision/grasp는 유지하고 child `VisualCuboid` 2개로 `HarimCartonBody`, `HarimCartonTopTape` carton overlay를 추가했습니다.

확인 결과:

- unittest 24개 통과
- Python compile 통과
- 7000-frame full end-to-end self-test 통과: `placed_bins=8; transfer_cycles=1`
- 로그에서 `bin_0`부터 `bin_7`까지 `demo-placed`, `stack-count 8/8 after bin_7`, `active-bin -> None` 확인

## 2026-05-29 적재 완료 신호등 및 pre-grip 현실감 게이트

장면 안에 적재 완료 신호등을 추가했습니다. 적재 중에는 빨간 불, `stack_complete` 감지 뒤에는 초록 불로 바뀌어 AMR 호출 타이밍이 화면에서도 보입니다.

- `CompletionSignalController`가 red/green light visibility를 전환합니다.
- `ARM_CLEAR_SETTLE_TIME = 1.8`초를 둬서 적재 완료 직후 로봇팔이 물러난 다음 AMR이 접근합니다.
- `restore_demo_carried_active_bin()`으로 공식 `ReachToPlace` 중 `active_bin`이 비는 경우를 복원합니다.
- pick/return 시간을 늘려 후반 박스에서도 그리퍼 근처에서 attach되도록 조정했습니다.
- `--self-test-max-pre-grip-offset` / `-SelfTestMaxPreGripOffset` 옵션으로 pre-grip 보정량을 회귀 테스트합니다.

확인 결과:

- unittest 28개 통과
- Python compile 통과
- 8000-frame full end-to-end self-test 통과
- `max_pre_grip_offset=0.0049` m로 5 cm 게이트 통과

## 2026-05-29 return-ready 도달 판정 및 실패 exit 보강

후반 박스에서 팔이 pick-ready 위치로 충분히 돌아오기 전에 다음 pick이 시작되면 pre-grip 보정량이 커져 GUI에서 박스가 순간 보정되는 것처럼 보일 수 있습니다. 이를 줄이기 위해 `return_ready`를 시간만으로 끝내지 않고 end-effector 위치 오차 기준으로 종료하도록 바꿨습니다.

- `RETURN_READY_POSITION_THRESHOLD = 0.04` m 이하일 때 `return_ready reached`로 다음 박스로 넘어갑니다.
- `RETURN_READY_DURATION = 5.0`초까지 기다릴 수 있게 했습니다.
- self-test 실패와 simulation exception은 `os._exit(1)`로 종료해 외부 PowerShell에서도 실패 exit code가 보존됩니다.

확인 결과:

- 실패 probe: `$LASTEXITCODE=1`
- unittest 28개 통과
- Python compile 통과
- 12000-frame full end-to-end self-test 통과
- `max_pre_grip_offset=0.0050` m로 5 cm 게이트 통과

## 2026-05-29 return-ready 오차 self-test 게이트

`return_ready` 완료 시점의 위치 오차도 self-test gate로 확인합니다. 이동 중간 오차가 아니라 `return_ready reached` 또는 `timed release`로 상태가 끝나는 시점의 최종 오차만 기록합니다.

- 새 옵션: `--self-test-max-return-ready-error`
- PowerShell wrapper 옵션: `-SelfTestMaxReturnReadyError`
- 완료 로그에 `max_return_ready_error`를 출력합니다.

확인 결과:

- 실패 probe: `max return-ready error 0.0395 m exceeded 0.0010 m`, `$LASTEXITCODE=1`
- 12000-frame full end-to-end self-test 통과
- `max_pre_grip_offset=0.0050`, `max_return_ready_error=0.0398`

## 2026-05-29 GUI release hold 보강

GUI에서 로봇팔이 박스를 놓지 않고 계속 들고 가는 것처럼 보이는 문제를 기준으로, release 후 박스를 한 번만 스냅하지 않고 팔이 빠지는 동안에도 목표 적재 좌표에 계속 고정하도록 보강했습니다.

- `mark_demo_bin_released()`로 release 대상 bin에 `demo_release_target_p/q`를 저장합니다.
- `hold_demo_released_bin_at_target()`가 `demo_released_bin`을 stack 좌표에 반복 고정하고, active/carry 상태를 계속 끊습니다.
- post-release lift, return-ready, decider loop, frame step 전후에서 release hold를 호출해 GUI 렌더 프레임에서도 박스가 팔을 따라 움직이지 않게 했습니다.
- `--self-test-max-release-drift` / `-SelfTestMaxReleaseDrift` 옵션을 추가해 release 후 박스 drift를 self-test gate로 확인합니다.

확인 결과:

- unittest 28개 통과
- Python compile 통과
- 5200-frame release hold gate 통과: `placed_bins=8`, `max_release_drift=0.0000`
- 12000-frame full end-to-end self-test 통과: `placed_bins=8; transfer_cycles=1; max_pre_grip_offset=0.0050; max_return_ready_error=0.0395; max_release_drift=0.0000`

## 2026-05-29 AMR 리프트/하역 검증 보강

AMR 이송 장면이 단순히 상태 전이만 통과하는 것이 아니라, 실제 적재물과 팔레트가 리프트와 함께 올라가고 하역 후 제자리에 남는지 self-test gate로 확인하도록 보강했습니다.

- `max_payload_lift_observed`를 추가해 stack item의 기준 Z 대비 실제 상승량을 기록합니다.
- `max_dropped_payload_drift`를 추가해 하역 후 AMR이 빠져나갈 때 박스/팔레트 assembly가 밀리는지 기록합니다.
- Python 옵션 `--self-test-min-payload-lift`, `--self-test-max-dropped-payload-drift`를 추가했습니다.
- PowerShell wrapper 옵션 `-SelfTestMinPayloadLift`, `-SelfTestMaxDroppedPayloadDrift`를 추가했습니다.
- 후반부 박스에서 reach/pick 복귀가 간헐적으로 시간 초과되는 문제가 있어 `REACH_PICK_MAX_DURATION = 12.0`, `RETURN_READY_DURATION = 10.0`으로 늘렸습니다. 빠르게 넘기는 것보다 실제로 pick-ready 근처에 도달한 뒤 다음 박스를 집도록 안정성을 우선했습니다.

확인 결과:

- unittest 29개 통과
- Python compile 통과
- 12000-frame full end-to-end self-test 통과
- 완료 로그: `placed_bins=8; transfer_cycles=1; max_pre_grip_offset=0.0048; max_return_ready_error=0.0392; max_release_drift=0.0000; max_payload_lift=0.1100; max_dropped_payload_drift=0.0000`

## 리프트-팔레트 접촉 간격 검증

AMR 리프트 플레이트가 팔레트 하부에서 떨어져 보이지 않도록 `AMR_LIFT_PLATE_OFFSET_Z`를 팔레트 deck underside 기준으로 계산합니다. 기본 설정에서는 리프트 상면과 팔레트 상판 하부 간격이 5 mm입니다.

추가 self-test 옵션:

- `--self-test-max-lift-contact-gap` / `-SelfTestMaxLiftContactGap`
- `--self-test-min-pallet-tunnel-clearance` / `-SelfTestMinPalletTunnelClearance`

검증 결과:

- unittest 35개 통과
- 12000-frame full end-to-end self-test 통과
- 완료 로그: `max_lift_contact_gap=0.0050; min_lift_contact_gap=0.0050; pallet_tunnel_clearance=0.0600`

## 두 줄 리프트 fork 형상

GUI에서 넓은 임시 판처럼 보이지 않도록 AMR 리프트 visual을 두 줄 fork/rail 형상으로 바꿨습니다. 각 fork는 팔레트 터널 안쪽에서 AMR pose와 lift offset을 따라 같이 움직입니다.

추가 self-test 옵션:

- `--self-test-min-lift-fork-inner-gap` / `-SelfTestMinLiftForkInnerGap`

검증 결과:

- unittest 37개 통과
- 12000-frame full end-to-end self-test 통과
- 완료 로그: `max_lift_contact_gap=0.0050; pallet_tunnel_clearance=0.1200; lift_fork_inner_gap=0.3600`

## Drop Slide Workstation 간격 검증

하역 작업대의 visible rail/roller가 팔레트 측면 runner와 겹치지 않도록 작업대 lane을 팔레트 중앙 터널 안쪽으로 정렬했습니다. 작업대 support 상면은 팔레트 상판 하부에서 5 mm 아래에 오도록 계산합니다.

추가 self-test 옵션:

- `--self-test-max-drop-support-gap` / `-SelfTestMaxDropSupportGap`
- `--self-test-min-drop-lane-clearance` / `-SelfTestMinDropLaneClearance`
- `--self-test-min-drop-runner-clearance` / `-SelfTestMinDropRunnerClearance`

검증 결과:

- unittest 38개 통과
- 12000-frame full end-to-end self-test 통과
- 완료 로그: `drop_support_gap=0.0050; drop_lane_clearance=0.1000; drop_runner_clearance=0.1700`

## Drop Slide와 AMR Fork 간섭 검증

하역 순간 AMR fork와 drop slide lane이 같은 위치를 차지하지 않도록 drop slide lane을 fork 바깥쪽으로 이동했습니다. 팔레트 중앙 터널 안에 남아 있으면서 AMR fork, side runner와 각각 clearance를 갖도록 계산합니다.

추가 self-test 옵션:

- `--self-test-min-drop-fork-clearance` / `-SelfTestMinDropForkClearance`

검증 결과:

- unittest 38개 통과
- 12000-frame full end-to-end self-test 통과
- 완료 로그: `pallet_tunnel_clearance=0.1600; drop_lane_clearance=0.0400; drop_runner_clearance=0.0700; drop_fork_clearance=0.0400`

## Stack Footprint 검증

적재 박스 전체가 팔레트 deck footprint 안쪽에 충분히 들어오는지 확인하는 gate를 추가했습니다. carton body 외곽과 pallet deck 외곽 사이의 최소 여백을 계산하고, overhang이 생기면 실패시킬 수 있습니다.

추가 self-test 옵션:

- `--self-test-min-stack-pallet-margin` / `-SelfTestMinStackPalletMargin`

검증 결과:

- unittest 39개 통과
- 12000-frame full end-to-end self-test 통과
- 완료 로그: `min_stack_pallet_margin=0.0850; max_stack_pallet_overhang=0.0000`

## AMR Slide-Out Exit Clearance 검증

하역 후 AMR fork가 dropped pallet 밖으로 충분히 빠져나갔는지 확인하는 gate를 추가했습니다. slide-out 완료 후 fork 뒤쪽 끝과 pallet deck 앞쪽 끝 사이의 X 방향 여유를 계산합니다.

추가 self-test 옵션:

- `--self-test-min-amr-exit-clearance` / `-SelfTestMinAmrExitClearance`

검증 결과:

- unittest 40개 통과
- 12000-frame full end-to-end self-test 통과
- 완료 로그: `amr_exit_clearance=0.6500; max_dropped_payload_drift=0.0000`

## Strict Full Realism Self-Test

현재까지 추가한 모든 realism gate를 한 번에 실행하는 wrapper를 추가했습니다.

```powershell
powershell -ExecutionPolicy Bypass -File .\run_harim_strict_self_test.ps1 -AcceptEula -SelfTestDebugBins
```

기본값은 headless, `12000` frames, `8` boxes, `1` transfer cycle입니다. GUI로 보고 싶으면 `-ShowGui`를 추가합니다.

검증 결과:

- unittest 41개 통과
- strict wrapper 기반 12000-frame full end-to-end self-test 통과
- 로그 파일: `isaacsim_logs/harim_strict_wrapper_full_e2e_12000.log`
- 완료 로그: `placed_bins=8; transfer_cycles=1; max_pre_grip_offset=0.0046; max_return_ready_error=0.0400; max_release_drift=0.0000; min_stack_pallet_margin=0.0850; max_dropped_payload_drift=0.0000; amr_exit_clearance=0.6500`

## Carton Side Label Visual

KLT collision/grasp 구조는 유지하면서 carton overlay에 side label과 red stripe visual을 추가했습니다. GUI에서 단순 갈색 박스가 아니라 물류 carton처럼 읽히도록 하기 위한 표시입니다.

추가 visual:

- `HarimCartonSideLabelFront`
- `HarimCartonSideLabelBack`
- `HarimCartonSideStripeFront`
- `HarimCartonSideStripeBack`

검증 결과:

- unittest 41개 통과
- strict wrapper 기반 12000-frame full end-to-end self-test 통과
- 로그 파일: `isaacsim_logs/harim_carton_label_strict_full_e2e_12000.log`

## Floor Marking Visual

pickup zone, drop zone, AMR path를 바닥에서 바로 읽을 수 있도록 얇은 `VisualCuboid` floor marking을 추가했습니다. 물리 충돌은 추가하지 않으므로 AMR/팔레트/박스 동작에는 영향을 주지 않습니다.

추가 visual:

- `AmrPathCenterLine`
- `PickupZone*`
- `DropZone*`

검증 결과:

- unittest 42개 통과
- strict wrapper 기반 12000-frame full end-to-end self-test 통과
- 로그 파일: `isaacsim_logs/harim_floor_markings_strict_full_e2e_12000.log`

## GUI Release-Retreat

GUI에서 박스가 흡착 패드에 계속 붙어 보이는 문제를 줄이기 위해 release 상태 자체에서 팔을 위로 retreat시키도록 바꿨습니다. release된 bin에는 `demo_force_released`를 남겨 active/carry 상태로 다시 복원되지 않게 하고, strict self-test는 `max_release_retreat_lift`가 최소 0.20m 이상인지 확인합니다.

검증 결과:

- unittest 42개 통과
- strict wrapper 기반 12000-frame full end-to-end self-test 통과
- 로그 파일: `isaacsim_logs/harim_release_retreat_gate_strict_full_e2e_12000.log`
- 완료 로그: `max_release_drift=0.0000; max_release_retreat_lift=0.2499; release_gripper_not_open=0; release_gripped_object_max=0`

## Load Restraint Visual

적재 완료 후 팔레트 위 박스 묶음에 6개 banding/load restraint visual이 나타나도록 추가했습니다. 적재 전에는 숨겨져 있고, `stack_complete` 이후 팔레트 assembly 일부로 AMR과 함께 이동합니다.

검증 결과:

- unittest 44개 통과
- strict wrapper 기반 12000-frame full end-to-end self-test 통과
- 로그 파일: `isaacsim_logs/harim_load_restraint_strict_full_e2e_12000.log`
- 완료 로그: `load_restraint_part_count=6; min_load_restraint_pallet_margin=0.0670; max_load_restraint_pallet_overhang=0.0000`

## Infeed Conveyor Visual

박스 유입 공정이 GUI에서 더 명확하게 보이도록 pick station 앞단에 infeed conveyor belt, guide rail, stop line, photo-eye sensor visual을 추가했습니다. strict self-test는 conveyor 길이, spawn 지점 이후 여유, guide rail clearance, belt support gap을 확인합니다.

검증 결과:

- unittest 45개 통과
- strict wrapper 기반 12000-frame full end-to-end self-test 통과
- 로그 파일: `isaacsim_logs/harim_infeed_conveyor_strict_full_e2e_12000.log`
- 완료 로그: `infeed_conveyor_length=0.9000; infeed_spawn_margin=0.4200; infeed_guide_clearance=0.4450; infeed_belt_support_gap=0.0080`
