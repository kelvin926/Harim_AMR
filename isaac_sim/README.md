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

참고: 일부 박스에서 Isaac Sim surface gripper의 실제 물리 close 판정 warning이 나올 수 있습니다. 데모는 `demo_carried_bin` fallback attach/release로 계속 진행되도록 구성했습니다.

## 2026-05-29 pre-grip 정렬 추가 보강

`ReachToPick` 직후 `DemoSettleBinAtGripper`가 active bin의 grasp frame을 UR10 end-effector frame에 맞춰 0.20초 동안 유지합니다. 접근 오차가 작으면 실제 suction close를 시도하고, 오차가 큰 경우에는 surface gripper close를 생략한 뒤 scripted fallback attach/release로 진행합니다. GUI에서 박스가 그리퍼에 계속 붙어 보이거나 release 뒤 다시 끌려가는 현상을 줄이기 위한 처리입니다.

추가 확인:

```powershell
cd E:\Harim_AMR
powershell -ExecutionPolicy Bypass -File .\run_harim_demo.ps1 -Headless -AcceptEula -SelfTestFrames 4200 -SelfTestMinPlacedBins 4 -SelfTestDebugBins -Cycles 1
```

2026-05-29 확인 결과: `placed_bins=4`까지 통과했고, AMR transfer self-test도 `completed transfer cycle 1`까지 통과했습니다.
