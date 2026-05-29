# Harim AMR Isaac Sim 구현 Todo

## 2026-05-29 영상 확인용 카메라 리그 보강

- [x] GUI에서 바로 장면을 확인할 수 있도록 `/World/HarimDemo/Cameras` 아래에 story camera 4개를 생성했다.
  - `OverviewCamera`: 팔레타이저와 AMR 주행 경로 전체를 보는 와이드 컷
  - `PalletizerCamera`: 컨베이어 유입, 흡착 pick, pallet place를 보는 팔레타이저 컷
  - `AmrRouteCamera`: 적재 완료 뒤 `iw_hub`가 팔레트 밑으로 들어가 lift-up 후 이동하는 경로 컷
  - `DropDockCamera`: 목표 위치의 슬라이드 작업대에 팔레트를 내려놓는 하역 컷
- [x] 각 카메라는 `harim:cameraRole` 속성을 갖고, GUI 실행 시 active viewport를 `OverviewCamera`로 전환하도록 했다.
- [x] self-test gate를 추가해 카메라 수, required role 수, 바닥 대비 최소 높이, target 거리까지 회귀 검증한다.
  - Python: `--self-test-min-camera-count`
  - Python: `--self-test-min-camera-role-count`
  - Python: `--self-test-min-camera-height`
  - Python: `--self-test-min-camera-target-distance`
  - PowerShell: `-SelfTestMinCameraCount`
  - PowerShell: `-SelfTestMinCameraRoleCount`
  - PowerShell: `-SelfTestMinCameraHeight`
  - PowerShell: `-SelfTestMinCameraTargetDistance`
- [x] strict wrapper에 카메라 gate를 포함했다.
  - `SelfTestMinCameraCount = 4`
  - `SelfTestMinCameraRoleCount = 4`
  - `SelfTestMinCameraHeight = 1.25`
  - `SelfTestMinCameraTargetDistance = 1.0`
- [x] 검증 완료
  - unittest 51개 통과
  - 12000-frame strict full end-to-end self-test 통과
  - 로그 파일: `isaacsim_logs/harim_camera_rig_strict_full_e2e_12000.log`
  - 완료 로그 핵심값: `placed_bins=8`, `transfer_cycles=1`, `release_gripper_not_open=0`, `release_gripped_object_max=0`, `camera_rig_count=4`, `camera_required_role_count=4`, `camera_min_height=2.3500`, `camera_min_target_distance=3.1016`

## 2026-05-29 GUI 확인 반영: 로봇팔 release 안정화

- GUI에서 로봇팔이 박스를 놓지 않는 것처럼 보이는 문제를 줄이기 위해, `DemoScriptedPlaceBin.exit()` 시점에 박스가 목표 stack pose에 도달하면 즉시 `release_demo_bin_at_target()`을 호출하도록 변경한다.
- release helper는 `demo_attached`, `demo_attach_T`, `demo_carried_bin`, `demo_scripted_place_bin`, `active_bin`, `is_attached`, `is_grasp_reached` 상태를 한 번에 끊고, 박스를 목표 위치에 kinematic으로 고정한다.
- `DemoReleaseBin`은 이미 release된 `demo_released_bin`을 우선 사용하며, 이후에는 surface gripper open 상태 유지와 arm retreat만 담당한다.
- `force_open_suction_gripper()`는 `gripper.open()`, `interface.open_gripper()`, `set_gripper_action_batch(..., -1.0)`를 여러 번 시도해 surface gripper joint가 남는 경우를 더 강하게 방지한다.
- strict self-test 로그 `isaacsim_logs/harim_release_detach_indicator_strict_full_e2e_12000.log` 기준 8개 박스 모두 `scripted-release` 후 `demo-placed`가 기록되고, `release_gripped_object_max=0`, `release_gripper_not_open=0`, `max_release_drift=0.0000`으로 확인했다.

## 2026-05-29 AMR 상태 indicator 보강

- `iw_hub` 안전 시각 요소에 warning strip 2개를 추가해 총 8개 AMR safety visual을 구성한다.
- beacon dome과 warning strip은 AMR 이동/리프트/하역 동작 중 표시하고, green status strip은 idle/wait 상태에서 표시하도록 role 기반 visibility를 추가한다.
- strict self-test에 AMR warning/idle indicator 구성 수, 관측 여부, visibility mismatch gate를 추가했다.

## 0. 목표 정의

본 프로젝트의 1차 목표는 YouTube 영상의 Robotize GoPal U24W 팔레트 AMR 데모와 유사한 물류 자동화 장면을 Isaac Sim에서 구현하는 것이다.

구현하려는 전체 작업 흐름은 다음과 같다.

1. 컨베이어로 박스가 유입된다.
2. 흡착식 그리퍼를 장착한 산업용 로봇팔이 박스를 집는다.
3. 로봇팔이 팔레트 위에 박스를 여러 층으로 적재한다.
4. 적재 완료 신호가 발생한다.
5. `iw_hub` AMR이 팔레트 하부로 진입한다.
6. `iw_hub`가 팔레트를 lift-up 한다.
7. `iw_hub`가 목표 위치로 팔레트를 운반한다.
8. 목표 위치에서 팔레트를 내려놓고 이탈한다.

1차 산출물은 실제 제어 검증용 디지털 트윈이 아니라, 회장님/의사결정자에게 보여줄 수 있는 설명용 시뮬레이션 영상 또는 데모 장면이다. 따라서 초기 구현은 실제 물리 정확도보다 안정적인 연출과 이해하기 쉬운 작업 흐름을 우선한다.

## 1. 구현 전략 요약

초기 구현은 다음 조합을 권장한다.

- 팔레타이징 셀: Isaac Sim의 `UR10 Bin Stacking` 또는 `UR10 Palletizing` 예제 기반
- 로봇팔: `UniversalRobots/ur10/ur10.usd` 우선 사용
- 그리퍼: Isaac Sim `Surface Gripper` 또는 UR10 suction accessory 사용
- 컨베이어: 기존 UR10 예제의 컨베이어 또는 `Conveyor Belt Utility`
- 팔레트 AMR: Isaac Sim 기본 자산의 `iw_hub`
- AMR 제어: 초기에는 `GoTo`, `Idle`, `LiftUp`, `LiftDown` 기반 scripted command 사용
- 전체 흐름 제어: Python FSM 또는 Cortex context monitor 기반 custom orchestrator
- 팔레트 운반 연출: 초기에는 lift-up 시점에 팔레트와 박스 묶음을 `iw_hub`에 attach하고, lift-down 후 detach

ROS2/Nav2 기반 자율주행은 1차 구현 범위에서 제외하고, 2차 목표로 둔다. 이유는 설명용 영상 제작에는 deterministic scripted motion이 더 빠르고 안정적이며, ROS2/Nav2는 환경 설정과 디버깅 비용이 크기 때문이다.

## 2. 참고 문서

작업 중 우선 참고할 공식 문서는 다음과 같다.

1. UR10 Bin Stacking Cortex 예제
   - URL: https://docs.isaacsim.omniverse.nvidia.com/5.1.0/cortex_tutorials/tutorial_cortex_5_ur10_bin_stacking.html
   - 역할: 컨베이어, 흡착 그리퍼, UR10, 팔레트 적재 상태머신 참고

2. UR10 Palletizing Replicator 예제
   - URL: https://docs.isaacsim.omniverse.nvidia.com/latest/replicator_tutorials/tutorial_replicator_ur10_palletizing.html
   - 역할: 팔레타이징 장면, 이벤트 기반 데이터/카메라 캡처, 조명/재질 랜덤화 참고

3. Replicator Agent Actor Control
   - URL: https://docs.isaacsim.omniverse.nvidia.com/5.1.0/action_and_event_data_generation/ext_replicator-agent/actor_control.html
   - 역할: `iw_hub`의 `GoTo`, `Idle`, `LiftUp`, `LiftDown` 명령 확인

4. ROS2 Navigation with iw.hub
   - URL: https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_navigation.html
   - 역할: 2차 단계에서 Nav2 기반 이동, 장애물 회피, `iw_hub_navigation` 패키지 참고

5. Surface Gripper Extension
   - URL: https://docs.isaacsim.omniverse.nvidia.com/5.1.0/robot_simulation/ext_isaacsim_robot_surface_gripper.html
   - 역할: 흡착식 그리퍼 구현, D6 joint 기반 grasp, open/close 제어 참고

6. Conveyor Belt Utility
   - URL: https://docs.isaacsim.omniverse.nvidia.com/5.1.0/digital_twin/warehouse_logistics/ext_isaacsim_asset_gen_conveyor.html
   - 역할: 컨베이어 벨트 생성 및 rigid body 이동 구현 참고

## 3. 1차 구현 범위

1차 구현에서는 다음을 완료한다.

- Isaac Sim에서 팔레타이징 셀 장면 실행
- 컨베이어에서 박스가 공급되는 장면 구성
- UR10 또는 유사 로봇팔이 박스를 흡착하여 팔레트에 적재
- 여러 개의 박스가 팔레트 위에 층으로 쌓이는 동작 구현
- 적재 완료 조건 감지
- 적재 완료 후 `iw_hub` AMR이 팔레트 하부로 접근
- `iw_hub` lift-up 동작
- 팔레트와 적재 박스가 AMR과 함께 이동하는 연출
- 목표 위치에서 lift-down
- 팔레트와 박스가 목표 위치에 남고, AMR이 이탈
- 영상 촬영용 카메라 경로 또는 고정 카메라 컷 구성

1차 구현에서 제외하는 항목은 다음과 같다.

- 실제 Robotize GoPal U24W의 정확한 CAD/치수 재현
- 실제 팔레트 하부 clearance 기반 기구 검증
- 박스 개별 물리 안정성을 100% 유지하는 고정밀 운반
- ROS2/Nav2 기반 완전 자율주행
- 실제 PLC, WMS, MES 연동
- 실제 하림 물류센터 레이아웃 완전 재현

## 4. 개발 단계별 Todo

### 4.1. Isaac Sim 환경 확인

- [ ] Isaac Sim 버전 확인
  - 권장 버전: Isaac Sim 5.1.0 기준
  - 문서와 설치 버전이 다르면 API 경로, extension 이름, 예제 위치가 달라질 수 있다.
- [ ] Isaac Sim 실행 경로 확인
  - Linux: `<isaac_sim_root>/python.sh`
  - Windows: `<isaac_sim_root>\python.bat`
- [ ] 예제 실행용 Python 명령 확인
- [ ] `standalone_examples` 디렉터리 위치 확인
- [ ] Isaac Sim asset root 접근 가능 여부 확인
- [ ] 기본 로봇/창고 자산이 로드되는지 확인
- [ ] extension manager에서 필요한 extension 활성화 여부 확인
  - `isaacsim.robot.surface_gripper`
  - `isaacsim.asset.gen.conveyor`
  - `isaacsim.ros2.bridge`는 2차 단계에서만 필요

### 4.2. UR10 Bin Stacking 예제 실행

- [ ] `UR10 Bin Stacking` 문서의 예제 실행
  - 예상 스크립트: `standalone_examples/api/isaacsim.cortex.framework/demo_ur10_conveyor_main.py`
- [ ] 컨베이어가 정상 작동하는지 확인
- [ ] UR10 로봇팔이 bin 또는 box를 정상 pick 하는지 확인
- [ ] suction gripper attach/detach 동작 확인
- [ ] 팔레트 또는 지정 영역 위에 물체를 정상 place 하는지 확인
- [ ] 예제의 상태머신 구조 파악
  - `Dispatch`
  - `PickBin`
  - `PlaceBin`
  - `FlipBin`
  - `GoHome`
  - `stack_complete`
- [ ] context monitor 구조 파악
  - active bin 감지
  - grasp transform 감지
  - grasp reached 여부 감지
  - stacked bins 목록 관리
- [ ] 예제 실행 중 에러 로그 기록
- [ ] 실행 가능한 원본 예제를 별도 브랜치 또는 백업 디렉터리에 보존

### 4.3. 팔레타이징 장면 단순화

- [ ] 기존 bin flipping 로직이 불필요하면 제거 또는 비활성화
- [ ] 컨베이어에서 들어오는 물체를 하림 박스 형태의 carton box로 변경
- [ ] 박스 크기 정의
  - 예: 0.4 m x 0.3 m x 0.25 m
  - 실제 상품 박스 크기가 있으면 반영
- [ ] 박스 질량 정의
  - 영상용이면 가볍게 설정
  - 물리 안정성 테스트용이면 실제에 가깝게 조정
- [ ] 박스 material 설정
  - 갈색 골판지 재질
  - `HARIM` 또는 하림 느낌의 로고/텍스트는 추후 적용
- [ ] 팔레트 크기 정의
  - US pallet 또는 UK pallet 중 목표 선택
  - 예제와 `iw_hub` 상판 폭에 맞춰 조정
- [ ] 팔레트 collision 단순화
  - 초기에는 box collider 또는 간단한 mesh collider 사용
  - 하부 fork/under-ride 공간이 보이도록 visual mesh만 상세화
- [ ] 팔레트 위치를 로봇팔 reach 영역과 `iw_hub` 접근 경로가 모두 가능한 곳에 배치

### 4.4. 적재 패턴 설계

- [ ] 1차 적재 패턴 결정
  - 예: 한 층에 3 x 2 = 6개
  - 예: 3층 총 18개
- [ ] 각 박스의 목표 위치 좌표 계산
  - 팔레트 중심 기준 local offset 배열 사용
  - 층별 z 높이 계산
- [ ] 층별 교차 적재 여부 결정
  - 1차 구현은 모든 층 동일 방향 권장
  - 영상 완성 후 교차 적재 추가
- [ ] `PlaceBin` 또는 유사 place state를 박스 좌표 배열 기반으로 수정
- [ ] 다음 적재 위치 index 관리
- [ ] 마지막 index 도달 시 `stack_complete = True` 설정
- [ ] 박스가 살짝 어긋나도 보기 좋도록 place 후 위치 보정 여부 검토
  - 영상용이면 place 직후 transform snap 가능
  - 물리 검증용이면 snap을 피하고 RMPflow 보정 사용

### 4.5. 흡착 그리퍼 구현

- [ ] UR10 예제의 기존 suction gripper 구조 확인
- [ ] `Surface Gripper Extension` 사용 여부 결정
- [ ] gripper open/close API 확인
  - `close_gripper`
  - `open_gripper`
  - `set_gripper_action`
- [ ] grip 가능 거리 설정
  - `Max Grip Distance`
- [ ] break force 설정
  - `Shear Force Limit`
  - `Coaxial Force Limit`
- [ ] 박스 상단을 향하는 attachment direction 확인
- [ ] 박스 pick 시점에 gripper와 box가 정확히 접촉하도록 pre-pick pose 조정
- [ ] place 시점에 release 후 box가 튀지 않도록 속도/높이/접촉 offset 조정
- [ ] 실제 흡착 물리 대신 scripted attach/detach가 더 안정적인지 비교
- [ ] 최종 영상용 안정화 방식 결정

### 4.6. 로봇팔 모션 안정화

- [ ] RMPflow 설정 확인
- [ ] pick approach pose 설정
- [ ] pick pose 설정
- [ ] lift pose 설정
- [ ] place approach pose 설정
- [ ] place pose 설정
- [ ] retract pose 설정
- [ ] 로봇팔 self collision 또는 주변 장비 collision 확인
- [ ] 팔레트 위 박스가 쌓일수록 로봇팔 경로가 충돌하지 않는지 확인
- [ ] 필요 시 invisible obstacle toggle 방식 사용
  - UR10 Bin Stacking 예제의 obstacle monitor 패턴 참고
- [ ] 완료 후 home pose 복귀
- [ ] 적재 완료 후 로봇팔이 AMR 접근 경로를 가리지 않도록 후퇴

### 4.7. `iw_hub` AMR 자산 확인

- [ ] Isaac Sim asset browser에서 `iw_hub` 자산 확인
- [ ] 가능하면 sensor 버전 사용 여부 확인
  - 예: `iw_hub_sensors.usd`
- [ ] `iw_hub`의 크기, 상판 높이, loader 높이 확인
- [ ] 팔레트 하부로 진입 가능한 visual clearance 확인
- [ ] `LiftUp` / `LiftDown` 동작 확인
  - 문서상 loader는 4 cm 이동
- [ ] `iw_hub`의 stage prim 이름 확인
  - 예: `/World/Robots/iw_hub`
  - 예: `/World/Robots/iw_hub_01`
- [ ] `GoTo` 명령 좌표계 확인
- [ ] 회전 방향과 최종 orientation 제어 가능 여부 확인
- [ ] 팔레트 하부 진입 전/후 충돌체 간섭 확인

### 4.8. `iw_hub` 이동 제어 방식 결정

1차 구현에서는 다음 중 하나를 선택한다.

#### 방식 A: Replicator Agent / Actor Control command 사용

- 장점
  - 문서상 `GoTo`, `Idle`, `LiftUp`, `LiftDown` 명령이 존재
  - 빠르게 영상용 시퀀스 구성 가능
  - ROS2 설정 불필요
- 단점
  - 세밀한 도킹 제어와 실제 navigation 검증에는 한계
  - 팔레트 attach/detach는 별도 Python 필요

작업 항목:

- [ ] Replicator Agent extension 활성화
- [ ] NavMesh 필요 여부 확인
- [ ] warehouse stage에 NavMesh bake
- [ ] `iw_hub` actor spawn
- [ ] command file로 단독 이동 테스트
- [ ] command 예시 작성
  - `iw_hub GoTo <pickup_approach_x> <pickup_approach_y> <pickup_approach_z>`
  - `iw_hub GoTo <pickup_under_pallet_x> <pickup_under_pallet_y> <pickup_under_pallet_z>`
  - `iw_hub LiftUp`
  - `iw_hub GoTo <drop_x> <drop_y> <drop_z>`
  - `iw_hub LiftDown`
  - `iw_hub GoTo <exit_x> <exit_y> <exit_z>`

#### 방식 B: Python FSM에서 직접 transform 또는 controller 제어

- 장점
  - 팔레트 attach/detach와 동기화가 쉬움
  - 영상용 카메라워크와 타이밍 제어가 쉬움
  - NavMesh 의존성이 줄어듦
- 단점
  - 실제 AMR navigation 느낌이 약할 수 있음

작업 항목:

- [ ] `iw_hub`의 목표 waypoint 배열 정의
- [ ] 각 waypoint까지 이동하는 simple controller 작성
- [ ] 목표 도착 판정 로직 작성
- [ ] pickup pose에서 lift-up 실행
- [ ] drop pose에서 lift-down 실행
- [ ] 카메라 컷과 동기화

#### 방식 C: ROS2 Nav2 사용

- 장점
  - 실제 자율주행 데모에 가까움
  - RViz와 Nav2 goal 사용 가능
  - 장애물 회피를 설명하기 좋음
- 단점
  - Windows 환경에서는 부분 지원이며 오류 가능
  - 설치, launch, map, localization, parameter 튜닝 비용이 큼
  - 1차 영상 제작에는 과함

작업 항목:

- [ ] 2차 단계에서 진행
- [ ] Linux 또는 WSL/Docker 기반 실행 검토
- [ ] `iw_hub_navigation` 패키지 실행
- [ ] occupancy map 생성 또는 기본 map 사용
- [ ] scripted goal sender 작성

1차 구현 권장안은 방식 A 또는 B이다. 안정적인 영상 제작을 최우선으로 하면 방식 B, 공식 `iw_hub` 명령 활용을 보여주려면 방식 A가 좋다.

### 4.9. 팔레트 attach/detach 설계

초기 구현에서는 팔레트와 적재 박스를 실제 접촉 물리만으로 운반하지 않는다. lift-up 이후에는 팔레트와 박스 묶음을 `iw_hub`에 attach하여 안정적으로 이동시키고, lift-down 후 detach한다.

작업 항목:

- [ ] 적재 완료 후 팔레트와 박스 prim 목록 수집
- [ ] `pallet_assembly` 그룹 prim 또는 Xform 생성
- [ ] 팔레트와 박스를 `pallet_assembly` 하위로 관리할지 결정
- [ ] lift-up 직전 world transform 보존
- [ ] lift-up 완료 시 `pallet_assembly`를 `iw_hub` loader frame에 parent 변경
- [ ] parent 변경 시 world transform 유지
- [ ] 이동 중에는 assembly가 AMR 상판과 함께 움직이도록 처리
- [ ] drop 위치 도착 후 lift-down 실행
- [ ] lift-down 완료 시 `pallet_assembly`를 world 하위로 detach
- [ ] detach 후 world transform 유지
- [ ] detach 후 팔레트가 바닥에 안정적으로 놓인 것처럼 보이도록 z 위치 보정
- [ ] 필요 시 rigid body를 일시적으로 kinematic 처리

주의할 점:

- 박스 개별 rigid body를 계속 활성화한 상태로 AMR이 움직이면 흔들림, 튕김, 무너짐이 발생할 수 있다.
- 설명용 영상에서는 적재 완료 후 박스들을 팔레트에 고정하거나 rigid body simulation을 잠시 비활성화하는 것이 안정적이다.
- 물리 검증 단계에서는 이 단순화를 제거하고 실제 접촉/마찰/질량 튜닝으로 전환한다.

### 4.10. 적재 완료 신호와 AMR 호출 연결

- [ ] `stack_complete` 상태가 어디에서 설정되는지 확인
- [ ] 적재 완료 시 로봇팔이 home 또는 safe pose로 복귀하도록 설정
- [ ] AMR 접근 경로에 로봇팔/컨베이어/안전펜스가 간섭하지 않도록 위치 조정
- [ ] `stack_complete`가 true가 되는 순간 AMR FSM 시작
- [ ] AMR FSM 상태 정의
  - `WAIT_FOR_STACK_COMPLETE`
  - `MOVE_TO_PICKUP_APPROACH`
  - `MOVE_UNDER_PALLET`
  - `LIFT_UP`
  - `ATTACH_PALLET`
  - `MOVE_TO_DROP`
  - `LIFT_DOWN`
  - `DETACH_PALLET`
  - `MOVE_TO_EXIT`
  - `DONE`
- [ ] 각 상태별 timeout 설정
- [ ] 실패 시 복구 또는 로그 출력
- [ ] 영상용으로 각 상태 전환 시 짧은 pause를 넣을지 결정

### 4.11. 장면 구성

- [ ] 기본 warehouse 환경 선택
  - `warehouse.usd`
  - `warehouse_with_forklifts.usd`
  - `small_warehouse_digital_twin.usd`
- [ ] 너무 복잡한 full warehouse는 1차 구현에서 피한다.
- [ ] 팔레타이저 셀 배치
  - 컨베이어
  - 로봇팔 베이스
  - 팔레트 위치
  - 제어 판넬 또는 안전펜스
- [ ] AMR 이동 경로 확보
  - pickup approach
  - pickup under pallet
  - drop zone
  - exit zone
- [ ] 바닥 material 설정
- [ ] 조명 설정
- [ ] 카메라가 볼 때 작업 흐름이 한눈에 보이도록 장비 간 거리 조정
- [ ] 실제 하림 물류센터 느낌을 위한 요소 추가
  - 박스 로고
  - 구역 라인
  - 팔레트 랙
  - 안전 펜스
  - 표지판
  - 바닥 마킹

### 4.12. 카메라와 영상 연출

최종 결과물이 설명용 영상이라면 카메라워크가 중요하다.

권장 카메라 컷:

1. 전체 셀 와이드샷
   - 컨베이어, 로봇팔, 팔레트, AMR 대기 위치가 모두 보이게 구성
2. 박스 유입 클로즈업
   - 컨베이어 위 박스 이동 강조
3. 흡착 pick 클로즈업
   - gripper가 박스 상단에 붙는 장면
4. 팔레트 적재 와이드샷
   - 여러 층으로 쌓이는 동작
5. 적재 완료 후 AMR 진입샷
   - `iw_hub`가 팔레트 밑으로 들어가는 장면
6. lift-up 클로즈업
   - 팔레트가 살짝 올라가는 장면
7. AMR 이동 추적샷
   - 적재 팔레트를 싣고 이동
8. drop zone 하역샷
   - lift-down 후 팔레트가 목표 위치에 남음
9. AMR 이탈 및 완료 와이드샷

작업 항목:

- [ ] 카메라 prim 생성
- [ ] 각 컷별 카메라 위치와 look-at target 정의
- [ ] 컷 전환 타이밍 정의
- [ ] 필요 시 카메라 path animation 작성
- [ ] 조명과 노출 확인
- [ ] 영상 렌더링 해상도 결정
  - 1920x1080 권장
- [ ] 프레임레이트 결정
  - 30fps 권장
- [ ] 렌더링 모드 결정
  - 빠른 preview: RTX Realtime
  - 최종 영상: 필요 시 Path Tracing 일부 사용

### 4.13. 파일/코드 구조 제안

초기 구현 코드 구조는 다음과 같이 잡는다.

```text
Harim_AMR/
  Todo.md
  isaac_sim/
    README.md
    scripts/
      run_harim_pallet_demo.py
      harim_scene_setup.py
      palletizing_controller.py
      amr_controller.py
      camera_director.py
    configs/
      pallet_pattern.yaml
      amr_waypoints.yaml
      camera_shots.yaml
    assets/
      textures/
        harim_box_basecolor.png
      usd/
        harim_box.usd
        harim_pallet.usd
```

각 파일의 역할:

- `run_harim_pallet_demo.py`
  - 전체 데모 실행 entrypoint
- `harim_scene_setup.py`
  - warehouse, conveyor, robot arm, pallet, AMR 배치
- `palletizing_controller.py`
  - 컨베이어 박스 감지, pick/place, 적재 패턴, `stack_complete` 관리
- `amr_controller.py`
  - `iw_hub` 이동, lift-up/down, pallet attach/detach
- `camera_director.py`
  - 카메라 컷 전환, 영상 녹화용 카메라 제어
- `pallet_pattern.yaml`
  - 박스 크기, 층 수, 좌표 패턴
- `amr_waypoints.yaml`
  - pickup/drop/exit waypoint
- `camera_shots.yaml`
  - 카메라 위치와 타이밍

## 5. 구현 상세 설계

### 5.1. 전체 FSM 설계

전체 데모는 하나의 상위 FSM으로 제어한다.

```text
INIT
  -> START_CONVEYOR
  -> PALLETIZING
  -> STACK_COMPLETE
  -> ARM_GO_HOME
  -> AMR_APPROACH
  -> AMR_UNDER_PALLET
  -> AMR_LIFT_UP
  -> ATTACH_PALLET_ASSEMBLY
  -> AMR_MOVE_TO_DROP
  -> AMR_LIFT_DOWN
  -> DETACH_PALLET_ASSEMBLY
  -> AMR_EXIT
  -> DEMO_DONE
```

상태별 책임:

- `INIT`
  - 장면 로드
  - asset path 확인
  - physics scene 설정
  - 카메라 초기화
- `START_CONVEYOR`
  - 박스 공급 시작
  - 첫 박스가 pick 위치에 도달할 때까지 대기
- `PALLETIZING`
  - 로봇팔 pick/place 반복
  - 적재 index 증가
  - 모든 박스 적재 시 `stack_complete`
- `STACK_COMPLETE`
  - 컨베이어 정지
  - 박스 rigid state 안정화
  - 팔레트 assembly 구성
- `ARM_GO_HOME`
  - 로봇팔 safe pose 이동
  - AMR 접근 경로 확보
- `AMR_APPROACH`
  - `iw_hub` pickup approach 위치로 이동
- `AMR_UNDER_PALLET`
  - 팔레트 하부 중앙으로 저속 진입
- `AMR_LIFT_UP`
  - loader 상승
- `ATTACH_PALLET_ASSEMBLY`
  - 팔레트와 박스 묶음을 AMR에 attach
- `AMR_MOVE_TO_DROP`
  - 목표 위치로 이동
- `AMR_LIFT_DOWN`
  - loader 하강
- `DETACH_PALLET_ASSEMBLY`
  - 팔레트를 world에 detach
- `AMR_EXIT`
  - AMR이 빈 상태로 빠져나감
- `DEMO_DONE`
  - 카메라 종료 컷
  - 필요 시 녹화 종료

### 5.2. 팔레트/박스 묶음 처리

1차 영상용 안정화 방식:

- 적재 중에는 박스 개별 rigid body를 활성화한다.
- 적재 완료 후 각 박스의 최종 transform을 저장한다.
- 팔레트와 박스를 하나의 `pallet_assembly` Xform 아래에 묶는다.
- 운반 중에는 `pallet_assembly`를 kinematic처럼 다룬다.
- AMR 이동 중에는 `pallet_assembly` transform을 AMR loader frame 기준 offset으로 갱신한다.
- drop 후에는 `pallet_assembly`를 world에 고정하고 필요 시 rigid body를 다시 활성화한다.

이 방식의 장점:

- 박스가 이동 중 무너지지 않는다.
- 영상 렌더링이 안정적이다.
- lift-up/down과 이동 타이밍을 정확히 맞출 수 있다.

이 방식의 단점:

- 실제 접촉 물리 검증은 아니다.
- AMR 급가속/감속에 따른 적재 안정성 검토는 불가능하다.

2차 물리 검증 방식:

- 팔레트와 박스를 각각 rigid body로 유지
- 팔레트와 `iw_hub` loader 사이 접촉/마찰로만 운반
- 박스 간 마찰계수, 질량, restitution 조정
- physics timestep과 solver iteration 증가
- lift 속도와 acceleration 제한
- 필요 시 박스끼리 약한 fixed joint 또는 contact stabilization 적용

### 5.3. `iw_hub` 도킹 포즈 설계

도킹 좌표는 다음 3단계로 나눈다.

1. `pickup_approach`
   - 팔레트 전방 1~2 m 위치
   - 팔레트 중앙축과 yaw 정렬
2. `pickup_pre_under`
   - 팔레트 입구 바로 앞
   - 저속 진입 시작점
3. `pickup_under`
   - 팔레트 중심 하부
   - lift-up 수행 위치

작업 항목:

- [ ] 팔레트 local frame 정의
- [ ] AMR가 진입할 방향 정의
- [ ] 팔레트 중심점 계산
- [ ] `pickup_approach` 좌표 계산
- [ ] `pickup_under` 좌표 계산
- [ ] AMR yaw가 팔레트 진입 방향과 일치하는지 확인
- [ ] under-ride 시 visual 간섭 여부 확인
- [ ] lift-up 후 팔레트 z offset 확인

### 5.4. 적재 완료 후 AMR 경로

예상 경로:

```text
AMR 대기 위치
  -> pickup_approach
  -> pickup_under
  -> lift_up
  -> drop_approach
  -> drop_pose
  -> lift_down
  -> exit_pose
```

작업 항목:

- [ ] AMR 대기 위치 설정
- [ ] pickup 접근 경로 설정
- [ ] 목표 하역 위치 설정
- [ ] drop 위치에 팔레트가 남아도 AMR이 빠져나갈 수 있는 clearance 확보
- [ ] 이동 중 카메라가 팔레트를 가리지 않도록 경로 조정
- [ ] 필요 시 바닥 라인 또는 목적지 마킹 추가

## 6. 에셋 후보

### 6.1. 로봇팔

우선순위:

1. `UniversalRobots/ur10/ur10.usd`
   - 장점: UR10 Bin Stacking 예제 재활용 가능
   - 단점: 대형 팔레타이저 느낌은 약할 수 있음

2. `UniversalRobots/ur20/ur20.usd` 또는 `UniversalRobots/ur30/ur30.usd`
   - 장점: 더 큰 협동로봇 느낌
   - 단점: suction gripper와 팔레타이징 로직을 추가로 맞춰야 함

3. `Kuka/KR210_L150/kr210_l150.usd`
   - 장점: 산업용 대형 팔레타이저 느낌이 강함
   - 단점: 모션 생성, 그리퍼, 적재 상태머신을 더 많이 직접 구현해야 함

1차 구현은 UR10으로 진행한다. 시각적 임팩트가 부족하면 2차로 UR20/UR30 또는 KUKA 교체를 검토한다.

### 6.2. AMR

우선순위:

1. `iw_hub`
   - 장점: Isaac Sim 문서에 `LiftUp`, `LiftDown` 지원이 명시되어 있음
   - 장점: `iw_hub_navigation` 예제가 있음
   - 단점: Robotize GoPal U24W와 외형/치수가 완전히 같지는 않음

2. 커스텀 GoPal 유사 모델
   - 장점: 영상 원본과 유사한 외형 가능
   - 단점: CAD/USD 제작, collision, joint, controller 작업 필요

1차 구현은 `iw_hub`로 진행한다. GoPal 외형 재현은 3차 목표로 둔다.

### 6.3. 그리퍼

1. UR10 예제 내 suction gripper
2. `Surface Gripper Extension`
3. custom scripted attach/detach gripper

1차 구현에서는 예제 내 suction 구조를 우선 사용하고, 안정성이 부족하면 custom attach/detach를 병행한다.

### 6.4. 창고/소품

사용 후보:

- `warehouse.usd`
- `warehouse_with_forklifts.usd`
- `small_warehouse_digital_twin.usd`
- pallet
- carton box
- rack
- shelf
- safety fence
- conveyor
- floor marking

## 7. 리스크와 대응

### 7.1. 박스 적재가 무너지는 문제

원인:

- 박스 마찰 부족
- place 위치 오차
- release 시 잔류 속도
- solver iteration 부족
- 박스 간 collision mesh 불안정

대응:

- place 후 transform snap 적용
- 적재 완료 후 박스 rigid body 비활성화
- 박스와 팔레트를 assembly로 묶기
- 물리 검증 단계에서만 contact 기반 운반 사용

### 7.2. UR10 reach가 부족한 문제

원인:

- 팔레트가 너무 멀거나 높음
- 박스 크기가 큼
- 적재 층수가 높음

대응:

- 팔레트를 UR10 가까이 배치
- pedestal 추가
- 적재 층수 감소
- UR20/UR30/KUKA로 교체 검토

### 7.3. `iw_hub`가 팔레트 밑으로 시각적으로 진입하지 못하는 문제

원인:

- 팔레트 하부 clearance 부족
- `iw_hub` 상판 높이가 높음
- collision mesh 간섭

대응:

- 팔레트 visual/collision 분리
- visual은 실제 팔레트처럼 만들고 collision은 단순화
- AMR 진입 시 collision 일부 비활성화
- 카메라 각도로 자연스럽게 연출
- lift-up 순간 attach 방식 사용

### 7.4. `LiftUp` 4 cm가 부족한 문제

원인:

- 문서상 `iw_hub` loader 이동량이 4 cm
- 영상에서 팔레트 상승이 잘 보이지 않을 수 있음

대응:

- 팔레트 초기 위치를 loader와 가깝게 조정
- lift-up 시 팔레트 assembly z offset을 함께 보정
- 카메라 클로즈업과 LED/효과로 lift-up 인지 강화
- 필요 시 custom lift animation 작성

### 7.5. ROS2/Nav2 환경 문제

원인:

- Windows에서 ROS2 Navigation은 부분 지원
- RViz2 실행 오류 가능
- Nav2 설치와 map 설정이 복잡함

대응:

- 1차 구현에서는 ROS2 제외
- 2차에서 Linux 환경 권장
- 먼저 Isaac Sim 기본 `iw_hub_navigation` 예제 단독 실행
- 이후 본 데모와 통합

## 8. 마일스톤

### M1. 레퍼런스 예제 실행

완료 조건:

- UR10 Bin Stacking 예제가 로컬 Isaac Sim에서 실행된다.
- 컨베이어, 흡착 pick, 팔레트 적재가 동작한다.
- `iw_hub` 단독 이동과 lift-up/down이 확인된다.

### M2. 하림 팔레타이징 셀 구성

완료 조건:

- 박스가 하림 물류 박스처럼 보인다.
- 컨베이어에서 박스가 공급된다.
- 로봇팔이 박스를 팔레트에 정해진 패턴으로 적재한다.
- 적재 완료 신호가 발생한다.

### M3. AMR 팔레트 픽업 구현

완료 조건:

- 적재 완료 후 AMR이 팔레트 아래로 진입한다.
- lift-up 동작이 보인다.
- 팔레트와 박스가 AMR과 함께 이동한다.

### M4. 하역 및 이탈 구현

완료 조건:

- AMR이 목표 위치에서 팔레트를 내려놓는다.
- 팔레트와 박스가 목표 위치에 남는다.
- AMR이 빈 상태로 이탈한다.

### M5. 영상용 연출 정리

완료 조건:

- 전체 작업 흐름이 한 번에 이해된다.
- 카메라 컷이 구성되어 있다.
- 최종 렌더링 또는 화면 녹화가 가능하다.

### M6. 2차 고도화

후속 목표:

- ROS2/Nav2 기반 `iw_hub` 이동
- 동적 장애물 회피
- GoPal 유사 외형 모델 제작
- 실제 하림 레이아웃 반영
- 박스/팔레트 실제 물리 안정성 검증

## 9. 우선순위 체크리스트

바로 진행할 순서는 다음과 같다.

- [x] Isaac Sim 5.1 설치/실행 확인
- [x] UR10 Bin Stacking 예제 실행
- [x] `iw_hub` 예제 또는 asset 로드 확인
- [x] `LiftUp` / `LiftDown` 단독 테스트
- [x] UR10 예제에서 적재 완료 조건 찾기
- [x] 적재 완료 후 AMR FSM 호출 구조 설계
- [x] 팔레트 attach/detach 테스트
- [x] 전체 1-cycle 데모 구현
- [ ] 카메라 컷 추가
- [ ] 최종 영상 렌더링

## 10. 최종 구현 판단

이 프로젝트는 Isaac Sim에서 충분히 구현 가능하다. 가장 빠르고 안정적인 접근은 기존 `UR10 Bin Stacking` 예제를 팔레타이징 셀의 출발점으로 삼고, `iw_hub`의 lift 기능을 가진 팔레트 AMR 시퀀스를 뒤에 붙이는 것이다.

초기에는 물리 정확도를 무리하게 추구하지 말고, 다음 세 가지를 우선 달성한다.

1. 컨베이어와 로봇팔이 박스를 팔레트에 쌓는 장면
2. 적재 완료 후 AMR이 팔레트를 들고 이동하는 장면
3. 목표 위치에 팔레트를 내려놓고 이탈하는 장면

이 세 장면이 자연스럽게 연결되면, 회장님 설명용 영상으로 충분한 설득력을 만들 수 있다. 이후 필요에 따라 ROS2/Nav2, 실제 CAD, 정확한 물리 시뮬레이션으로 확장한다.

---

## 2026-05-29 진행 메모

현재 1차 구현은 `방식 B: Python FSM에서 직접 transform/controller 제어`로 진행한다.
이유는 이번 목표가 실제 Nav2 검증보다 설명 가능한 통합 데모 구현이고, 팔레트 attach/detach 타이밍을 `stack_complete`와 정확히 맞추기 쉽기 때문이다.

완료한 항목:

- [x] `E:\Harim_AMR\.conda\env_isaacsim_5_1_0`에 Isaac Sim 5.1.0 pip 환경 설치
- [x] 기존 `E:\isaac-sim-5.1.0`, `E:\IsaacLab`, 기존 conda env를 사용하지 않도록 작업 범위 분리
- [x] `isaacsim`, `isaacsim-cortex`, `isaacsim-example`, `isaacsim-robot`, `isaacsim-replicator` 설치 확인
- [x] 공식 UR10 palletizing 예제 파일 위치 확인
- [x] 공식 Cortex UR10 bin stacking behavior의 `stack_complete` 구조 확인
- [x] 공식 `iw_hub` asset 및 `LiftUp` / `LiftDown` command class 존재 확인
- [x] 실행 스크립트 추가: `isaac_sim/scripts/run_harim_pallet_demo.py`
- [x] PowerShell 실행 래퍼 추가: `run_harim_demo.ps1`
- [x] 로컬 cache/user/runtime 디렉터리를 `E:\Harim_AMR` 내부로 유도
- [x] UR10 palletizing scene 로드
- [x] Cortex UR10 robot을 `world.add_robot()`으로 등록
- [x] 기존 UR10 bin stacking task 재사용
- [x] stack pattern을 `--stack-cols`, `--stack-rows`, `--stack-layers` 옵션으로 축소/조정 가능하게 구성
- [x] `stack_complete` 감시 후 AMR 이송 FSM 시작
- [x] `iw_hub` USD 로드
- [x] visual lift plate 추가
- [x] visual pallet deck/block 추가
- [x] AMR waypoint 이동 FSM 구현
- [x] LiftUp 단계에서 pallet 및 stacked bin 상승 연출
- [x] attach 단계에서 stacked bins + pallet visual assembly를 AMR 기준 offset으로 묶음
- [x] 이동 중 assembly follow
- [x] drop pose에서 LiftDown 및 detach
- [x] exit pose 이탈
- [x] 기본 `--cycles 0` 무한 반복 구조
- [x] `--self-test-frames` 개발 검증 옵션 추가
- [x] Python 문법 검사 통과
- [x] PowerShell 래퍼 syntax 검사 통과

현재 제한/확인 필요:

- Isaac Kit 첫 실행에서 NVIDIA Omniverse Kit EULA 확인이 필요하다.
- Codex가 사용자를 대신해 EULA에 동의할 수 없어서 실제 headless self-test는 EULA prompt에서 중단되었다.
- 사용자가 내용을 확인하고 동의하는 경우 `run_harim_demo.ps1 -AcceptEula`를 명시적으로 붙여 실행한다.
- EULA 수락 후에는 `-Headless -SelfTestFrames 2`로 stage 초기화 검증을 먼저 수행하는 것이 좋다.

실행 예:

```powershell
cd E:\Harim_AMR
powershell -ExecutionPolicy Bypass -File .\run_harim_demo.ps1 -AcceptEula
```

짧은 초기화 검증:

```powershell
cd E:\Harim_AMR
powershell -ExecutionPolicy Bypass -File .\run_harim_demo.ps1 -Headless -AcceptEula -SelfTestFrames 2
```

---

## 2026-05-29 추가 진행 메모

런타임 전 코드 검토와 lightweight unittest 과정에서 발견한 문제를 수정했다.

- [x] `BinStackingTask`에 존재하지 않는 `on_bin_event`를 `make_decider_network()`에 넘기던 오류 수정
- [x] UR10 behavior monitor callback은 no-op lambda로 연결
- [x] LiftUp 중 stacked bin 높이를 매 프레임 누적해서 더하던 문제 수정
- [x] LiftUp도 LiftDown과 동일하게 delta z만 적용하도록 변경
- [x] Isaac Sim을 띄우지 않고 custom orchestrator FSM을 검증하는 unittest 추가
- [x] `cycles=1` 단일 사이클: stack_complete -> pickup -> lift -> attach -> drop -> detach -> exit -> DONE_IDLE 검증
- [x] `cycles=0` 무한 반복: 1사이클 후 world reset/play 및 WAIT_STACK_COMPLETE 복귀 검증

검증 명령:

```powershell
cd E:\Harim_AMR
.\.conda\env_isaacsim_5_1_0\python.exe -m unittest .\isaac_sim\tests\test_harim_transfer_orchestrator.py
.\.conda\env_isaacsim_5_1_0\python.exe -m py_compile .\isaac_sim\scripts\run_harim_pallet_demo.py .\isaac_sim\tests\test_harim_transfer_orchestrator.py
```

검증 결과:

- [x] unittest 2개 통과
- [x] Python compile 통과
- [x] PowerShell wrapper syntax 통과

아직 실제 Isaac Sim stage 초기화와 물리 실행은 NVIDIA Omniverse Kit EULA 확인 이후에만 가능하다.

---

## 2026-05-29 lift prim 보강 메모

`iw_hub` asset 내부에 lift prim이 존재하는 경우 visual plate뿐 아니라 실제 asset lift prim도 함께 움직이도록 보강했다.

- [x] `/World/HarimDemo/iw_hub/chassis/lift` prim 존재 여부 확인 로직 추가
- [x] lift prim이 있으면 `SingleXFormPrim(..., reset_xform_properties=False)`로 wrapping
- [x] lift prim이 없으면 기존 visual lift plate만 사용하도록 fallback
- [x] LiftUp/LiftDown 중 실제 lift prim world pose도 `lift_offset`에 맞춰 갱신
- [x] 실제 lift prim 연동 unittest 추가

검증 결과:

- [x] custom orchestrator unittest 3개 통과
- [x] Python compile 통과

---

## 2026-05-29 extension 로딩 순서 보강 메모

설치된 Isaac Sim 5.1.0 소스를 확인한 결과 `CortexUr10` import 경로가 `isaacsim.robot.manipulators.grippers.surface_gripper.SurfaceGripper`를 즉시 import한다.
따라서 `CortexUr10`을 import하기 전에 `isaacsim.robot.surface_gripper` extension을 먼저 enable하도록 순서를 수정했다.

- [x] `SimulationApp` 생성 직후 `isaacsim.robot.surface_gripper` extension enable
- [x] 필수 extension enable 실패 시 명확한 `RuntimeError` 발생
- [x] `isaacsim.anim.robot`은 optional extension enable 과정에서 `pkg_resources` 의존성 문제를 유발해 제거하고, USD asset reference 기반으로 `iw_hub`를 직접 로드하도록 정리
- [x] extension enable 이후 `simulation_app.update()` 호출
- [x] `CortexUr10` import 전에 surface gripper extension을 enable하는 source-order unittest 추가
- [x] `iw_hub/chassis/lift`가 Xformable prim일 때만 `SingleXFormPrim`으로 wrapping하도록 보강

검증 결과:

- [x] custom orchestrator unittest 4개 통과
- [x] Python compile 통과

---

## 2026-05-29 stage loading 대기 보강 메모

USD reference를 stage에 추가한 직후 child prim을 검사하거나 robot wrapper를 만들면 asset loading 타이밍에 따라 prim이 아직 준비되지 않을 수 있다.
설치된 Isaac Sim 5.1.0 테스트 코드에서 사용하는 `omni.usd.get_context().get_stage_loading_status()[2]` 패턴을 참고해 stage loading 대기를 추가했다.

- [x] `wait_for_stage_loading(simulation_app, usd_context, label)` helper 추가
- [x] UR10 palletizing scene/background reference 후 stage loading 완료 대기
- [x] `iw_hub` reference 후 stage loading 완료 대기
- [x] loading timeout 시 어떤 asset에서 멈췄는지 보이도록 `RuntimeError` 메시지 구성
- [x] loading 완료까지 update가 반복되는지 unittest 추가
- [x] loading pending 상태가 지속되면 timeout 되는지 unittest 추가

검증 결과:

- [x] custom orchestrator unittest 6개 통과
- [x] Python compile 통과
- [x] PowerShell wrapper syntax 통과

---

## 2026-05-29 headless Isaac Sim 검증 메모

사용자가 NVIDIA Omniverse Kit EULA 동의 의사를 명시했으므로 `run_harim_demo.ps1 -AcceptEula` 경로로 실제 Isaac Sim headless self-test를 수행했다.

추가 수정한 항목:

- [x] pip 설치 환경에서 `isaacsim.examples.interactive.ur10_palletizing` import가 불가능한 문제를 제거
- [x] 필요한 `Ur10Assets`와 `BinStackingTask` 최소 구현을 `run_harim_pallet_demo.py` 내부에 두어 interactive sample extension 의존성 제거
- [x] `isaacsim.core.utils.math`에 없는 `pack_R` 사용을 `isaacsim.cortex.framework.math_util`로 교체
- [x] headless self-test에서 `SimulationApp.is_running()`이 false인 경우에도 지정 프레임을 직접 실행하도록 `--self-test-frames` 경로 수정
- [x] `--self-test-frames`가 `is_running()` gate에 막히지 않는지 source-order unittest 추가
- [x] `isaacsim.anim.robot` enable 제거로 `pkg_resources` 관련 런타임 오류 회피

검증 명령:

```powershell
cd E:\Harim_AMR
.\.conda\env_isaacsim_5_1_0\python.exe -m unittest .\isaac_sim\tests\test_harim_transfer_orchestrator.py
.\.conda\env_isaacsim_5_1_0\python.exe -m py_compile .\isaac_sim\scripts\run_harim_pallet_demo.py .\isaac_sim\tests\test_harim_transfer_orchestrator.py
powershell -ExecutionPolicy Bypass -File .\run_harim_demo.ps1 -Headless -AcceptEula -SelfTestFrames 2 -Cycles 1
```

검증 결과:

- [x] custom orchestrator unittest 9개 통과
- [x] Python compile 통과
- [x] Isaac Sim 5.1.0 headless self-test 통과
- [x] 로그에서 `[HarimDemo] using iw_hub lift prim: /World/HarimDemo/iw_hub/chassis/lift` 확인
- [x] 로그에서 `[HarimDemo] self-test completed after 2 frames` 확인

현재 구현 완료 범위:

- [x] 컨베이어 기반 bin 공급 task
- [x] UR10 + suction gripper 기반 Cortex bin stacking behavior 연결
- [x] stack pattern 옵션화
- [x] `stack_complete` 감시
- [x] `iw_hub` USD asset 로드
- [x] `iw_hub/chassis/lift` prim 연동
- [x] 팔레트 visual assembly 생성
- [x] pickup, lift-up, attach, drop 이동, lift-down, detach, exit FSM 구현
- [x] `--cycles 0` 기본 무한 반복 유지

남겨둔 범위:

- [ ] 실제 장시간 GUI 실행에서 전체 적재 완료까지 눈으로 확인
- [ ] 카메라 컷/영상 렌더링
- [ ] ROS2/Nav2 기반 자율주행

---

## 2026-05-29 슬라이드 하역/No-Flip 진행 메모

사용자 추가 요구사항:

- [x] 팔레트/박스가 바닥이나 AMR을 뚫는 것처럼 보이는 문제 완화
- [x] 목표 위치에서 참고 영상처럼 팔레트를 내려놓고 AMR이 슬라이드 형태로 빠져나가는 연출 구현
- [x] AMR 운반 거리를 10 m 이상으로 설정
- [x] bin이 처음부터 upside-down 상태로 스폰되게 변경
- [x] flip station을 거치지 않고 바로 place 동작으로 진행
- [x] `Todo.md` 체크박스 진행 상태 갱신

구현한 항목:

- [x] 기본 drop X를 pickup X 기준 `+10.6 m`로 변경
- [x] PowerShell 래퍼에 `-MoveSpeed`, `-PickupX`, `-PickupY`, `-DropX`, `-DropY` 옵션 추가
- [x] 하역 후 `DETACH`에서 팔레트/박스 assembly의 world pose를 기록
- [x] `SLIDE_OUT_FROM_PALLET` 상태 추가
- [x] 하역 후에는 팔레트/박스 pose를 고정하고 AMR만 전방으로 슬라이드 이탈
- [x] 팔레트 하부 중앙 통로를 확보하도록 중앙 블록을 제거한 visual pallet layout 적용
- [x] dropped assembly를 매 step 고정하고 velocity를 0으로 만들어 물리 관통/낙하 연출을 줄임
- [x] bin spawn orientation을 공식 예제의 flip quaternion 방식에 맞춰 항상 upside-down으로 설정
- [x] `NoFlipDispatch`를 추가해 `flip_bin` child 없이 `pick_bin -> place_bin`만 사용
- [x] UR10 stage 안의 flip 관련 prim은 로드 후 invisible 처리
- [x] 빠른 Isaac 검증용 `--self-test-force-stack-complete` 옵션 추가

검증한 항목:

- [x] default pickup/drop 거리 10 m 이상 unittest 추가
- [x] upside-down spawn orientation unittest 추가
- [x] no-flip dispatch source-order unittest 추가
- [x] 팔레트 하부 중앙 통로 unittest 추가
- [x] 슬라이드 이탈 중 dropped payload stationary unittest 추가
- [x] custom orchestrator unittest 14개 통과
- [x] Python compile 통과
- [x] Isaac Sim 5.1.0 headless transfer self-test 통과

검증 명령:

```powershell
cd E:\Harim_AMR
.\.conda\env_isaacsim_5_1_0\python.exe -m unittest .\isaac_sim\tests\test_harim_transfer_orchestrator.py
.\.conda\env_isaacsim_5_1_0\python.exe -m py_compile .\isaac_sim\scripts\run_harim_pallet_demo.py .\isaac_sim\tests\test_harim_transfer_orchestrator.py
powershell -ExecutionPolicy Bypass -File .\run_harim_demo.ps1 -Headless -AcceptEula -SelfTestFrames 260 -SelfTestForceStackComplete -Cycles 1 -MoveSpeed 20
```

headless transfer self-test 로그 확인:

- [x] `[HarimDemo] state -> MOVE_TO_DROP`
- [x] `[HarimDemo] slide-released pallet assembly at drop pose`
- [x] `[HarimDemo] state -> SLIDE_OUT_FROM_PALLET`
- [x] `[HarimDemo] completed transfer cycle 1`
- [x] `[HarimDemo] self-test completed after 260 frames`

---

## 2026-05-29 UR10 흡착 해제 보강 메모

GUI 확인 중 발견된 문제:

- [x] UR10이 박스를 흡착한 뒤 팔레트 위치에서 놓지 않고 계속 들고 있는 문제

원인 판단:

- 공식 `CloseSuctionGripper` 상태는 `suction_gripper.is_closed()`가 true가 될 때까지 대기한다.
- 현재 데모는 bin을 처음부터 upside-down 상태로 스폰하고 flip station을 제거했기 때문에, 공식 suction close 판정이 데모 의도대로 안정적으로 true가 되지 않는 경우가 있었다.
- pick 상태가 종료되지 않거나 place 상태의 도달 대기가 길어지면 `<open gripper>`가 호출되지 않아 GUI에서는 로봇팔이 박스를 놓지 않는 것처럼 보였다.
- 또한 이동 중 `active_bin`이 새 bin으로 바뀌면 집은 박스와 놓는 박스 참조가 어긋날 수 있어, 들고 있는 박스를 별도 `demo_carried_bin`으로 고정했다.

수정 내용:

- [x] `DemoAttachBin` / `DemoReleaseBin` 추가
  - suction close/open API는 호출하되, 공식 `is_closed()` 무한 대기에 의존하지 않음
  - attach 시점에 박스 transform을 end-effector 기준 offset으로 저장
  - 이동 중 `sync_demo_attached_bin()`으로 박스가 gripper를 따라가도록 보정
  - release 시점에는 목표 stack coordinate에 박스를 snap하고 kinematic 상태로 안정화
- [x] `DemoPickAndPlaceBin` 추가
  - `ReachToPick`
  - 데모용 attach
  - timed lift
  - timed reach-to-place
  - 데모용 release
  - completion marking
  - 위 순서를 하나의 locked state sequence로 묶어 pick 후 place decision 전환에서 멈추지 않게 함
- [x] `demo_carried_bin`으로 실제로 집은 bin 참조를 고정
  - 로그에서 `demo-attached bin_0` 이후 `demo-placed bin_0`처럼 동일 박스가 놓이는지 확인
- [x] `--self-test-min-placed-bins` 옵션 추가
  - headless 검증에서 지정 개수 이상 placed bin이 없으면 실패하도록 확인용 옵션 제공
- [x] PowerShell wrapper에서 Python unbuffered 실행 적용
  - `<close gripper>`, `<open gripper>`, `demo-placed` 로그가 즉시 보이도록 `PYTHONUNBUFFERED=1`, `python -u` 적용

검증 명령:

```powershell
cd E:\Harim_AMR
.\.conda\env_isaacsim_5_1_0\python.exe -m unittest .\isaac_sim\tests\test_harim_transfer_orchestrator.py
.\.conda\env_isaacsim_5_1_0\python.exe -m py_compile .\isaac_sim\scripts\run_harim_pallet_demo.py .\isaac_sim\tests\test_harim_transfer_orchestrator.py
powershell -ExecutionPolicy Bypass -File .\run_harim_demo.ps1 -Headless -AcceptEula -SelfTestFrames 1800 -SelfTestMinPlacedBins 1 -Cycles 1
powershell -ExecutionPolicy Bypass -File .\run_harim_demo.ps1 -Headless -AcceptEula -SelfTestFrames 260 -SelfTestForceStackComplete -Cycles 1 -MoveSpeed 20
```

검증 결과:

- [x] unittest 21개 통과
- [x] Python compile 통과
- [x] UR10 normal headless place probe 통과
- [x] 로그 확인: `<close gripper>`
- [x] 로그 확인: `[HarimDemo] demo-attached bin_0`
- [x] 로그 확인: `[HarimDemo] reach_place timed release`
- [x] 로그 확인: `<open gripper>`
- [x] 로그 확인: `[HarimDemo] demo-placed bin_0 at [1.05, -0.62, -0.51]`
- [x] 로그 확인: `[HarimDemo] self-test completed after 1800 frames`
- [x] AMR force-stack transfer self-test 통과
- [x] 로그 확인: `[HarimDemo] attached 4 stacked items and 12 pallet parts`
- [x] 로그 확인: `[HarimDemo] slide-released pallet assembly at drop pose`
- [x] 로그 확인: `[HarimDemo] completed transfer cycle 1`

---

## 2026-05-29 현실감 보강 수정 메모

GUI 확인 중 발견된 문제:

- [x] 박스가 팔레트를 뚫고 떨어지는 문제
- [x] AMR/리프트 부분이 공중에 떠 보이는 문제
- [x] 팔레트 조각들이 서로 연결되지 않은 것처럼 보이는 문제

수정 내용:

- [x] 새 팔레트를 `VisualCuboid` 중심에서 `FixedCuboid` 충돌 지지 구조 중심으로 변경
  - 연결된 상판: `harim_pallet_connected_top_deck`
  - 양쪽 하부 러너: `PalletRunner_*`
  - 측면 블록: `PalletBlock_*`
  - 상판 홈 visual: `PalletGroove_*`
  - 보이지 않는 상판 충돌 지지면: `PalletTopSupport`
- [x] 기본 예제 팔레트를 지운 뒤에도 박스가 지지될 수 있도록 팔레트 상판에 충돌체 추가
- [x] 적재 완료 시점부터 stacked bin을 lock/hold하도록 변경
  - `stack_complete` 감지 시 `locked_stack_poses`에 박스 pose 저장
  - 이후 AMR 접근/대기/리프트 시작 전까지 박스 pose와 속도를 계속 고정
  - USD rigid body가 있으면 `kinematicEnabled=True`도 시도
- [x] AMR 기본 Z 위치를 warehouse floor 기준 `WORLD_FLOOR_Z = -1.1818`에 맞춤
- [x] 실제 `iw_hub/chassis/lift` prim이 있으면 보조 visual lift plate는 숨김
  - 의도: 검은 보조 lift plate가 실제 리프트와 따로 공중에 떠 보이는 문제 제거
- [x] 드롭 위치 작업대의 레일/다리에 `FixedCuboid` 충돌체 적용
- [x] 작업대 다리 높이를 floor까지 닿도록 계산
- [x] 드롭 작업대 위에도 보이지 않는 `DropSlideTopSupport` 충돌 지지면 추가

검증 내용:

- [x] custom orchestrator unittest 20개 통과
- [x] Python compile 통과
- [x] Isaac Sim 5.1.0 headless realism self-test 통과

검증 명령:

```powershell
cd E:\Harim_AMR
.\.conda\env_isaacsim_5_1_0\python.exe -m unittest .\isaac_sim\tests\test_harim_transfer_orchestrator.py
.\.conda\env_isaacsim_5_1_0\python.exe -m py_compile .\isaac_sim\scripts\run_harim_pallet_demo.py .\isaac_sim\tests\test_harim_transfer_orchestrator.py
powershell -ExecutionPolicy Bypass -File .\run_harim_demo.ps1 -Headless -AcceptEula -SelfTestFrames 260 -SelfTestForceStackComplete -Cycles 1 -MoveSpeed 20
```

headless realism self-test 확인 로그:

- [x] `[HarimDemo] using iw_hub lift prim`
- [x] `[HarimDemo] attached 4 stacked items and 12 pallet parts`
- [x] `[HarimDemo] state -> MOVE_TO_DROP`
- [x] `[HarimDemo] slide-released pallet assembly at drop pose`
- [x] `[HarimDemo] completed transfer cycle 1`
- [x] `[HarimDemo] self-test completed after 260 frames`

---

## 2026-05-29 AMR 경로/팔레트 중복/드롭 작업대 수정 메모

사용자 추가 요구사항:

- [x] `pallet_holder`는 제거해도 됨
- [x] AMR이 로봇팔 밑 테이블을 뚫고 이동하는 문제 해결
- [x] AMR이 차라리 멀리서 다가오도록 경로 변경
- [x] 기본 UR10 예제에 포함된 팔레트와 새로 만든 팔레트가 겹치는 문제 해결
- [x] 목표 위치에 팔레트를 슬라이드로 올려놓는 작업대 추가

구현 내용:

- [x] AMR 시작 위치를 pickup 기준 `-X` 테이블 쪽이 아니라 `+X` 방향 먼 위치로 변경
  - 시작 위치: `pickup_x + AMR_START_STANDOFF`
  - 접근 위치: `pickup_x + AMR_APPROACH_STANDOFF`
  - pickup 직전에는 목표 팔레트 위치로만 짧게 진입
  - 의도: 로봇팔 테이블/컨베이어 아래를 통과하는 것처럼 보이는 경로를 피하고, 드롭 존 쪽 넓은 공간에서 접근하게 함
- [x] 기본 UR10 palletizing scene 안의 중복 프림을 비활성화
  - 대상 패턴: `flip`, `pallet`, `pallet_holder`
  - 처리 방식: visibility만 끄는 것이 아니라 `prim.SetActive(False)`로 비활성화
  - 의도: 기본 예제 팔레트/holder의 visual 및 collision이 새 팔레트와 겹치지 않게 함
- [x] 목표 위치에 슬라이드형 팔레트 작업대 visual 추가
  - `DropSlideRail_*`: 양쪽 가이드 레일
  - `DropSlideRoller_*`: 롤러/슬라이드 면
  - `DropSlideLeg_*`: 지지 다리
  - 위치는 `drop_x/drop_y` 기준으로 생성
  - 의도: AMR이 팔레트를 내려놓고 앞으로 빠져나갈 때, 팔레트가 목표 작업대 위에 남아 있는 장면을 명확히 보이게 함
- [x] AMR lift plate 초기 위치도 변경된 시작 위치와 맞춤

검증 내용:

- [x] AMR이 pickup보다 큰 X 좌표에서 시작하고 접근하는지 unittest 추가
- [x] 기본 예제의 `pallet_holder` 제거 의도가 코드에 남아 있는지 unittest 추가
- [x] drop slide workstation 생성 코드가 존재하는지 unittest 추가
- [x] lift prim 추적 테스트의 시작 위치를 새 AMR 시작 위치에 맞춤
- [x] custom orchestrator unittest 16개 통과
- [x] Python compile 통과
- [x] Isaac Sim 5.1.0 headless transfer self-test 통과

검증 명령:

```powershell
cd E:\Harim_AMR
.\.conda\env_isaacsim_5_1_0\python.exe -m unittest .\isaac_sim\tests\test_harim_transfer_orchestrator.py
.\.conda\env_isaacsim_5_1_0\python.exe -m py_compile .\isaac_sim\scripts\run_harim_pallet_demo.py .\isaac_sim\tests\test_harim_transfer_orchestrator.py
powershell -ExecutionPolicy Bypass -File .\run_harim_demo.ps1 -Headless -AcceptEula -SelfTestFrames 260 -SelfTestForceStackComplete -Cycles 1 -MoveSpeed 20
```

headless transfer self-test 확인 로그:

- [x] `[HarimDemo] state -> MOVE_TO_DROP`
- [x] `[HarimDemo] slide-released pallet assembly at drop pose`
- [x] `[HarimDemo] state -> SLIDE_OUT_FROM_PALLET`
- [x] `[HarimDemo] completed transfer cycle 1`
- [x] `[HarimDemo] self-test completed after 260 frames`

---

## 2026-05-29 GUI 로봇팔 흡착 해제/다중 박스 적재 보강

사용자가 GUI에서 확인했을 때 로봇팔이 박스를 놓지 않는 것처럼 보이는 문제가 있었다. headless 로그상으로는 1번째 박스에 대해 `<open gripper>`와 `demo-placed`가 출력되었지만, 2번째 박스 적재가 이어지지 않아 GUI에서는 시퀀스가 멈춘 것처럼 보일 수 있었다.

원인:

- `DemoPickAndPlaceBin` state machine이 1회 완료된 뒤 같은 decider branch에 계속 머물면 내부 state가 `None`인 상태로 재시작되지 않았다.
- 1번째 박스 완료 직후 다음 박스가 이미 active로 잡히면 parent decider가 `pick_place_bin`에서 다른 branch로 빠졌다가 재진입하지 않아 2번째 pick/place가 시작되지 않았다.
- 컨베이어가 로봇팔 작업 중에도 새 박스를 계속 흘려 보내 `bin_1`, `bin_2` 등이 지나가고, 나중에 잡힌 active bin으로 인해 pick 동작이 늦어졌다.
- `DemoReleaseBin`이 한 프레임에서만 `open()`을 호출하면 surface gripper 제약이 늦게 풀리는 경우 GUI에서 계속 붙어 보일 수 있었다.

수정:

- [x] `DemoPickAndPlaceBin.decide()`에서 내부 state가 `None`이면 `enter()`를 다시 호출해 다음 active bin에 대해 pick/place sequence를 재시작하도록 수정
- [x] active bin이 없을 때는 `go_home`으로 긴 동작을 타지 않고 `DemoWaitForNextBin`에서 대기하도록 수정
- [x] `BinStackingTask`가 decider context를 참조하도록 연결하고, active bin 또는 carried bin이 있을 때는 새 박스를 스폰하지 않도록 수정
- [x] 컨베이어 속도를 `-0.30`에서 `-0.45`로 조정해 박스 유입 속도를 조금 높임
- [x] active bin이 잡히면 `PICK_STATION_BIN_POSITION`으로 정렬하고 정지시켜 픽업 스테이션에서 집는 장면이 되도록 수정
- [x] `DemoReleaseBin`을 0.35초 동안 유지되는 state로 바꿔 `open()`을 반복 호출하고, 박스를 목표 stack 좌표에 고정하도록 수정
- [x] `ReachToPick`에도 `DemoTimedState`를 적용해 접근이 오래 걸려도 시퀀스가 무한 대기하지 않도록 수정
- [x] self-test 실패가 `simulation_app.close()`에 가려지지 않도록 `[HarimDemo] self-test failed: ...` 로그와 `SystemExit(1)`을 추가
- [x] `-SelfTestDebugBins` 옵션을 Python script와 PowerShell wrapper에 추가해 bin spawn, active-bin, stack-count 전환을 추적할 수 있게 함

검증 명령:

```powershell
cd E:\Harim_AMR
.\.conda\env_isaacsim_5_1_0\python.exe -m unittest .\isaac_sim\tests\test_harim_transfer_orchestrator.py
.\.conda\env_isaacsim_5_1_0\python.exe -m py_compile .\isaac_sim\scripts\run_harim_pallet_demo.py .\isaac_sim\tests\test_harim_transfer_orchestrator.py
powershell -ExecutionPolicy Bypass -File .\run_harim_demo.ps1 -Headless -AcceptEula -SelfTestFrames 4200 -SelfTestMinPlacedBins 2 -SelfTestDebugBins -Cycles 1
```

확인된 핵심 로그:

- [x] `[HarimDemo] spawned bin_0`
- [x] `[HarimDemo] active-bin -> bin_0`
- [x] `[HarimDemo] reach_pick start`
- [x] `<close gripper>`
- [x] `[HarimDemo] demo-attached bin_0`
- [x] `<open gripper>`
- [x] `[HarimDemo] demo-placed bin_0 at [1.05, -0.62, -0.51]`
- [x] `[HarimDemo] stack-count 1/8 after bin_0`
- [x] `[HarimDemo] spawned bin_1`
- [x] `[HarimDemo] active-bin -> bin_1`
- [x] `[HarimDemo] demo-placed bin_1 at [0.8400000000000001, -0.62, -0.51]`
- [x] `[HarimDemo] stack-count 2/8 after bin_1`
- [x] `[HarimDemo] self-test completed after 4200 frames; placed_bins=4`

남은 주의점:

- 2번째 이후 일부 박스에서 Isaac Sim surface gripper가 실제 물리 close 판정을 못 받아 warning을 출력할 수 있다. 데모 로직은 `demo_carried_bin` 기반 fallback attach/release로 진행되므로 적재 시퀀스는 계속 동작한다.
- GUI 시각 품질을 더 높이려면 후속 작업에서 pick station 위치, UR10 중간 경유 자세, suction gripper의 실제 grip threshold를 추가로 튜닝하면 된다.

---

## 2026-05-29 UR10 pre-grip 정렬과 release 안정화 추가 보강

GUI 확인에서 박스가 그리퍼에 계속 붙어 보이는 문제를 더 줄이기 위해 pick 직전과 release 구간을 다시 조정했다.

수정 내용:

- [x] `DemoSettleBinAtGripper` state 추가
  - `ReachToPick` 직후 active bin의 grasp frame을 현재 UR10 end-effector frame에 맞춘다.
  - 최소 0.30초, 최대 1.10초 동안 보간 정렬해서 표면 그리퍼 close 직전 박스가 그리퍼 근처에 안정적으로 보이도록 한다.
- [x] `demo_pre_grip_initial_offset` 기록
  - headless debug에서 `pre-grip offset ... m` 로그로 실제 접근 오차를 확인한다.
  - GUI에서 release 후에도 surface gripper joint가 박스를 붙잡아 보이는 문제를 피하기 위해 실제 `suction_gripper.close()` 호출은 기본 경로에서 제외한다.
  - suction 동작은 `demo_carried_bin` 기반 scripted attach/release로 처리하고, gripper는 attach 전후 `open()` 상태를 유지한다.
- [x] 스폰 직후 박스의 rigid body kinematic 상태는 강제로 바꾸지 않도록 정리
  - 새 박스를 스폰할 때 dynamic/kinematic을 무리하게 전환하면 컨베이어 active 영역에 들어오기 전에 박스가 빠지는 경우가 있었다.
  - 물리 상태는 기본 예제에 맡기고, active bin으로 잡힌 뒤 pick station과 gripper 정렬 단계에서만 kinematic 고정을 사용한다.
- [x] release는 기존처럼 0.35초 동안 유지
  - `open()`을 반복 호출하고, 박스를 목표 stack 좌표에 고정해 GUI에서 다시 그리퍼에 끌려가는 것처럼 보이는 현상을 줄인다.

검증 명령:

```powershell
cd E:\Harim_AMR
.\.conda\env_isaacsim_5_1_0\python.exe -m unittest .\isaac_sim\tests\test_harim_transfer_orchestrator.py
.\.conda\env_isaacsim_5_1_0\python.exe -m py_compile .\isaac_sim\scripts\run_harim_pallet_demo.py .\isaac_sim\tests\test_harim_transfer_orchestrator.py
powershell -ExecutionPolicy Bypass -File .\run_harim_demo.ps1 -Headless -AcceptEula -SelfTestFrames 4200 -SelfTestMinPlacedBins 4 -SelfTestDebugBins -Cycles 1
powershell -ExecutionPolicy Bypass -File .\run_harim_demo.ps1 -Headless -AcceptEula -SelfTestFrames 260 -SelfTestForceStackComplete -Cycles 1 -MoveSpeed 20
```

확인된 결과:

- [x] unittest 21개 통과
- [x] Python compile 통과
- [x] 5600-frame UR10 적재 self-test 통과
  - `placed_bins=4`
  - `pre-grip offset` 로그 확인
  - `demo-attached bin_N`과 `demo-placed bin_N`이 같은 bin으로 유지되는 것을 확인
- [x] 260-frame AMR transfer self-test 통과
  - `attached 4 stacked items and 12 pallet parts`
  - `slide-released pallet assembly at drop pose`
  - `completed transfer cycle 1`

남은 주의점:

- 이번 보강은 설명용 데모의 안정성과 시각적 연속성을 우선한 방식이다. 박스는 흡착식 그리퍼에 붙는 것처럼 이동하지만, 실제 surface gripper joint 검증이라기보다는 scripted suction attach 연출에 가깝다.
- 전체 2 x 2 x 2 스택을 끝까지 자연스럽게 촬영하려면 다음 단계에서 카메라 컷, 박스 재질, 하림 박스 텍스처, 팔레트 적재 완료 이후 AMR 호출 타이밍을 다듬으면 된다.

---

## 2026-05-29 GUI release 최종 보강 메모

GUI에서 로봇팔이 박스를 놓지 않는 것처럼 보이던 문제를 기준으로 release 경로를 다시 정리했다.

수정 내용:

- [x] `demo_pre_grip_bin` 추가
  - pick 직후 settle 중인 bin을 별도로 고정한다.
  - Cortex context의 `active_bin`이 잠깐 `None`이 되거나 다음 박스로 바뀌어도 attach/release 대상이 바뀌지 않도록 한다.
- [x] `BinStackingTask.pre_step()`에서 `demo_pre_grip_bin` 또는 `demo_carried_bin`이 있으면 새 박스를 스폰하지 않도록 차단한다.
- [x] 실제 `suction_gripper.close()` 호출 제거
  - surface gripper joint가 GUI에서 release 후에도 박스를 붙잡아 보이는 경로를 없앴다.
  - attach 전에는 `suction_gripper.open()`을 호출해 stale joint 가능성을 줄인다.
- [x] `DemoReleaseBin`은 기존처럼 0.35초 동안 `open()`을 반복하고, release된 bin을 목표 stack 좌표에 고정한다.
- [x] 컨베이어 박스는 pick window 근처 `y=0.68`에 정렬 스폰한다.
- [x] 박스 yaw 랜덤화를 제거하고 항상 같은 upside-down orientation으로 스폰한다.
  - 실제 팔레타이징 컨베이어의 가이드 정렬에 해당하는 처리다.
- [x] `DemoTimedArmMoveTo`를 full pose 명령에서 position-only 명령으로 변경했다.
  - place 자세의 orientation을 그대로 끌고 돌아오면서 UR10이 pick 자세로 복귀하지 못하던 문제를 줄인다.
  - `posture_config=self.context.robot.default_config`로 기본 posture를 유도한다.

검증 명령:

```powershell
cd E:\Harim_AMR
.\.conda\env_isaacsim_5_1_0\python.exe -m unittest .\isaac_sim\tests\test_harim_transfer_orchestrator.py
.\.conda\env_isaacsim_5_1_0\python.exe -m py_compile .\isaac_sim\scripts\run_harim_pallet_demo.py .\isaac_sim\tests\test_harim_transfer_orchestrator.py
powershell -ExecutionPolicy Bypass -File .\run_harim_demo.ps1 -Headless -AcceptEula -SelfTestFrames 5600 -SelfTestMinPlacedBins 4 -SelfTestDebugBins -Cycles 1
powershell -ExecutionPolicy Bypass -File .\run_harim_demo.ps1 -Headless -AcceptEula -SelfTestFrames 260 -SelfTestForceStackComplete -Cycles 1 -MoveSpeed 20
```

확인 결과:

- [x] unittest 21개 통과
- [x] Python compile 통과
- [x] UR10 5600-frame self-test 통과
  - `self-test completed after 5600 frames; placed_bins=4`
  - `bin_0`부터 `bin_3`까지 attach와 place 대상이 일치
  - 확인된 pre-grip offset은 약 0.0044-0.0049 m 범위
- [x] AMR 260-frame transfer self-test 통과
  - `attached 4 stacked items and 12 pallet parts`
  - `slide-released pallet assembly at drop pose`
  - `completed transfer cycle 1`

---

## 2026-05-29 전체 2층 적재 후 AMR 이송 검증 메모

부분 적재가 아니라 기본 목표인 `2 x 2 x 2` 전체 적재 후 AMR 이송까지 한 번에 검증하도록 self-test를 보강했다.

수정 내용:

- [x] UR10 데모 state timer를 wall-clock `time.time()` 기준에서 simulation dt 기반 `demo_sim_time` 기준으로 변경
  - headless 실행 부하에 따라 같은 frame 수에서도 pick timeout과 pre-grip offset이 달라지는 문제를 줄인다.
  - GUI 실행에서도 물리 step 기준으로 timing이 맞으므로 시각적으로 더 일관된 흐름이 된다.
- [x] `--self-test-min-transfer-cycles` 옵션 추가
  - placed bin 개수뿐 아니라 AMR transfer cycle 완료 여부까지 self-test gate로 확인한다.
  - PowerShell wrapper에도 `-SelfTestMinTransferCycles` 옵션을 추가했다.
- [x] pick/place/return timing 단축
  - 기존 긴 timeout 대기보다 산업용 팔레타이징에 가까운 빠른 cycle로 조정했다.
  - `ReachToPick`: 3.20초
  - `ReachToPlace`: 3.00초
  - lift/retract/return 구간도 짧게 조정했다.
- [x] `stack_complete` 이후 컨베이어 공급 정지
  - 8개 적재 완료 뒤 AMR이 접근하는 중 불필요한 `bin_8`이 스폰되는 장면을 제거했다.

검증 명령:

```powershell
cd E:\Harim_AMR
.\.conda\env_isaacsim_5_1_0\python.exe -m unittest .\isaac_sim\tests\test_harim_transfer_orchestrator.py
.\.conda\env_isaacsim_5_1_0\python.exe -m py_compile .\isaac_sim\scripts\run_harim_pallet_demo.py .\isaac_sim\tests\test_harim_transfer_orchestrator.py
powershell -ExecutionPolicy Bypass -File .\run_harim_demo.ps1 -Headless -AcceptEula -SelfTestFrames 7000 -SelfTestMinPlacedBins 8 -SelfTestMinTransferCycles 1 -SelfTestDebugBins -Cycles 1
```

확인 결과:

- [x] unittest 21개 통과
- [x] Python compile 통과
- [x] 7000-frame end-to-end self-test 통과
  - `stack-count 8/8 after bin_7`
  - `stack_complete detected`
  - `attached 8 stacked items and 12 pallet parts`
  - `slide-released pallet assembly at drop pose`
  - `completed transfer cycle 1`
  - `self-test completed after 7000 frames; placed_bins=8; transfer_cycles=1`
- [x] `stack_complete` 이후 추가 `spawned bin_8` 로그가 나오지 않음을 확인

---

## 2026-05-29 AMR 이동 가속/감속 보강 메모

전체 흐름이 통과한 뒤 AMR 이동의 시각적 현실성을 조금 더 높이기 위해 waypoint 이동을 일정 속도 직선 보정에서 완만한 가속/감속 보간으로 바꿨다.

수정 내용:

- [x] `smoothstep()` helper 추가
  - 0-1 진행률을 부드러운 S-curve로 바꿔 출발과 정지 시 속도가 갑자기 튀지 않도록 한다.
- [x] AMR waypoint 전환 시 `move_start_pose`, `move_target`, `move_duration`을 저장
  - 이동 중 매 frame 현재 위치에서 다시 방향을 계산하는 방식 대신, 시작점과 목표점 사이를 simulation time 기준으로 보간한다.
- [x] `_move_amr_toward_target()`을 smoothstep 보간 기반으로 변경
  - 평균 이동 시간은 기존 `distance / move_speed` 기준을 유지하되, 초반/후반이 부드럽게 움직인다.
- [x] UR10 pre-grip settle에서도 같은 `smoothstep()` helper를 재사용한다.
- [x] AMR easing 동작을 확인하는 unittest 추가
  - 이동 25% 시점에서 선형 25%보다 덜 이동하는지 확인해 가속 구간이 적용됐음을 검증한다.

검증 명령:

```powershell
cd E:\Harim_AMR
.\.conda\env_isaacsim_5_1_0\python.exe -m unittest .\isaac_sim\tests\test_harim_transfer_orchestrator.py
.\.conda\env_isaacsim_5_1_0\python.exe -m py_compile .\isaac_sim\scripts\run_harim_pallet_demo.py .\isaac_sim\tests\test_harim_transfer_orchestrator.py
powershell -ExecutionPolicy Bypass -File .\run_harim_demo.ps1 -Headless -AcceptEula -SelfTestFrames 7000 -SelfTestMinPlacedBins 8 -SelfTestMinTransferCycles 1 -SelfTestDebugBins -Cycles 1
```

확인 결과:

- [x] unittest 22개 통과
- [x] Python compile 통과
- [x] 7000-frame end-to-end self-test 통과
  - `stack-count 8/8 after bin_7`
  - `attached 8 stacked items and 12 pallet parts`
  - `slide-released pallet assembly at drop pose`
  - `completed transfer cycle 1`
  - `self-test completed after 7000 frames; placed_bins=8; transfer_cycles=1`

---

## 2026-05-29 적재 좌표 흔들림 보강 메모

최신 end-to-end 로그에서 위층 박스 일부가 `1.0509`, `0.8368`처럼 canonical stack grid에서 몇 mm씩 벗어나는 것을 확인했다. 원인은 공식 `ReachToPlace`가 아래 박스 위치에 맞추려고 `context.stack_coordinates` 배열 자체를 조금씩 수정하는 데 있었다. 영상용 팔레타이징은 팔레트 위 격자 적재가 깔끔하게 보여야 하므로 release 좌표는 별도의 canonical grid를 쓰도록 수정했다.

수정 내용:

- [x] `clone_stack_coordinates()` 추가
  - stack coordinate list를 deep-copy해 공식 behavior가 수정해도 원본 grid가 보존되도록 한다.
- [x] `demo_stack_coordinates` 추가
  - Cortex behavior용 `context.stack_coordinates`와 demo release용 canonical coordinates를 분리한다.
- [x] `DemoReleaseBin`은 `get_demo_stack_coordinate()`를 통해 canonical 좌표에 스냅한다.
- [x] reset cycle 때도 `context.stack_coordinates`와 `demo_stack_coordinates`를 모두 새 copy로 복구한다.
- [x] coordinate clone이 deep-copy인지 확인하는 unittest 추가.

검증 명령:

```powershell
cd E:\Harim_AMR
.\.conda\env_isaacsim_5_1_0\python.exe -m unittest .\isaac_sim\tests\test_harim_transfer_orchestrator.py
.\.conda\env_isaacsim_5_1_0\python.exe -m py_compile .\isaac_sim\scripts\run_harim_pallet_demo.py .\isaac_sim\tests\test_harim_transfer_orchestrator.py
powershell -ExecutionPolicy Bypass -File .\run_harim_demo.ps1 -Headless -AcceptEula -SelfTestFrames 7000 -SelfTestMinPlacedBins 8 -SelfTestMinTransferCycles 1 -SelfTestDebugBins -Cycles 1
```

확인 결과:

- [x] unittest 23개 통과
- [x] Python compile 통과
- [x] 7000-frame end-to-end self-test 통과
  - `bin_0`부터 `bin_7`까지 모두 canonical grid 좌표에 place됨
  - `stack-count 8/8 after bin_7`
  - `attached 8 stacked items and 12 pallet parts`
  - `completed transfer cycle 1`
  - `self-test completed after 7000 frames; placed_bins=8; transfer_cycles=1`

---

## 2026-05-29 GUI release 상태 정리 및 박스 외형 보강 메모

GUI 확인에서 로봇팔이 박스를 놓지 않는 것처럼 보이는 문제를 다시 줄이기 위해 release 순간의 Cortex context 상태를 더 명확히 정리했다. 핵심은 박스를 목표 좌표에 스냅한 직후 `active_bin`, `demo_carried_bin`, `demo_pre_grip_bin`을 즉시 비우고, 완료 마킹용으로만 `demo_released_bin`을 잠시 보존하는 것이다. 이 상태가 남아 있으면 GUI 프레임에서 원본 Cortex monitor가 여전히 같은 박스를 active/carry 대상으로 해석할 여지가 있었다.

수정 내용:

- [x] `clear_demo_carry_context()` helper 추가
  - release 직후 `active_bin`, `demo_carried_bin`, `demo_pre_grip_bin`, `demo_pre_grip_initial_offset`을 한번에 비운다.
- [x] `DemoReleaseBin`에서 박스 place 직후 `demo_released_bin`만 남기고 carry context를 즉시 해제
  - release 중에는 계속 `suction_gripper.open()`을 호출한다.
  - place 좌표는 기존처럼 canonical `demo_stack_coordinates` 기준으로 고정한다.
- [x] `DemoMarkCarriedBinComplete`는 `demo_released_bin`을 우선 사용해 stack count를 올린 뒤 해당 값을 비운다.
- [x] 새 박스 스폰 조건에 `demo_released_bin`을 포함
  - release 후 arm이 빠져나가고 stack count 마킹이 끝나기 전까지 다음 박스가 너무 빨리 스폰되지 않게 한다.
- [x] release 후 팔을 `POST_RELEASE_CLEARANCE_LIFT = 0.22` m 만큼 올리도록 조정
  - GUI에서 흡착 컵과 박스 사이의 분리가 더 명확하게 보이도록 했다.
- [x] KLT bin 위에 lightweight carton visual overlay 추가
  - 기존 grasp/collision 로직은 그대로 두고, child `VisualCuboid` 2개(`HarimCartonBody`, `HarimCartonTopTape`)만 붙여 박스처럼 보이게 했다.

검증 명령:

```powershell
cd E:\Harim_AMR
.\.conda\env_isaacsim_5_1_0\python.exe -m unittest isaac_sim.tests.test_harim_transfer_orchestrator
.\.conda\env_isaacsim_5_1_0\python.exe -m py_compile isaac_sim/scripts/run_harim_pallet_demo.py isaac_sim/tests/test_harim_transfer_orchestrator.py
powershell -ExecutionPolicy Bypass -File .\run_harim_demo.ps1 -Headless -AcceptEula -SelfTestFrames 7000 -SelfTestMinPlacedBins 8 -SelfTestMinTransferCycles 1 -SelfTestDebugBins -Cycles 1
```

확인 결과:

- [x] unittest 24개 통과
- [x] Python compile 통과
- [x] 7000-frame end-to-end self-test 통과
  - `bin_0`부터 `bin_7`까지 모두 `demo-placed`
  - `stack-count 8/8 after bin_7`
  - `active-bin -> None`
  - `attached 8 stacked items and 12 pallet parts`
  - `slide-released pallet assembly at drop pose`
  - `completed transfer cycle 1`
  - `self-test completed after 7000 frames; placed_bins=8; transfer_cycles=1`

---

## 2026-05-29 적재 완료 신호등 및 pre-grip 현실감 게이트 메모

GUI에서 자연스럽게 보이려면 단순히 전체 사이클이 끝나는 것만으로는 부족하다. 후반 박스에서 로봇팔이 pick 위치에 충분히 도달하지 못한 채 scripted attach가 들어가면 박스가 그리퍼로 순간 보정되는 것처럼 보인다. 그래서 이번 변경에서는 적재 완료 신호를 실제 장면 안에 보이게 만들고, self-test가 pre-grip 보정량까지 검사하도록 보강했다.

수정 내용:

- [x] `CompletionSignalController` 추가
  - 적재 중에는 빨간 불, `stack_complete` 감지 후에는 초록 불이 보이도록 `StackCompleteSignalRed` / `StackCompleteSignalGreen` visibility를 전환한다.
- [x] 장면에 완료 신호등 visual 추가
  - `StackCompleteSignalBase`, `StackCompleteSignalPost`, `StackCompleteSignalHousing`, red/green light로 구성한다.
  - 적재 완료 신호가 로그뿐 아니라 화면에서도 보이도록 했다.
- [x] `ARM_CLEAR_SETTLE_TIME = 1.8` 추가
  - 적재 완료 직후 AMR이 바로 들어오지 않고, 로봇팔이 go-home/clear 자세로 물러날 시간을 둔다.
- [x] `restore_demo_carried_active_bin()` 추가
  - 공식 `ReachToPlace`가 실행되는 도중 Cortex monitor가 `active_bin`을 `None`으로 비우는 경우가 있어, 실제 운반 중인 `demo_carried_bin`을 state 실행 전에 다시 `active_bin`으로 복원한다.
  - 이 보강 없이는 `AttributeError: 'NoneType' object has no attribute 'bin_obj'`가 발생할 수 있었다.
- [x] pick/return timing 현실감 보강
  - `REACH_PICK_MAX_DURATION = 5.8`
  - `REACH_PLACE_MAX_DURATION = 3.6`
  - `RETURN_READY_DURATION = 2.2`
  - 후반 박스에서도 팔이 pick 위치에 실제로 가까워진 뒤 scripted attach가 들어가도록 조정했다.
- [x] `--self-test-max-pre-grip-offset` 옵션 추가
  - self-test에서 pre-grip 보정량이 지정 임계값을 넘으면 실패하도록 했다.
  - PowerShell wrapper에도 `-SelfTestMaxPreGripOffset` 옵션을 추가했다.

검증 명령:

```powershell
cd E:\Harim_AMR
.\.conda\env_isaacsim_5_1_0\python.exe -m unittest isaac_sim.tests.test_harim_transfer_orchestrator
.\.conda\env_isaacsim_5_1_0\python.exe -m py_compile isaac_sim/scripts/run_harim_pallet_demo.py isaac_sim/tests/test_harim_transfer_orchestrator.py
powershell -ExecutionPolicy Bypass -File .\run_harim_demo.ps1 -Headless -AcceptEula -SelfTestFrames 8000 -SelfTestMinPlacedBins 8 -SelfTestMinTransferCycles 1 -SelfTestMaxPreGripOffset 0.05 -SelfTestDebugBins -Cycles 1
```

확인 결과:

- [x] unittest 28개 통과
- [x] Python compile 통과
- [x] 8000-frame end-to-end self-test 통과
  - `stack-count 8/8 after bin_7`
  - `stack_complete detected`
  - `attached 8 stacked items and 12 pallet parts`
  - `slide-released pallet assembly at drop pose`
  - `completed transfer cycle 1`
  - `self-test completed after 8000 frames; placed_bins=8; transfer_cycles=1; max_pre_grip_offset=0.0049`

---

## 2026-05-29 return-ready 도달 판정 및 실패 exit 보강 메모

pre-grip gate를 실제로 엄격하게 걸어 보니, 후반 박스에서 `return_ready`가 단순 시간 만료로 끝난 뒤 다음 pick이 시작되는 경우가 있었다. 이 경우 팔이 pick-ready 위치로 충분히 돌아오기 전에 `ReachToPick`이 시작되어 pre-grip 보정량이 수십 cm 이상으로 커질 수 있다. GUI에서는 박스가 그리퍼 쪽으로 순간 보정되는 것처럼 보일 수 있으므로, return-ready를 실제 end-effector 위치 오차 기준으로 종료하도록 바꿨다.

수정 내용:

- [x] `RETURN_READY_POSITION_THRESHOLD = 0.04` 추가
  - `return_ready`는 end-effector 위치와 목표 위치의 오차가 4 cm 이하가 되어야 정상 완료된다.
- [x] `RETURN_READY_DURATION = 5.0`으로 확장
  - 단순히 빠르게 다음 박스로 넘어가기보다 팔이 실제로 pick-ready 근처에 도달하는 것을 우선한다.
- [x] `DemoTimedArmMoveTo`가 `position_error`를 계산하도록 수정
  - 도달 시 `return_ready reached; error=... m` 로그를 남긴다.
  - 최대 시간 초과 시에는 `timed release; error=... m`로 남겨 후속 튜닝 근거를 제공한다.
- [x] self-test 실패 exit 보강
  - Isaac/Kit 종료 경로가 `SystemExit(1)`을 0으로 흡수하는 경우가 있어, self-test 실패와 simulation exception에서는 `os._exit(1)`을 사용한다.
  - 실패 로그가 있으면 외부 PowerShell `$LASTEXITCODE`도 1이 되도록 확인했다.

검증 명령:

```powershell
cd E:\Harim_AMR
.\.conda\env_isaacsim_5_1_0\python.exe -m unittest isaac_sim.tests.test_harim_transfer_orchestrator
.\.conda\env_isaacsim_5_1_0\python.exe -m py_compile isaac_sim/scripts/run_harim_pallet_demo.py isaac_sim/tests/test_harim_transfer_orchestrator.py
powershell -ExecutionPolicy Bypass -File .\run_harim_demo.ps1 -Headless -AcceptEula -SelfTestFrames 10 -SelfTestMinPlacedBins 1 -Cycles 1
powershell -ExecutionPolicy Bypass -File .\run_harim_demo.ps1 -Headless -AcceptEula -SelfTestFrames 12000 -SelfTestMinPlacedBins 8 -SelfTestMinTransferCycles 1 -SelfTestMaxPreGripOffset 0.05 -SelfTestDebugBins -Cycles 1
```

확인 결과:

- [x] unittest 28개 통과
- [x] Python compile 통과
- [x] 실패 probe에서 외부 `$LASTEXITCODE=1` 확인
- [x] 12000-frame end-to-end self-test 통과
  - 모든 `return_ready`가 `reached`로 종료
  - `stack-count 8/8 after bin_7`
  - `attached 8 stacked items and 12 pallet parts`
  - `slide-released pallet assembly at drop pose`
  - `completed transfer cycle 1`
  - `self-test completed after 12000 frames; placed_bins=8; transfer_cycles=1; max_pre_grip_offset=0.0050`

---

## 2026-05-29 return-ready 오차 self-test 게이트 추가 메모

이전 보강으로 `return_ready`가 실제 위치 오차 기준으로 종료되도록 만들었지만, 그 값은 로그로만 확인했다. 앞으로 다시 시간 만료나 모션 지연 때문에 다음 pick이 빨리 시작되는 회귀를 잡기 위해, return-ready 완료 시점의 최대 위치 오차를 self-test gate로 승격했다.

수정 내용:

- [x] `--self-test-max-return-ready-error` 옵션 추가
  - `return_ready`가 종료된 순간의 end-effector 위치 오차가 지정 임계값을 넘으면 self-test를 실패시킨다.
- [x] PowerShell wrapper에 `-SelfTestMaxReturnReadyError` 옵션 추가
- [x] `DemoTimedArmMoveTo`는 이동 중간 오차가 아니라 `reached` 또는 `timed release`로 상태가 끝나는 시점의 최종 오차만 `demo_max_return_ready_error`에 기록한다.
- [x] self-test 완료 로그에 `max_return_ready_error`를 추가했다.

검증 명령:

```powershell
cd E:\Harim_AMR
.\.conda\env_isaacsim_5_1_0\python.exe -m unittest isaac_sim.tests.test_harim_transfer_orchestrator
.\.conda\env_isaacsim_5_1_0\python.exe -m py_compile isaac_sim/scripts/run_harim_pallet_demo.py isaac_sim/tests/test_harim_transfer_orchestrator.py
powershell -ExecutionPolicy Bypass -File .\run_harim_demo.ps1 -Headless -AcceptEula -SelfTestFrames 3000 -SelfTestMinPlacedBins 1 -SelfTestMaxReturnReadyError 0.001 -Cycles 1
powershell -ExecutionPolicy Bypass -File .\run_harim_demo.ps1 -Headless -AcceptEula -SelfTestFrames 12000 -SelfTestMinPlacedBins 8 -SelfTestMinTransferCycles 1 -SelfTestMaxPreGripOffset 0.05 -SelfTestMaxReturnReadyError 0.05 -SelfTestDebugBins -Cycles 1
```

확인 결과:

- [x] unittest 28개 통과
- [x] Python compile 통과
- [x] 실패 probe에서 `OUTER_LASTEXIT=1` 확인
  - `max return-ready error 0.0395 m exceeded 0.0010 m`
- [x] 12000-frame end-to-end self-test 통과
  - 모든 `return_ready`가 `reached`로 종료
  - `stack-count 8/8 after bin_7`
  - `completed transfer cycle 1`
  - `self-test completed after 12000 frames; placed_bins=8; transfer_cycles=1; max_pre_grip_offset=0.0050; max_return_ready_error=0.0398`

---

## 2026-05-29 GUI release hold 보강 메모

GUI에서 확인했을 때 로봇팔이 박스를 놓지 않고 같이 들어 올리는 것처럼 보이는 문제가 있었다. headless 로그는 `<open gripper>`와 `demo-placed`가 정상 출력됐지만, release 이후 arm retreat/return 구간에서 박스 transform을 계속 고정하지 않으면 GUI 프레임에서 stale surface-gripper 상태나 Cortex active 상태가 박스를 다시 끌고 가는 것처럼 보일 수 있다.

수정 내용:

- [x] release 대상 bin에 `demo_release_target_p`, `demo_release_target_q` 저장
  - `mark_demo_bin_released()`가 `demo_attached`, `demo_attach_T`, `is_attached`를 즉시 해제한다.
  - 완료 마킹 전까지 `context.demo_released_bin`으로 release 대상을 보존한다.
- [x] `hold_demo_released_bin_at_target()` 추가
  - release된 bin을 목표 stack 좌표에 반복 고정한다.
  - `active_bin`이 release된 bin으로 다시 잡히면 즉시 비운다.
  - linear/angular velocity를 0으로 만들고 kinematic 상태를 유지한다.
- [x] post-release lift, return-ready, decider loop, frame step 전후에서 release hold 호출
  - 팔이 위로 빠지는 동안에도 박스가 팔레트 위 좌표에 남도록 한다.
  - GUI 렌더 타이밍에서 박스가 그리퍼를 따라 올라가는 것처럼 보이는 경로를 줄인다.
- [x] release drift self-test gate 추가
  - Python 옵션: `--self-test-max-release-drift`
  - PowerShell 옵션: `-SelfTestMaxReleaseDrift`
  - 완료 로그에 `max_release_drift`를 출력한다.

검증 명령:

```powershell
cd E:\Harim_AMR
.\.conda\env_isaacsim_5_1_0\python.exe -m py_compile isaac_sim\scripts\run_harim_pallet_demo.py
.\.conda\env_isaacsim_5_1_0\python.exe -m unittest isaac_sim.tests.test_harim_transfer_orchestrator
powershell -ExecutionPolicy Bypass -File .\run_harim_demo.ps1 -Headless -AcceptEula -SelfTestFrames 5200 -SelfTestMinPlacedBins 4 -SelfTestMaxPreGripOffset 0.05 -SelfTestMaxReturnReadyError 0.05 -SelfTestMaxReleaseDrift 0.02 -SelfTestDebugBins -Cycles 1
powershell -ExecutionPolicy Bypass -File .\run_harim_demo.ps1 -Headless -AcceptEula -SelfTestFrames 12000 -SelfTestMinPlacedBins 8 -SelfTestMinTransferCycles 1 -SelfTestMaxPreGripOffset 0.05 -SelfTestMaxReturnReadyError 0.05 -SelfTestMaxReleaseDrift 0.005 -SelfTestDebugBins -Cycles 1
```

확인 결과:

- [x] unittest 28개 통과
- [x] Python compile 통과
- [x] 5200-frame release hold gate 통과
  - `placed_bins=8`
  - `max_release_drift=0.0000`
- [x] 12000-frame full end-to-end self-test 통과
  - `placed_bins=8`
  - `transfer_cycles=1`
  - `max_pre_grip_offset=0.0050`
  - `max_return_ready_error=0.0395`
  - `max_release_drift=0.0000`

---

## 2026-05-29 AMR 리프트/하역 현실성 검증 보강 메모

이전 self-test는 AMR state가 `LIFT_UP`, `MOVE_TO_DROP`, `DETACH`, `SLIDE_OUT_FROM_PALLET`까지 진행되는지는 확인했지만, 실제 적재 payload가 얼마나 들어 올려졌는지와 하역 후 팔레트 assembly가 AMR 이탈 중 제자리에 남는지는 별도 수치로 검증하지 않았다. 현실감 기준에서는 이 두 가지가 핵심이므로 self-test 지표를 추가했다.

수정 내용:

- [x] payload lift 측정 추가
  - `stack_complete` 시점에 각 stack item의 기준 pose를 저장한다.
  - `LIFT_UP` 동안 각 item의 실제 Z 상승량을 계산해 `max_payload_lift_observed`에 기록한다.
  - full self-test에서 `--self-test-min-payload-lift`로 최소 상승량을 요구할 수 있다.
- [x] dropped payload drift 측정 추가
  - `DETACH` 시점의 박스/팔레트 pose를 기록한다.
  - `SLIDE_OUT_FROM_PALLET` 동안 현재 pose와 drop pose의 거리 차이를 `max_dropped_payload_drift`에 기록한다.
  - full self-test에서 `--self-test-max-dropped-payload-drift`로 하역 후 밀림 허용치를 제한할 수 있다.
- [x] PowerShell wrapper 옵션 추가
  - `-SelfTestMinPayloadLift`
  - `-SelfTestMaxDroppedPayloadDrift`
- [x] 후반 pick 안정성 보강
  - `REACH_PICK_MAX_DURATION = 12.0`
  - `RETURN_READY_DURATION = 10.0`
  - 후반부 박스에서 reach가 시간 초과되며 pre-grip offset이 커지는 run이 있어, 팔이 충분히 pick-ready/grasp 근처로 수렴할 시간을 더 준다.

검증 명령:

```powershell
cd E:\Harim_AMR
.\.conda\env_isaacsim_5_1_0\python.exe -m py_compile isaac_sim\scripts\run_harim_pallet_demo.py
.\.conda\env_isaacsim_5_1_0\python.exe -m unittest isaac_sim.tests.test_harim_transfer_orchestrator
powershell -ExecutionPolicy Bypass -File .\run_harim_demo.ps1 -Headless -AcceptEula -SelfTestFrames 12000 -SelfTestMinPlacedBins 8 -SelfTestMinTransferCycles 1 -SelfTestMaxPreGripOffset 0.05 -SelfTestMaxReturnReadyError 0.05 -SelfTestMaxReleaseDrift 0.005 -SelfTestMinPayloadLift 0.10 -SelfTestMaxDroppedPayloadDrift 0.005 -SelfTestDebugBins -Cycles 1
```

확인 결과:

- [x] unittest 29개 통과
- [x] Python compile 통과
- [x] 12000-frame full end-to-end self-test 통과
  - `placed_bins=8`
  - `transfer_cycles=1`
  - `max_pre_grip_offset=0.0048`
  - `max_return_ready_error=0.0392`
  - `max_release_drift=0.0000`
  - `max_payload_lift=0.1100`
  - `max_dropped_payload_drift=0.0000`

---

## 2026-05-29 리프트-팔레트 접촉 간격 및 터널 여유폭 gate 추가

GUI에서 AMR 리프트부가 팔레트와 떨어져 보이면 팔레트가 접촉 없이 공중에서 따라 올라가는 장면처럼 보일 수 있다. 이번 보강에서는 iw_hub 상단 리프트 플레이트를 팔레트 상판 하부 바로 아래로 올리고, 그 간격을 self-test에서 직접 검증하도록 했다.

수정 내용:

- [x] 팔레트 주요 치수를 상수화
  - `PALLET_DECK_SCALE`
  - `PALLET_RUNNER_SCALE`
  - `PALLET_BLOCK_SCALE`
  - `PALLET_GROOVE_SCALE`
  - `LIFT_PLATE_SCALE`
- [x] 리프트 플레이트 높이 재계산
  - 기존 `AMR_LIFT_PLATE_OFFSET_Z = 0.48`은 팔레트 deck underside보다 약 5 cm 낮아 보일 수 있었다.
  - 새 값은 `PALLET_DECK_UNDERSIDE_Z`, `LIFT_PLATE_SCALE`, `LIFT_TO_PALLET_CONTACT_GAP = 0.005`로 계산한다.
  - 결과적으로 리프트 상면과 팔레트 상판 하부 사이 간격은 5 mm로 유지된다.
- [x] 리프트 플레이트를 항상 visible 처리
  - 실제 iw_hub asset lift prim이 있어도 얇은 접촉 플레이트를 보이게 해서 GUI에서 팔레트를 받치는 면이 명확하게 보이도록 했다.
- [x] self-test geometry gate 추가
  - Python 옵션: `--self-test-max-lift-contact-gap`
  - Python 옵션: `--self-test-min-pallet-tunnel-clearance`
  - PowerShell 옵션: `-SelfTestMaxLiftContactGap`
  - PowerShell 옵션: `-SelfTestMinPalletTunnelClearance`
- [x] 완료 로그에 지표 추가
  - `max_lift_contact_gap`
  - `min_lift_contact_gap`
  - `pallet_tunnel_clearance`

검증 명령:

```powershell
cd E:\Harim_AMR
.\.conda\env_isaacsim_5_1_0\python.exe -m py_compile isaac_sim\scripts\run_harim_pallet_demo.py
.\.conda\env_isaacsim_5_1_0\python.exe -m unittest isaac_sim.tests.test_harim_transfer_orchestrator
powershell -ExecutionPolicy Bypass -File .\run_harim_demo.ps1 -Headless -AcceptEula -SelfTestFrames 12000 -SelfTestMinPlacedBins 8 -SelfTestMinTransferCycles 1 -SelfTestMaxPreGripOffset 0.05 -SelfTestMaxReturnReadyError 0.05 -SelfTestMaxReleaseDrift 0.005 -SelfTestRequireGripperOpenAfterRelease -SelfTestMaxStackLateralGap 0.03 -SelfTestMaxStackSupportGap 0.02 -SelfTestMinPayloadLift 0.10 -SelfTestMaxDroppedPayloadDrift 0.005 -SelfTestMaxLiftContactGap 0.01 -SelfTestMinPalletTunnelClearance 0.04 -SelfTestDebugBins -Cycles 1
```

확인 결과:

- [x] Python compile 통과
- [x] unittest 35개 통과
- [x] 12000-frame full end-to-end self-test 통과
  - 로그 파일: `isaacsim_logs/harim_lift_contact_gate_full_e2e_12000.log`
  - `placed_bins=8`
  - `transfer_cycles=1`
  - `max_pre_grip_offset=0.0046`
  - `max_return_ready_error=0.0399`
  - `max_release_drift=0.0000`
  - `release_gripper_not_open=0`
  - `release_gripped_object_max=0`
  - `max_stack_lateral_gap=0.0200`
  - `max_stack_support_gap=0.0100`
  - `max_payload_lift=0.1100`
  - `max_dropped_payload_drift=0.0000`
  - `max_lift_contact_gap=0.0050`
  - `min_lift_contact_gap=0.0050`
  - `pallet_tunnel_clearance=0.0600`

---

## 2026-05-29 AMR 리프트를 두 줄 fork/rail 형상으로 변경

리프트 접촉 간격은 맞췄지만, 단일 넓은 판은 실제 팔레트 AMR보다 임시 지지판처럼 보일 수 있었다. 이번 보강에서는 GUI에서 보이는 리프트 지지부를 좌우 두 줄 fork/rail 형상으로 바꿔 팔레트 터널 안으로 들어가는 구조가 더 명확하게 보이도록 했다.

수정 내용:

- [x] 단일 `IwHubLiftPlate` visual 제거
- [x] `IwHubLiftFork_0`, `IwHubLiftFork_1` 두 개의 얇은 fork rail 생성
  - 각 fork scale: `LIFT_FORK_SCALE = [1.10, 0.12, 0.035]`
  - fork Y offset: `-0.24`, `+0.24`
  - fork 사이 내부 간격: 0.36 m
- [x] orchestrator가 두 fork visual을 AMR pose와 lift offset에 맞춰 함께 이동하도록 수정
- [x] 팔레트 터널 clearance 계산을 단일 판 half-width가 아니라 두 fork의 outer half-width 기준으로 변경
- [x] self-test gate 추가
  - Python 옵션: `--self-test-min-lift-fork-inner-gap`
  - PowerShell 옵션: `-SelfTestMinLiftForkInnerGap`
- [x] 완료 로그에 `lift_fork_inner_gap` 추가

검증 명령:

```powershell
cd E:\Harim_AMR
.\.conda\env_isaacsim_5_1_0\python.exe -m py_compile isaac_sim\scripts\run_harim_pallet_demo.py
.\.conda\env_isaacsim_5_1_0\python.exe -m unittest isaac_sim.tests.test_harim_transfer_orchestrator
powershell -ExecutionPolicy Bypass -File .\run_harim_demo.ps1 -Headless -AcceptEula -SelfTestFrames 12000 -SelfTestMinPlacedBins 8 -SelfTestMinTransferCycles 1 -SelfTestMaxPreGripOffset 0.05 -SelfTestMaxReturnReadyError 0.05 -SelfTestMaxReleaseDrift 0.005 -SelfTestRequireGripperOpenAfterRelease -SelfTestMaxStackLateralGap 0.03 -SelfTestMaxStackSupportGap 0.02 -SelfTestMinPayloadLift 0.10 -SelfTestMaxDroppedPayloadDrift 0.005 -SelfTestMaxLiftContactGap 0.01 -SelfTestMinPalletTunnelClearance 0.10 -SelfTestMinLiftForkInnerGap 0.30 -SelfTestDebugBins -Cycles 1
```

확인 결과:

- [x] Python compile 통과
- [x] unittest 37개 통과
- [x] 12000-frame full end-to-end self-test 통과
  - 로그 파일: `isaacsim_logs/harim_lift_fork_gate_full_e2e_12000.log`
  - `placed_bins=8`
  - `transfer_cycles=1`
  - `max_pre_grip_offset=0.0046`
  - `max_return_ready_error=0.0398`
  - `max_release_drift=0.0000`
  - `release_gripper_not_open=0`
  - `release_gripped_object_max=0`
  - `max_stack_lateral_gap=0.0200`
  - `max_stack_support_gap=0.0100`
  - `max_payload_lift=0.1100`
  - `max_dropped_payload_drift=0.0000`
  - `max_lift_contact_gap=0.0050`
  - `min_lift_contact_gap=0.0050`
  - `pallet_tunnel_clearance=0.1200`
  - `lift_fork_inner_gap=0.3600`

---

## 2026-05-29 drop slide workstation 지지 높이 및 터널 간격 gate 추가

하역 작업대가 존재하더라도 visible rail/roller가 팔레트 측면 runner와 겹치면 GUI에서 팔레트를 뚫고 지나가는 것처럼 보일 수 있다. 이번 보강에서는 drop slide workstation의 지지 lane을 AMR fork와 같은 중앙 터널 쪽으로 옮기고, 팔레트 상판 하부를 실제로 받는 높이/간격을 self-test gate로 묶었다.

수정 내용:

- [x] drop slide lane 위치를 팔레트 측면 runner 영역이 아니라 중앙 터널 안쪽으로 정렬
  - `DROP_SLIDE_LANE_Y_OFFSETS`를 `LIFT_FORK_OFFSETS`의 Y offset과 맞춤
  - lane Y offset: `-0.24`, `+0.24`
- [x] drop support 상면 높이를 팔레트 deck underside 기준으로 계산
  - `DROP_SLIDE_SUPPORT_GAP = 0.005`
  - 팔레트 상판 하부와 작업대 support 상면 간격이 5 mm가 되도록 맞춤
- [x] visible roller 치수를 좁혀 팔레트 side runner와 겹치지 않게 조정
  - `DROP_SLIDE_ROLLER_SCALE = [0.12, 0.16, 0.035]`
- [x] 숨겨진 collision support도 단일 넓은 slab이 아니라 lane별 support로 변경
  - `DropSlideTopSupport_0`
  - `DropSlideTopSupport_1`
- [x] self-test gate 추가
  - Python 옵션: `--self-test-max-drop-support-gap`
  - Python 옵션: `--self-test-min-drop-lane-clearance`
  - Python 옵션: `--self-test-min-drop-runner-clearance`
  - PowerShell 옵션: `-SelfTestMaxDropSupportGap`
  - PowerShell 옵션: `-SelfTestMinDropLaneClearance`
  - PowerShell 옵션: `-SelfTestMinDropRunnerClearance`
- [x] 완료 로그에 drop workstation 지표 추가
  - `drop_support_gap`
  - `drop_lane_clearance`
  - `drop_runner_clearance`

검증 명령:

```powershell
cd E:\Harim_AMR
.\.conda\env_isaacsim_5_1_0\python.exe -m py_compile isaac_sim\scripts\run_harim_pallet_demo.py
.\.conda\env_isaacsim_5_1_0\python.exe -m unittest isaac_sim.tests.test_harim_transfer_orchestrator
powershell -ExecutionPolicy Bypass -File .\run_harim_demo.ps1 -Headless -AcceptEula -SelfTestFrames 12000 -SelfTestMinPlacedBins 8 -SelfTestMinTransferCycles 1 -SelfTestMaxPreGripOffset 0.05 -SelfTestMaxReturnReadyError 0.05 -SelfTestMaxReleaseDrift 0.005 -SelfTestRequireGripperOpenAfterRelease -SelfTestMaxStackLateralGap 0.03 -SelfTestMaxStackSupportGap 0.02 -SelfTestMinPayloadLift 0.10 -SelfTestMaxDroppedPayloadDrift 0.005 -SelfTestMaxLiftContactGap 0.01 -SelfTestMinPalletTunnelClearance 0.10 -SelfTestMinLiftForkInnerGap 0.30 -SelfTestMaxDropSupportGap 0.01 -SelfTestMinDropLaneClearance 0.08 -SelfTestMinDropRunnerClearance 0.10 -SelfTestDebugBins -Cycles 1
```

확인 결과:

- [x] Python compile 통과
- [x] unittest 38개 통과
- [x] 12000-frame full end-to-end self-test 통과
  - 로그 파일: `isaacsim_logs/harim_drop_slide_gate_full_e2e_12000.log`
  - `placed_bins=8`
  - `transfer_cycles=1`
  - `max_pre_grip_offset=0.0046`
  - `max_return_ready_error=0.0399`
  - `max_release_drift=0.0000`
  - `release_gripper_not_open=0`
  - `release_gripped_object_max=0`
  - `max_stack_lateral_gap=0.0200`
  - `max_stack_support_gap=0.0100`
  - `max_payload_lift=0.1100`
  - `max_dropped_payload_drift=0.0000`
  - `max_lift_contact_gap=0.0050`
  - `pallet_tunnel_clearance=0.1200`
  - `lift_fork_inner_gap=0.3600`
  - `drop_support_gap=0.0050`
  - `drop_lane_clearance=0.1000`
  - `drop_runner_clearance=0.1700`

---

## 2026-05-29 drop slide lane과 AMR fork 간섭 제거

이전 보강에서 drop slide lane을 팔레트 중앙 터널로 옮겼지만, AMR fork와 같은 Y 위치를 사용하면 하역 순간에 작업대 lane과 AMR fork가 서로 겹칠 수 있었다. 이번 보강에서는 drop slide lane을 AMR fork보다 바깥쪽으로 옮기고, 두 구조물 사이 clearance를 self-test gate로 검증하도록 했다.

수정 내용:

- [x] 팔레트 터널 half width를 실제 runner/block 내부 여유에 맞게 `0.46 m`로 조정
- [x] drop slide lane 위치 조정
  - 기존: AMR fork와 같은 `Y = +/-0.24`
  - 변경: AMR fork보다 바깥쪽인 `Y = +/-0.38`
- [x] drop slide lane 폭 축소
  - `DROP_SLIDE_RAIL_SCALE = [1.80, 0.08, 0.09]`
  - `DROP_SLIDE_ROLLER_SCALE = [0.12, 0.08, 0.035]`
  - `DROP_SLIDE_TOP_SUPPORT_SCALE = [1.95, 0.08, 0.035]`
- [x] AMR fork와 drop lane 사이의 최소 간격 계산 추가
  - `compute_drop_workstation_fork_clearance()`
- [x] self-test gate 추가
  - Python 옵션: `--self-test-min-drop-fork-clearance`
  - PowerShell 옵션: `-SelfTestMinDropForkClearance`
- [x] 완료 로그에 `drop_fork_clearance` 추가

검증 명령:

```powershell
cd E:\Harim_AMR
.\.conda\env_isaacsim_5_1_0\python.exe -m py_compile isaac_sim\scripts\run_harim_pallet_demo.py
.\.conda\env_isaacsim_5_1_0\python.exe -m unittest isaac_sim.tests.test_harim_transfer_orchestrator
powershell -ExecutionPolicy Bypass -File .\run_harim_demo.ps1 -Headless -AcceptEula -SelfTestFrames 12000 -SelfTestMinPlacedBins 8 -SelfTestMinTransferCycles 1 -SelfTestMaxPreGripOffset 0.05 -SelfTestMaxReturnReadyError 0.05 -SelfTestMaxReleaseDrift 0.005 -SelfTestRequireGripperOpenAfterRelease -SelfTestMaxStackLateralGap 0.03 -SelfTestMaxStackSupportGap 0.02 -SelfTestMinPayloadLift 0.10 -SelfTestMaxDroppedPayloadDrift 0.005 -SelfTestMaxLiftContactGap 0.01 -SelfTestMinPalletTunnelClearance 0.10 -SelfTestMinLiftForkInnerGap 0.30 -SelfTestMaxDropSupportGap 0.01 -SelfTestMinDropLaneClearance 0.03 -SelfTestMinDropRunnerClearance 0.05 -SelfTestMinDropForkClearance 0.03 -SelfTestDebugBins -Cycles 1
```

확인 결과:

- [x] Python compile 통과
- [x] unittest 38개 통과
- [x] 12000-frame full end-to-end self-test 통과
  - 로그 파일: `isaacsim_logs/harim_drop_fork_clearance_gate_full_e2e_12000.log`
  - `placed_bins=8`
  - `transfer_cycles=1`
  - `max_pre_grip_offset=0.0046`
  - `max_return_ready_error=0.0399`
  - `max_release_drift=0.0000`
  - `max_stack_lateral_gap=0.0200`
  - `max_stack_support_gap=0.0100`
  - `max_payload_lift=0.1100`
  - `max_dropped_payload_drift=0.0000`
  - `max_lift_contact_gap=0.0050`
  - `pallet_tunnel_clearance=0.1600`
  - `lift_fork_inner_gap=0.3600`
  - `drop_support_gap=0.0050`
  - `drop_lane_clearance=0.0400`
  - `drop_runner_clearance=0.0700`
  - `drop_fork_clearance=0.0400`

---

## 2026-05-29 stack footprint / pallet deck margin gate 추가

박스 간 간격과 수직 지지는 검증했지만, 적재된 박스 전체 footprint가 팔레트 deck 안쪽에 충분히 들어오는지는 별도 gate가 없었다. 이번 보강에서는 carton 외곽과 pallet deck 외곽 사이의 최소 margin을 계산해, 박스가 팔레트 밖으로 튀어나오거나 가장자리에 너무 붙는 경우 self-test가 실패하도록 했다.

수정 내용:

- [x] `compute_stack_pallet_footprint_metrics()` 추가
  - 모든 stack coordinate의 carton body 외곽 X/Y 범위를 계산한다.
  - `pickup_x`, `pickup_y` 기준 pallet deck footprint와 비교한다.
  - 최소 여백 `min_stack_pallet_margin`을 계산한다.
  - 음수 margin은 overhang으로 보고 `max_stack_pallet_overhang`으로 기록한다.
- [x] self-test gate 추가
  - Python 옵션: `--self-test-min-stack-pallet-margin`
  - PowerShell 옵션: `-SelfTestMinStackPalletMargin`
- [x] 완료 로그에 stack footprint 지표 추가
  - `min_stack_pallet_margin`
  - `max_stack_pallet_overhang`

검증 명령:

```powershell
cd E:\Harim_AMR
.\.conda\env_isaacsim_5_1_0\python.exe -m py_compile isaac_sim\scripts\run_harim_pallet_demo.py
.\.conda\env_isaacsim_5_1_0\python.exe -m unittest isaac_sim.tests.test_harim_transfer_orchestrator
powershell -ExecutionPolicy Bypass -File .\run_harim_demo.ps1 -Headless -AcceptEula -SelfTestFrames 12000 -SelfTestMinPlacedBins 8 -SelfTestMinTransferCycles 1 -SelfTestMaxPreGripOffset 0.05 -SelfTestMaxReturnReadyError 0.05 -SelfTestMaxReleaseDrift 0.005 -SelfTestRequireGripperOpenAfterRelease -SelfTestMaxStackLateralGap 0.03 -SelfTestMaxStackSupportGap 0.02 -SelfTestMinStackPalletMargin 0.08 -SelfTestMinPayloadLift 0.10 -SelfTestMaxDroppedPayloadDrift 0.005 -SelfTestMaxLiftContactGap 0.01 -SelfTestMinPalletTunnelClearance 0.10 -SelfTestMinLiftForkInnerGap 0.30 -SelfTestMaxDropSupportGap 0.01 -SelfTestMinDropLaneClearance 0.03 -SelfTestMinDropRunnerClearance 0.05 -SelfTestMinDropForkClearance 0.03 -SelfTestDebugBins -Cycles 1
```

확인 결과:

- [x] Python compile 통과
- [x] unittest 39개 통과
- [x] 12000-frame full end-to-end self-test 통과
  - 로그 파일: `isaacsim_logs/harim_stack_pallet_margin_gate_full_e2e_12000.log`
  - `placed_bins=8`
  - `transfer_cycles=1`
  - `max_pre_grip_offset=0.0046`
  - `max_return_ready_error=0.0398`
  - `max_release_drift=0.0000`
  - `max_stack_lateral_gap=0.0200`
  - `max_stack_support_gap=0.0100`
  - `min_stack_pallet_margin=0.0850`
  - `max_stack_pallet_overhang=0.0000`
  - `max_payload_lift=0.1100`
  - `max_dropped_payload_drift=0.0000`
  - `max_lift_contact_gap=0.0050`
  - `pallet_tunnel_clearance=0.1600`
  - `drop_fork_clearance=0.0400`

---

## 2026-05-29 carton side label visual 추가

동작/간격 gate는 통과하고 있지만, GUI에서 박스가 단순한 갈색 cuboid와 top tape만으로 보이면 실제 물류 carton 느낌이 약하다. 이번 보강에서는 기존 KLT collision/grasp 구조는 그대로 두고 child visual overlay만 추가해 박스 양쪽 면에 라벨 패널과 빨간 방향 스트립을 표시한다.

수정 내용:

- [x] carton side label 치수/색상 상수 추가
  - `CARTON_SIDE_LABEL_SCALE = [0.140, 0.006, 0.055]`
  - `CARTON_SIDE_STRIPE_SCALE = [0.030, 0.007, 0.065]`
  - `CARTON_LABEL_COLOR`
- [x] `_add_carton_visual()` 보강
  - 기존 `HarimCartonBody`, `HarimCartonTopTape` 유지
  - `HarimCartonSideLabelFront`
  - `HarimCartonSideLabelBack`
  - `HarimCartonSideStripeFront`
  - `HarimCartonSideStripeBack`
- [x] carton visual dimension unittest 보강
  - label과 stripe가 carton body보다 얇고, side label처럼 보이는 치수인지 확인
- [x] 기존 strict wrapper로 전체 회귀 검증

검증 명령:

```powershell
cd E:\Harim_AMR
.\.conda\env_isaacsim_5_1_0\python.exe -m py_compile isaac_sim\scripts\run_harim_pallet_demo.py
.\.conda\env_isaacsim_5_1_0\python.exe -m unittest isaac_sim.tests.test_harim_transfer_orchestrator
powershell -ExecutionPolicy Bypass -File .\run_harim_strict_self_test.ps1 -AcceptEula -SelfTestDebugBins
```

확인 결과:

- [x] Python compile 통과
- [x] unittest 41개 통과
- [x] strict wrapper 기반 12000-frame full end-to-end self-test 통과
  - 로그 파일: `isaacsim_logs/harim_carton_label_strict_full_e2e_12000.log`
  - `placed_bins=8`
  - `transfer_cycles=1`
  - `max_pre_grip_offset=0.0046`
  - `max_return_ready_error=0.0399`
  - `max_release_drift=0.0000`
  - `min_stack_pallet_margin=0.0850`
  - `max_dropped_payload_drift=0.0000`
  - `amr_exit_clearance=0.6500`

---

## 2026-05-29 floor marking visual 추가

물체와 동작은 안정화됐지만, GUI에서 pickup/drop 위치와 AMR 이동 경로가 바닥에서 구분되지 않으면 설명 영상에서 공정 흐름이 약하게 보인다. 이번 보강에서는 물리 충돌 없는 얇은 `VisualCuboid` floor marking을 추가해 작업 구역과 AMR 동선을 명확하게 했다.

수정 내용:

- [x] floor marking 상수 추가
  - `FLOOR_MARKING_Z`
  - `FLOOR_MARKING_THICKNESS`
  - `AMR_PATH_MARKING_WIDTH`
  - `WORK_ZONE_MARKING_SIZE`
  - `WORK_ZONE_MARKING_EDGE_WIDTH`
  - pickup/drop/path marking color
- [x] `create_floor_markings()` 추가
  - `AmrPathCenterLine`: pickup에서 drop까지 이어지는 AMR 이동 차선
  - `PickupZone*`: 팔레타이징 완료 후 AMR이 진입하는 pickup 작업 구역 outline
  - `DropZone*`: 하역 작업대가 있는 drop 작업 구역 outline
- [x] 충돌체는 추가하지 않고 `VisualCuboid`만 사용
  - 기존 AMR, 팔레트, drop slide 물리/이송 gate에 영향이 없도록 했다.
- [x] floor marking dimension unittest 추가
  - marking이 팔레트보다 넓은 작업 구역을 표시하는지 확인
  - marking thickness가 1 cm 이하인지 확인
  - pickup/drop 색상이 서로 구분되는지 확인

검증 명령:

```powershell
cd E:\Harim_AMR
.\.conda\env_isaacsim_5_1_0\python.exe -m py_compile isaac_sim\scripts\run_harim_pallet_demo.py
.\.conda\env_isaacsim_5_1_0\python.exe -m unittest isaac_sim.tests.test_harim_transfer_orchestrator
powershell -ExecutionPolicy Bypass -File .\run_harim_strict_self_test.ps1 -AcceptEula -SelfTestDebugBins
```

확인 결과:

- [x] Python compile 통과
- [x] unittest 42개 통과
- [x] strict wrapper 기반 12000-frame full end-to-end self-test 통과
  - 로그 파일: `isaacsim_logs/harim_floor_markings_strict_full_e2e_12000.log`
  - `placed_bins=8`
  - `transfer_cycles=1`
  - `max_pre_grip_offset=0.0046`
  - `max_return_ready_error=0.0398`
  - `max_release_drift=0.0000`
  - `min_stack_pallet_margin=0.0850`
  - `max_dropped_payload_drift=0.0000`
  - `amr_exit_clearance=0.6500`

---

## 2026-05-29 strict full realism self-test wrapper 추가

현실성 gate가 많아지면서 매번 긴 명령을 직접 입력하면 특정 gate를 빼먹기 쉽다. 이번 보강에서는 지금까지 추가한 모든 full end-to-end realism gate를 한 번에 실행하는 wrapper를 추가했다.

수정 내용:

- [x] `run_harim_strict_self_test.ps1` 추가
  - 기본 `SelfTestFrames = 12000`
  - 기본 `Cycles = 1`
  - 기본은 headless 실행
  - `-ShowGui`를 주면 GUI 실행 가능
  - `-AcceptEula`, `-SelfTestDebugBins` 전달 지원
- [x] 현재 full realism gate를 모두 포함
  - 8박스 적재
  - AMR transfer 1회 완료
  - pre-grip offset
  - return-ready error
  - release drift
  - release gripper open state
  - stack lateral/support gap
  - stack pallet margin
  - payload lift/drop drift
  - AMR exit clearance
  - lift contact/tunnel/fork geometry
  - drop support/lane/runner/fork clearance
- [x] PowerShell script 호출은 array splatting이 아니라 hashtable splatting으로 처리
  - script parameter에 named argument가 정확히 전달되도록 수정했다.
- [x] strict wrapper가 모든 gate를 포함하는지 unittest 추가

검증 명령:

```powershell
cd E:\Harim_AMR
.\.conda\env_isaacsim_5_1_0\python.exe -m py_compile isaac_sim\scripts\run_harim_pallet_demo.py
.\.conda\env_isaacsim_5_1_0\python.exe -m unittest isaac_sim.tests.test_harim_transfer_orchestrator
powershell -ExecutionPolicy Bypass -File .\run_harim_strict_self_test.ps1 -AcceptEula -SelfTestDebugBins
```

확인 결과:

- [x] Python compile 통과
- [x] unittest 41개 통과
- [x] strict wrapper 기반 12000-frame full end-to-end self-test 통과
  - 로그 파일: `isaacsim_logs/harim_strict_wrapper_full_e2e_12000.log`
  - `placed_bins=8`
  - `transfer_cycles=1`
  - `max_pre_grip_offset=0.0046`
  - `max_return_ready_error=0.0400`
  - `max_release_drift=0.0000`
  - `release_gripper_not_open=0`
  - `release_gripped_object_max=0`
  - `max_stack_lateral_gap=0.0200`
  - `max_stack_support_gap=0.0100`
  - `min_stack_pallet_margin=0.0850`
  - `max_stack_pallet_overhang=0.0000`
  - `max_payload_lift=0.1100`
  - `max_dropped_payload_drift=0.0000`
  - `amr_exit_clearance=0.6500`
  - `max_lift_contact_gap=0.0050`
  - `pallet_tunnel_clearance=0.1600`
  - `drop_fork_clearance=0.0400`

---

## 2026-05-29 AMR slide-out exit clearance gate 추가

하역 후 payload drift는 0으로 검증하고 있었지만, AMR fork가 dropped pallet footprint 밖으로 충분히 빠져나갔는지는 별도 지표가 없었다. 이번 보강에서는 slide-out 완료 후 AMR lift fork의 뒤쪽 끝과 dropped pallet deck 앞쪽 끝 사이의 X 방향 여유를 계산해 self-test gate로 확인한다.

수정 내용:

- [x] `compute_amr_exit_clearance()` 추가
  - AMR fork rear X = `amr_x - LIFT_FORK_SCALE[0] / 2`
  - dropped pallet front X = `drop_x + PALLET_DECK_SCALE[0] / 2`
  - 두 값의 차이를 `amr_exit_clearance`로 기록한다.
- [x] self-test gate 추가
  - Python 옵션: `--self-test-min-amr-exit-clearance`
  - PowerShell 옵션: `-SelfTestMinAmrExitClearance`
- [x] 완료 로그에 `amr_exit_clearance` 추가
- [x] transfer cycle이 완료되지 않았는데 exit clearance gate를 요구하면 실패하도록 처리

검증 명령:

```powershell
cd E:\Harim_AMR
.\.conda\env_isaacsim_5_1_0\python.exe -m py_compile isaac_sim\scripts\run_harim_pallet_demo.py
.\.conda\env_isaacsim_5_1_0\python.exe -m unittest isaac_sim.tests.test_harim_transfer_orchestrator
powershell -ExecutionPolicy Bypass -File .\run_harim_demo.ps1 -Headless -AcceptEula -SelfTestFrames 12000 -SelfTestMinPlacedBins 8 -SelfTestMinTransferCycles 1 -SelfTestMaxPreGripOffset 0.05 -SelfTestMaxReturnReadyError 0.05 -SelfTestMaxReleaseDrift 0.005 -SelfTestRequireGripperOpenAfterRelease -SelfTestMaxStackLateralGap 0.03 -SelfTestMaxStackSupportGap 0.02 -SelfTestMinStackPalletMargin 0.08 -SelfTestMinPayloadLift 0.10 -SelfTestMaxDroppedPayloadDrift 0.005 -SelfTestMinAmrExitClearance 0.60 -SelfTestMaxLiftContactGap 0.01 -SelfTestMinPalletTunnelClearance 0.10 -SelfTestMinLiftForkInnerGap 0.30 -SelfTestMaxDropSupportGap 0.01 -SelfTestMinDropLaneClearance 0.03 -SelfTestMinDropRunnerClearance 0.05 -SelfTestMinDropForkClearance 0.03 -SelfTestDebugBins -Cycles 1
```

확인 결과:

- [x] Python compile 통과
- [x] unittest 40개 통과
- [x] 12000-frame full end-to-end self-test 통과
  - 로그 파일: `isaacsim_logs/harim_amr_exit_clearance_gate_full_e2e_12000.log`
  - `placed_bins=8`
  - `transfer_cycles=1`
  - `max_pre_grip_offset=0.0046`
  - `max_return_ready_error=0.0400`
  - `max_release_drift=0.0000`
  - `max_stack_lateral_gap=0.0200`
  - `max_stack_support_gap=0.0100`
  - `min_stack_pallet_margin=0.0850`
  - `max_payload_lift=0.1100`
  - `max_dropped_payload_drift=0.0000`
  - `amr_exit_clearance=0.6500`
  - `max_lift_contact_gap=0.0050`
  - `pallet_tunnel_clearance=0.1600`
  - `drop_fork_clearance=0.0400`

---

## 2026-05-29 GUI release 강제 해제 추가 보강

GUI 확인에서 로봇팔이 박스를 놓지 않는 것처럼 보이는 증상이 다시 관찰되어, release 순간에 scripted attach 상태와 실제 surface gripper 상태를 더 강하게 분리했다. 핵심은 “시각적으로 박스가 그리퍼를 따라가는 경로”를 줄이는 것이다.

수정 내용:

- [x] `force_open_suction_gripper()` 추가
  - 기존 high-level `suction_gripper.open()` 호출에 더해 internal surface gripper interface의 `open_gripper()`도 직접 호출한다.
  - GUI에서 surface gripper 상태가 한 프레임 늦게 남아 박스를 붙잡아 보이는 경우를 줄인다.
- [x] `DemoReleaseBin.enter()`에서 release 대상 bin의 attach 상태를 먼저 끊음
  - `demo_attached = False`
  - `demo_attach_T = None`
  - `is_attached = False`
  - 그 뒤 kinematic 전환, velocity reset, stack 목표 pose 고정을 수행한다.
- [x] `DemoReleaseBin.step()` 동안 `force_open_suction_gripper()`를 반복 호출
  - release state가 유지되는 0.35초 동안 gripper open과 목표 pose hold가 계속 적용된다.
- [x] 불안정했던 `return_ready` 최소 유지 실험은 제거
  - `return_ready` 도달 후 추가로 잡아두면 position-only 명령 상태에서 오히려 후반 pick 자세가 흐트러지는 run이 있었다.
  - 기존 안정 timing으로 되돌리고 release 강제 해제만 남겼다.
- [x] AMR lift-up/lift-down은 `smoothstep` easing과 settle 시간을 유지
  - 팔레트가 순간이동하듯 들리는 느낌을 줄인다.
  - `AMR_LIFT_DURATION = 1.25`, `AMR_LIFT_SETTLE_TIME = 0.25`

검증 명령:

```powershell
cd E:\Harim_AMR
.\.conda\env_isaacsim_5_1_0\python.exe -m py_compile isaac_sim\scripts\run_harim_pallet_demo.py
.\.conda\env_isaacsim_5_1_0\python.exe -m unittest isaac_sim.tests.test_harim_transfer_orchestrator
powershell -ExecutionPolicy Bypass -File .\run_harim_demo.ps1 -Headless -AcceptEula -SelfTestFrames 12000 -SelfTestMinPlacedBins 8 -SelfTestMinTransferCycles 1 -SelfTestMaxPreGripOffset 0.05 -SelfTestMaxReturnReadyError 0.05 -SelfTestMaxReleaseDrift 0.005 -SelfTestMinPayloadLift 0.10 -SelfTestMaxDroppedPayloadDrift 0.005 -SelfTestDebugBins -Cycles 1
```

확인 결과:

- [x] unittest 31개 통과
- [x] Python compile 통과
- [x] 12000-frame full end-to-end self-test 통과
  - `placed_bins=8`
  - `transfer_cycles=1`
  - `max_pre_grip_offset=0.0049`
  - `max_return_ready_error=0.0396`
  - `max_release_drift=0.0000`
  - `max_payload_lift=0.1100`
  - `max_dropped_payload_drift=0.0000`

---

## 2026-05-29 release gripper 상태 gate와 관절 settle 보강

release drift만으로는 GUI에서 “박스가 아직 그리퍼에 붙어 보이는지”를 직접 검증하기 어렵다. 이번 보강에서는 release 순간의 surface gripper 상태를 self-test gate로 추가하고, 후반부 pick 안정성을 위해 release 후 짧은 관절 settle 동작을 넣었다.

수정 내용:

- [x] release gripper 상태 계측 추가
  - 옵션: `--self-test-require-gripper-open-after-release`
  - PowerShell 옵션: `-SelfTestRequireGripperOpenAfterRelease`
  - release 순간 gripper open 여부, gripped object 개수, probe 실패 횟수를 기록한다.
  - 완료 로그에 `release_gripper_samples`, `release_gripper_not_open`, `release_gripped_object_max`, `release_gripper_probe_failures`를 출력한다.
- [x] `DemoTimedArmJointSettle` 추가
  - release 후 arm이 다음 pick으로 바로 넘어가기 전에 `POST_RELEASE_JOINT_SETTLE_DURATION = 0.65`초 동안 기본 관절 자세 쪽으로 부드럽게 보간한다.
  - 후반부 박스에서 place 자세가 누적되어 `return_ready`/`reach_pick`이 크게 실패하는 run을 줄인다.
  - 완료 로그에 `joint_settle_count`를 출력한다.
- [x] probe 범위 조정
  - gripper interface를 release hold 전체 loop에서 계속 조회하면 arm control 안정성에 영향을 줄 수 있어, release 순간 1회만 확인한다.

검증 명령:

```powershell
cd E:\Harim_AMR
.\.conda\env_isaacsim_5_1_0\python.exe -m py_compile isaac_sim\scripts\run_harim_pallet_demo.py
.\.conda\env_isaacsim_5_1_0\python.exe -m unittest isaac_sim.tests.test_harim_transfer_orchestrator
powershell -ExecutionPolicy Bypass -File .\run_harim_demo.ps1 -Headless -AcceptEula -SelfTestFrames 5600 -SelfTestMinPlacedBins 4 -SelfTestMaxPreGripOffset 0.05 -SelfTestMaxReturnReadyError 0.05 -SelfTestMaxReleaseDrift 0.005 -SelfTestRequireGripperOpenAfterRelease -SelfTestDebugBins -Cycles 1
powershell -ExecutionPolicy Bypass -File .\run_harim_demo.ps1 -Headless -AcceptEula -SelfTestFrames 12000 -SelfTestMinPlacedBins 8 -SelfTestMinTransferCycles 1 -SelfTestMaxPreGripOffset 0.05 -SelfTestMaxReturnReadyError 0.05 -SelfTestMaxReleaseDrift 0.005 -SelfTestRequireGripperOpenAfterRelease -SelfTestMinPayloadLift 0.10 -SelfTestMaxDroppedPayloadDrift 0.005 -SelfTestDebugBins -Cycles 1
```

확인 결과:

- [x] unittest 32개 통과
- [x] Python compile 통과
- [x] 5600-frame release/gripper gate 통과
  - `placed_bins=8`
  - `max_pre_grip_offset=0.0046`
  - `release_gripper_samples=8`
  - `release_gripper_not_open=0`
  - `release_gripped_object_max=0`
  - `joint_settle_count=8`
- [x] 12000-frame full end-to-end self-test 통과
  - `placed_bins=8`
  - `transfer_cycles=1`
  - `max_pre_grip_offset=0.0046`
  - `max_return_ready_error=0.0399`
  - `max_release_drift=0.0000`
  - `release_gripper_samples=8`
  - `release_gripper_not_open=0`
  - `release_gripped_object_max=0`
  - `release_gripper_probe_failures=0`
  - `joint_settle_count=8`

---

## 2026-05-29 infeed conveyor visual 및 gate 추가

공정 1번인 "컨베이어로 박스 유입"이 로그와 박스 spawn만으로는 GUI에서 약하게 보였다. 이번 보강에서는 pick station 앞단에 infeed conveyor belt, guide rail, stop line, photo-eye sensor visual을 추가해 박스가 컨베이어 설비에서 들어와 로봇팔이 집는 장면으로 읽히도록 했다.

수정 내용:

- [x] infeed conveyor geometry 상수 추가
  - `INFEED_CONVEYOR_START_Y`
  - `INFEED_CONVEYOR_END_Y`
  - `INFEED_CONVEYOR_WIDTH`
  - `INFEED_CONVEYOR_TOP_Z`
  - `INFEED_GUIDE_RAIL_X_OFFSETS`
  - `INFEED_STOP_LINE_Y`
  - `INFEED_ROLLER_Y_OFFSETS`
- [x] `create_infeed_conveyor_visual()` 추가
  - `InfeedConveyorBelt`
  - `InfeedGuideRail_*`
  - `InfeedRoller_*`
  - `InfeedStopLine`
  - `InfeedPhotoEye_*`
  - `InfeedPhotoEyeBeam`
- [x] `compute_infeed_conveyor_metrics()` 추가
  - conveyor 길이
  - spawn 지점 이후 여유
  - pick/stop line 이전 여유
  - guide rail 내부 clearance
  - belt top과 carton bottom 사이 support gap
- [x] strict self-test gate 추가
  - Python: `--self-test-min-infeed-conveyor-length`
  - Python: `--self-test-min-infeed-spawn-margin`
  - Python: `--self-test-min-infeed-guide-clearance`
  - Python: `--self-test-max-infeed-belt-support-gap`
  - PowerShell wrapper도 동일 옵션 추가
- [x] strict wrapper 기본 gate 추가
  - `SelfTestMinInfeedConveyorLength = 0.80`
  - `SelfTestMinInfeedSpawnMargin = 0.30`
  - `SelfTestMinInfeedGuideClearance = 0.40`
  - `SelfTestMaxInfeedBeltSupportGap = 0.02`

검증 명령:

```powershell
cd E:\Harim_AMR
.\.conda\env_isaacsim_5_1_0\python.exe -m py_compile .\isaac_sim\scripts\run_harim_pallet_demo.py
.\.conda\env_isaacsim_5_1_0\python.exe -m unittest isaac_sim.tests.test_harim_transfer_orchestrator
powershell -ExecutionPolicy Bypass -File .\run_harim_strict_self_test.ps1 -AcceptEula -SelfTestDebugBins
```

검증 결과:

- [x] Python compile 통과
- [x] unittest 45개 통과
- [x] strict wrapper 기반 12000-frame full end-to-end self-test 통과
- [x] 로그 파일: `isaacsim_logs/harim_infeed_conveyor_strict_full_e2e_12000.log`
- [x] 주요 완료 metric:
  - `placed_bins=8`
  - `transfer_cycles=1`
  - `infeed_conveyor_length=0.9000`
  - `infeed_spawn_margin=0.4200`
  - `infeed_pick_margin=0.2200`
  - `infeed_guide_clearance=0.4450`
  - `infeed_belt_support_gap=0.0080`
  - `max_release_drift=0.0000`
  - `max_dropped_payload_drift=0.0000`

---

## 2026-05-29 적재물 banding / load restraint visual 추가

팔레트 위 박스가 안정적으로 이동하더라도, GUI에서 단순히 박스만 얹혀 있으면 AMR이 무거운 적재물을 안전하게 운반한다는 느낌이 약하다. 이번 보강에서는 적재 완료 후에만 나타나는 짙은색 banding/load restraint visual을 추가해 박스 묶음이 팔레트 위에 고정된 것처럼 보이게 했다.

수정 내용:

- [x] load restraint visual 상수 추가
  - `LOAD_RESTRAINT_EXPECTED_PARTS = 6`
  - `LOAD_RESTRAINT_STRAP_WIDTH`
  - `LOAD_RESTRAINT_STRAP_THICKNESS`
  - `LOAD_RESTRAINT_SURFACE_OFFSET`
  - `LOAD_RESTRAINT_COLOR`
- [x] `compute_load_restraint_specs()` 추가
  - stack footprint와 높이를 기준으로 top longitudinal/lateral strap 2개를 계산한다.
  - front/back/left/right vertical strap 4개를 계산한다.
  - 각 strap은 pallet center 기준 offset과 scale로 반환된다.
- [x] `compute_load_restraint_metrics()` 추가
  - `load_restraint_part_count`
  - `min_load_restraint_pallet_margin`
  - `max_load_restraint_pallet_overhang`
- [x] load restraint visual을 `pallet_parts`에 포함
  - AMR lift-up, 이동, lift-down, drop 후 hold까지 팔레트 assembly와 같이 움직인다.
- [x] 적재 완료 전에는 banding을 숨김
  - `reset_visual_state()`에서 `visible=False`
  - `stack_complete` 후 `_lock_stack_items()`에서 `visible=True`
- [x] strict self-test gate 추가
  - Python: `--self-test-min-load-restraint-count`
  - Python: `--self-test-min-load-restraint-pallet-margin`
  - PowerShell: `-SelfTestMinLoadRestraintCount`
  - PowerShell: `-SelfTestMinLoadRestraintPalletMargin`
- [x] strict wrapper에 `SelfTestMinLoadRestraintCount = 6`, `SelfTestMinLoadRestraintPalletMargin = 0.06` 추가

검증 명령:

```powershell
cd E:\Harim_AMR
.\.conda\env_isaacsim_5_1_0\python.exe -m py_compile .\isaac_sim\scripts\run_harim_pallet_demo.py
.\.conda\env_isaacsim_5_1_0\python.exe -m unittest isaac_sim.tests.test_harim_transfer_orchestrator
powershell -ExecutionPolicy Bypass -File .\run_harim_strict_self_test.ps1 -AcceptEula -SelfTestDebugBins
```

검증 결과:

- [x] Python compile 통과
- [x] unittest 44개 통과
- [x] strict wrapper 기반 12000-frame full end-to-end self-test 통과
- [x] 로그 파일: `isaacsim_logs/harim_load_restraint_strict_full_e2e_12000.log`
- [x] 주요 완료 metric:
  - `placed_bins=8`
  - `transfer_cycles=1`
  - `attached 8 stacked items and 18 pallet parts`
  - `load_restraint_part_count=6`
  - `min_load_restraint_pallet_margin=0.0670`
  - `max_load_restraint_pallet_overhang=0.0000`
  - `max_release_drift=0.0000`
  - `max_payload_lift=0.1100`
  - `max_dropped_payload_drift=0.0000`

---

## 2026-05-29 stack gap / support gap 현실성 gate 추가

release와 AMR 하역은 검증됐지만, 박스가 팔레트 위에서 너무 떠 있거나 서로 크게 벌어져 보이는지는 별도 지표가 없었다. 이번 보강에서는 적재 좌표 자체의 lateral gap과 vertical support gap을 계산해 self-test에서 확인한다.

수정 내용:

- [x] `compute_stack_geometry_metrics()` 추가
  - 인접 박스 사이 X/Y air gap을 계산한다.
  - 1층 박스 bottom과 팔레트 top support 사이 gap을 계산한다.
  - 상층 박스 bottom과 하층 박스 top 사이 gap을 계산한다.
- [x] self-test gate 추가
  - Python 옵션: `--self-test-max-stack-lateral-gap`
  - Python 옵션: `--self-test-max-stack-support-gap`
  - PowerShell 옵션: `-SelfTestMaxStackLateralGap`
  - PowerShell 옵션: `-SelfTestMaxStackSupportGap`
- [x] 완료 로그에 stack geometry 지표 추가
  - `max_stack_lateral_gap`
  - `min_stack_lateral_gap`
  - `max_stack_support_gap`
  - `min_stack_support_gap`

검증 명령:

```powershell
cd E:\Harim_AMR
.\.conda\env_isaacsim_5_1_0\python.exe -m py_compile isaac_sim\scripts\run_harim_pallet_demo.py
.\.conda\env_isaacsim_5_1_0\python.exe -m unittest isaac_sim.tests.test_harim_transfer_orchestrator
powershell -ExecutionPolicy Bypass -File .\run_harim_demo.ps1 -Headless -AcceptEula -SelfTestFrames 5600 -SelfTestMinPlacedBins 4 -SelfTestMaxPreGripOffset 0.05 -SelfTestMaxReturnReadyError 0.05 -SelfTestMaxReleaseDrift 0.005 -SelfTestRequireGripperOpenAfterRelease -SelfTestMaxStackLateralGap 0.03 -SelfTestMaxStackSupportGap 0.02 -SelfTestDebugBins -Cycles 1
powershell -ExecutionPolicy Bypass -File .\run_harim_demo.ps1 -Headless -AcceptEula -SelfTestFrames 12000 -SelfTestMinPlacedBins 8 -SelfTestMinTransferCycles 1 -SelfTestMaxPreGripOffset 0.05 -SelfTestMaxReturnReadyError 0.05 -SelfTestMaxReleaseDrift 0.005 -SelfTestRequireGripperOpenAfterRelease -SelfTestMaxStackLateralGap 0.03 -SelfTestMaxStackSupportGap 0.02 -SelfTestMinPayloadLift 0.10 -SelfTestMaxDroppedPayloadDrift 0.005 -SelfTestDebugBins -Cycles 1
```

확인 결과:

- [x] unittest 33개 통과
- [x] Python compile 통과
- [x] 5600-frame stack gap gate 통과
  - `placed_bins=8`
  - `max_stack_lateral_gap=0.0200`
  - `min_stack_lateral_gap=0.0100`
  - `max_stack_support_gap=0.0100`
  - `min_stack_support_gap=0.0025`
- [x] 12000-frame full end-to-end self-test 통과
  - `placed_bins=8`
  - `transfer_cycles=1`
  - `max_pre_grip_offset=0.0046`
  - `max_return_ready_error=0.0399`
  - `max_release_drift=0.0000`
  - `release_gripper_not_open=0`
  - `release_gripped_object_max=0`
  - `joint_settle_count=8`
  - `max_stack_lateral_gap=0.0200`
  - `min_stack_lateral_gap=0.0100`
  - `max_stack_support_gap=0.0100`
  - `min_stack_support_gap=0.0025`
  - `max_payload_lift=0.1100`
  - `max_dropped_payload_drift=0.0000`

---

## 2026-05-29 GUI release-retreat 보강

GUI에서 로봇팔이 박스를 놓지 않는 것처럼 보이는 문제를 줄이기 위해 release 동작을 다시 보강했다. 기존에는 `<open gripper>` 직후 같은 place pose에서 짧게 대기한 뒤 별도 lift state로 넘어갔기 때문에, GUI에서는 박스와 흡착 패드가 계속 붙어 있는 것처럼 보일 수 있었다. 이제 release 상태에 들어가면 박스를 즉시 목표 적재 좌표에 고정하고, 같은 상태 안에서 로봇팔을 위로 retreat시켜 흡착 패드와 박스 사이가 바로 벌어지게 했다.

수정 내용:

- [x] `POST_RELEASE_CLEARANCE_LIFT`를 `0.32m`로 확대
- [x] `RELEASE_RETREAT_DURATION = 0.55s` 추가
- [x] `DemoReleaseBin`에서 release와 동시에 `MotionCommand(self.retreat_pq)`를 보내도록 변경
- [x] release된 bin에 `demo_force_released = True` 표시 추가
- [x] `restore_demo_carried_active_bin()`, `hold_active_bin_for_pick()`, `sync_demo_attached_bin()`에서 release된 bin이 다시 active/carry 상태로 복원되지 않게 차단
- [x] self-test metric `demo_max_release_retreat_lift` 추가
- [x] self-test option 추가
  - Python: `--self-test-min-release-retreat-lift`
  - PowerShell: `-SelfTestMinReleaseRetreatLift`
- [x] strict wrapper에 `SelfTestMinReleaseRetreatLift = 0.20` gate 추가

검증 명령:

```powershell
cd E:\Harim_AMR
.\.conda\env_isaacsim_5_1_0\python.exe -m py_compile .\isaac_sim\scripts\run_harim_pallet_demo.py
.\.conda\env_isaacsim_5_1_0\python.exe -m unittest isaac_sim.tests.test_harim_transfer_orchestrator
powershell -ExecutionPolicy Bypass -File .\run_harim_strict_self_test.ps1 -AcceptEula -SelfTestDebugBins
```

검증 결과:

- [x] Python compile 통과
- [x] unittest 42개 통과
- [x] strict wrapper 기반 12000-frame full end-to-end self-test 통과
- [x] 로그 파일: `isaacsim_logs/harim_release_retreat_gate_strict_full_e2e_12000.log`
- [x] 주요 완료 metric:
  - `placed_bins=8`
  - `transfer_cycles=1`
  - `max_release_drift=0.0000`
  - `max_release_retreat_lift=0.2499`
  - `release_gripper_samples=280`
  - `release_gripper_not_open=0`
  - `release_gripped_object_max=0`
  - `release_gripper_probe_failures=0`
  - `joint_settle_count=8`
## 2026-05-29 GUI release scripted-place 재보강

GUI에서 로봇팔이 박스를 놓지 않는 것처럼 보인다는 피드백을 기준으로 release 직전 구간을 다시 보강했다. 기존에는 `ReachToPlace`가 목표 pose에 정확히 수렴하지 못하면 timeout 후 release로 넘어갔고, 이 구간에서 GUI상 박스가 흡착 패드에 계속 붙어 있는 것처럼 보일 수 있었다. 이제 `ReachToPlace` 뒤에 `DemoScriptedPlaceBin`을 추가해 박스를 목표 적재 좌표로 짧게 정렬한 뒤, `DemoReleaseBin`에서 gripper open, attach 상태 해제, arm retreat를 수행한다.

수정 내용:

- [x] `SCRIPTED_PLACE_DURATION = 0.70`
- [x] `SCRIPTED_PLACE_EE_HOVER = 0.18`
- [x] `DemoScriptedPlaceBin` 추가
  - carried bin을 다음 stack coordinate로 보간 이동
  - `demo_scripted_place_bin` 플래그로 `sync_demo_attached_bin()`의 재부착 보정을 차단
  - suction gripper open을 반복 호출해 실제 surface gripper joint가 남아 있어도 release 쪽으로 강제
- [x] `DemoReleaseBin`에서 release 후 TCP와 박스 사이 분리 거리 기록
  - `demo_max_release_separation`
  - `demo_max_release_vertical_clearance`
- [x] self-test gate 추가
  - Python: `--self-test-min-scripted-place-count`
  - Python: `--self-test-max-scripted-place-error`
  - Python: `--self-test-min-release-separation`
  - PowerShell: `-SelfTestMinScriptedPlaceCount`
  - PowerShell: `-SelfTestMaxScriptedPlaceError`
  - PowerShell: `-SelfTestMinReleaseSeparation`
- [x] strict wrapper gate 추가
  - `SelfTestMinScriptedPlaceCount = 8`
  - `SelfTestMaxScriptedPlaceError = 0.005`
  - `SelfTestMinReleaseSeparation = 0.20`

드롭 위치 작업대 보강:

- [x] drop slide 끝단에 pallet stop block visual 추가
- [x] locator post/cap visual 추가
- [x] self-test gate 추가
  - `drop_dock_stop_count`
  - `drop_dock_stop_gap`
  - `drop_dock_guide_clearance`
  - `drop_dock_fork_clearance`
  - `drop_dock_runner_clearance`

검증 명령:

```powershell
cd E:\Harim_AMR
.\.conda\env_isaacsim_5_1_0\python.exe -m py_compile .\isaac_sim\scripts\run_harim_pallet_demo.py
.\.conda\env_isaacsim_5_1_0\python.exe -m unittest isaac_sim.tests.test_harim_transfer_orchestrator
powershell -ExecutionPolicy Bypass -File .\run_harim_strict_self_test.ps1 -AcceptEula -SelfTestDebugBins
```

검증 결과:

- [x] Python compile 통과
- [x] unittest 46개 통과
- [x] strict wrapper 기반 12000-frame full end-to-end self-test 통과
- [x] 로그 파일: `isaacsim_logs/harim_scripted_place_release_strict_full_e2e_12000.log`
- [x] 주요 완료 metric:
  - `placed_bins=8`
  - `transfer_cycles=1`
  - `scripted_place_count=8`
  - `max_scripted_place_error=0.0000`
  - `max_release_separation=1.6853`
  - `max_release_vertical_clearance=1.0331`
  - `release_gripper_not_open=0`
  - `release_gripped_object_max=0`
  - `max_release_drift=0.0000`
  - `drop_dock_stop_count=2`
  - `drop_dock_stop_gap=0.0350`
  - `drop_dock_guide_clearance=0.1500`
  - `drop_dock_fork_clearance=0.0350`
  - `drop_dock_runner_clearance=0.0650`

---
## 2026-05-29 팔레타이저 셀 안전 펜스 visual/gate 추가

동작은 안정화됐지만 GUI에서 로봇팔 셀이 산업 현장처럼 보이려면 안전 펜스와 출입 게이트가 필요하다. 이번 보강에서는 물리 충돌을 추가하지 않는 `VisualCuboid` 기반 안전 펜스를 팔레타이저 주변에 추가하고, AMR 통로와 infeed conveyor 유입구에는 충분한 opening을 남겼다.

수정 내용:

- [x] `make_safety_fence_specs()` 추가
  - 코너/게이트 post
  - 상/하단 rail 2단
  - AMR 통과 gate
  - infeed conveyor gate
- [x] `create_safety_fence_visual()` 추가
  - `SafetyFenceSouthRail_*`
  - `SafetyFenceNorthRailLeft_*`
  - `SafetyFenceNorthRailRight_*`
  - `SafetyFencePost_WGateLow`
  - `SafetyFencePost_WGateHigh`
  - `SafetyFencePost_EGateLow`
  - `SafetyFencePost_EGateHigh`
  - `SafetyFencePost_InfeedLeft`
  - `SafetyFencePost_InfeedRight`
- [x] self-test gate 추가
  - Python: `--self-test-min-safety-fence-part-count`
  - Python: `--self-test-min-safety-fence-amr-gate-clearance`
  - Python: `--self-test-min-safety-fence-infeed-gate-clearance`
  - PowerShell: `-SelfTestMinSafetyFencePartCount`
  - PowerShell: `-SelfTestMinSafetyFenceAmrGateClearance`
  - PowerShell: `-SelfTestMinSafetyFenceInfeedGateClearance`
- [x] strict wrapper gate 추가
  - `SelfTestMinSafetyFencePartCount = 20`
  - `SelfTestMinSafetyFenceAmrGateClearance = 0.25`
  - `SelfTestMinSafetyFenceInfeedGateClearance = 0.20`

검증 명령:

```powershell
cd E:\Harim_AMR
.\.conda\env_isaacsim_5_1_0\python.exe -m py_compile .\isaac_sim\scripts\run_harim_pallet_demo.py
.\.conda\env_isaacsim_5_1_0\python.exe -m unittest isaac_sim.tests.test_harim_transfer_orchestrator
powershell -ExecutionPolicy Bypass -File .\run_harim_strict_self_test.ps1 -AcceptEula -SelfTestDebugBins
```

검증 결과:

- [x] Python compile 통과
- [x] unittest 47개 통과
- [x] strict wrapper 기반 12000-frame full end-to-end self-test 통과
- [x] 로그 파일: `isaacsim_logs/harim_safety_fence_strict_full_e2e_12000.log`
- [x] 주요 완료 metric:
  - `placed_bins=8`
  - `transfer_cycles=1`
  - `safety_fence_part_count=24`
  - `safety_fence_amr_gate_clearance=0.3050`
  - `safety_fence_infeed_gate_clearance=0.2250`
  - `scripted_place_count=8`
  - `release_gripper_not_open=0`
  - `release_gripped_object_max=0`
  - `max_dropped_payload_drift=0.0000`

---
## 2026-05-29 AMR safety beacon/scanner visual 추가

iw_hub가 팔레트를 들고 이동할 때 장비 자체가 너무 기본 asset처럼 보이는 문제가 있어, AMR 위에 안전 beacon과 전후방 scanner, 좌우 status strip visual을 추가했다. visual은 독립 prim으로 생성하되 `HarimTransferOrchestrator.set_amr_pose()`에서 AMR pose와 함께 동기화한다.

수정 내용:

- [x] `AMR_SAFETY_VISUAL_SPECS` 추가
  - `AmrBeaconPole`
  - `AmrBeaconDome`
  - `AmrFrontSafetyScanner`
  - `AmrRearSafetyScanner`
  - `AmrLeftStatusStrip`
  - `AmrRightStatusStrip`
- [x] `create_amr_safety_visuals()` 추가
- [x] `HarimTransferOrchestrator`에 AMR safety visual 동기화 추가
  - `amr_safety_parts`
  - `amr_safety_offsets`
  - `_set_amr_safety_visual_pose()`
  - `max_amr_safety_pose_error`
- [x] self-test gate 추가
  - Python: `--self-test-min-amr-safety-part-count`
  - Python: `--self-test-min-amr-safety-beacon-height`
  - Python: `--self-test-min-amr-safety-scanner-clearance`
  - Python: `--self-test-max-amr-safety-pose-error`
  - PowerShell: `-SelfTestMinAmrSafetyPartCount`
  - PowerShell: `-SelfTestMinAmrSafetyBeaconHeight`
  - PowerShell: `-SelfTestMinAmrSafetyScannerClearance`
  - PowerShell: `-SelfTestMaxAmrSafetyPoseError`
- [x] strict wrapper gate 추가
  - `SelfTestMinAmrSafetyPartCount = 6`
  - `SelfTestMinAmrSafetyBeaconHeight = 0.60`
  - `SelfTestMinAmrSafetyScannerClearance = 0.10`
  - `SelfTestMaxAmrSafetyPoseError = 0.005`

검증 명령:

```powershell
cd E:\Harim_AMR
.\.conda\env_isaacsim_5_1_0\python.exe -m py_compile .\isaac_sim\scripts\run_harim_pallet_demo.py
.\.conda\env_isaacsim_5_1_0\python.exe -m unittest isaac_sim.tests.test_harim_transfer_orchestrator
powershell -ExecutionPolicy Bypass -File .\run_harim_strict_self_test.ps1 -AcceptEula -SelfTestDebugBins
```

검증 결과:

- [x] Python compile 통과
- [x] unittest 49개 통과
- [x] strict wrapper 기반 12000-frame full end-to-end self-test 통과
- [x] 로그 파일: `isaacsim_logs/harim_amr_safety_visuals_strict_full_e2e_12000.log`
- [x] 주요 완료 metric:
  - `placed_bins=8`
  - `transfer_cycles=1`
  - `amr_safety_part_count=6`
  - `amr_safety_beacon_height=0.7450`
  - `amr_safety_scanner_clearance=0.1325`
  - `max_amr_safety_pose_error=0.0000`
  - `max_payload_lift=0.1100`
  - `max_dropped_payload_drift=0.0000`

---
