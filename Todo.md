# Harim AMR Isaac Sim 구현 Todo

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
