# Isaac Sim Demo Details

이 폴더는 Harim AMR 데모의 Isaac Sim 실행 코드와 테스트를 담고 있습니다.

## 현재 구현

- Isaac Sim 5.1 pip 환경을 사용합니다.
- 공식 UR10 bin stacking/palletizing 예제 asset을 기반으로 컨베이어, UR10, suction gripper, 예제 팔레트를 사용합니다.
- bin은 처음부터 upside-down orientation으로 스폰해서 flip station을 거치지 않고 바로 place합니다.
- `pallet_holder`와 `flip` 관련 prim은 no-flip 흐름에 맞춰 비활성화합니다.
- 예제 팔레트 prim 자체는 유지하고, AMR lift-up/down 및 이송 시퀀스에 맞춰 위치만 갱신합니다.
- `iw_hub`는 Isaac Sim sample asset을 reference해서 사용합니다.
- `iw_hub/chassis/lift` prim이 있으면 해당 lift prim을 함께 움직입니다.
- 첨부 레퍼런스 정면 형상의 pickup/drop slide station을 생성합니다. 검정 벌키 좌우 지지대, 검정 중앙 브리지, 은색 상부 리니어 레일, 흰 전면 하단 패널, 노란 삼각 경고 라벨을 좌우 대칭으로 배치합니다.
- 별도로 만든 visual pallet과 lift plate는 현재 사용하지 않습니다.
- self-test용 payload는 임의 cuboid가 아니라 UR10 예제의 `small_KLT.usd`를 reference합니다.

## 실행

루트 디렉터리에서 실행합니다.

Ubuntu:

```bash
./run_harim_demo.sh --accept-eula --cycles 1
./run_harim_demo.sh --headless --accept-eula --self-test-frames 2 --cycles 1
./run_harim_demo.sh --headless --accept-eula --self-test-frames 260 --self-test-force-stack-complete --cycles 1 --move-speed 20
./run_harim_demo.sh --headless --accept-eula --self-test-frames 2 --cycles 1 --capture-path isaacsim_logs/reference_station_capture.png
```

Windows:

```powershell
powershell -ExecutionPolicy Bypass -File .\run_harim_demo.ps1 -AcceptEula -Cycles 1
powershell -ExecutionPolicy Bypass -File .\run_harim_demo.ps1 -Headless -AcceptEula -SelfTestFrames 2 -Cycles 1
powershell -ExecutionPolicy Bypass -File .\run_harim_demo.ps1 -Headless -AcceptEula -SelfTestFrames 260 -SelfTestForceStackComplete -Cycles 1 -MoveSpeed 20
powershell -ExecutionPolicy Bypass -File .\run_harim_demo.ps1 -Headless -AcceptEula -SelfTestFrames 2 -Cycles 1 -CapturePath isaacsim_logs\reference_station_capture.png
```

## 테스트

Isaac Sim을 띄우지 않고 orchestration FSM과 소스 구조를 검증합니다.

Ubuntu:

```bash
./.conda/env_isaacsim_5_1_0/bin/python -m py_compile ./isaac_sim/scripts/run_harim_pallet_demo.py ./isaac_sim/tests/test_harim_transfer_orchestrator.py
./.conda/env_isaacsim_5_1_0/bin/python -m unittest isaac_sim.tests.test_harim_transfer_orchestrator
```

Windows:

```powershell
.\.conda\env_isaacsim_5_1_0\python.exe -m py_compile .\isaac_sim\scripts\run_harim_pallet_demo.py .\isaac_sim\tests\test_harim_transfer_orchestrator.py
.\.conda\env_isaacsim_5_1_0\python.exe -m unittest isaac_sim.tests.test_harim_transfer_orchestrator
```

## 파일

- `scripts/run_harim_pallet_demo.py`: 데모 실행 본체
- `tests/test_harim_transfer_orchestrator.py`: lightweight unit tests

루트의 [README.md](../README.md)와 [CONTRIBUTING.md](../CONTRIBUTING.md)에 협업 절차와 환경 설치 방법이 정리되어 있습니다.
