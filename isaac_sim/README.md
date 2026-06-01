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
- 별도로 만든 visual pallet, lift plate, drop slide workstation은 현재 사용하지 않습니다.
- self-test용 payload는 임의 cuboid가 아니라 UR10 예제의 `small_KLT.usd`를 reference합니다.

## 실행

루트 디렉터리에서 실행합니다. 아래 `C:\Projects\Harim_AMR`는 예시이며, 실제 clone한 폴더로 바꿔도 됩니다.

```powershell
cd C:\Projects\Harim_AMR
powershell -ExecutionPolicy Bypass -File .\run_harim_demo.ps1 -AcceptEula -Cycles 1
```

Headless 초기화 확인:

```powershell
powershell -ExecutionPolicy Bypass -File .\run_harim_demo.ps1 -Headless -AcceptEula -SelfTestFrames 2 -Cycles 1
```

AMR transfer 빠른 검증:

```powershell
powershell -ExecutionPolicy Bypass -File .\run_harim_demo.ps1 -Headless -AcceptEula -SelfTestFrames 260 -SelfTestForceStackComplete -Cycles 1 -MoveSpeed 20
```

## 테스트

Isaac Sim을 띄우지 않고 orchestration FSM과 소스 구조를 검증합니다.

```powershell
.\.conda\env_isaacsim_5_1_0\python.exe -m py_compile .\isaac_sim\scripts\run_harim_pallet_demo.py .\isaac_sim\tests\test_harim_transfer_orchestrator.py
.\.conda\env_isaacsim_5_1_0\python.exe -m unittest isaac_sim.tests.test_harim_transfer_orchestrator
```

## 파일

- `scripts/run_harim_pallet_demo.py`: 데모 실행 본체
- `tests/test_harim_transfer_orchestrator.py`: lightweight unit tests

루트의 [README.md](../README.md)와 [CONTRIBUTING.md](../CONTRIBUTING.md)에 협업 절차와 환경 설치 방법이 정리되어 있습니다.
