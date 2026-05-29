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

```powershell
powershell -ExecutionPolicy Bypass -File .\run_harim_demo.ps1 -Cycles 1 -StackCols 3 -StackRows 2 -StackLayers 2
```

## 구현 구조

- 공식 UR10 palletizing Cortex 예제를 기반으로 컨베이어 유입, suction gripper pick, 팔레트 위 적재를 수행합니다.
- Cortex behavior의 `stack_complete` 상태를 감시합니다.
- 적재 완료 후 custom orchestrator가 `iw_hub`를 pickup pose로 이동시킵니다.
- LiftUp 단계에서 팔레트와 적재물을 들어 올리는 연출을 수행합니다.
- 이동 중에는 팔레트와 적재된 bin assembly를 `iw_hub` 기준 offset으로 따라가게 합니다.
- Drop pose에서 LiftDown, detach 후 `iw_hub`가 이탈합니다.
- `--cycles 0` 기본값은 무한 반복입니다.

## 주의

첫 실행 시 NVIDIA Isaac asset 다운로드와 shader cache 생성 때문에 시간이 오래 걸릴 수 있습니다.
NVIDIA EULA 확인이 필요한 환경에서는 Isaac Sim 첫 실행 단계에서 사용자 확인이 필요할 수 있습니다.

## 로컬 검증

Isaac Sim을 띄우지 않고 custom orchestrator FSM만 검증하려면 다음 unittest를 실행합니다.

```powershell
cd E:\Harim_AMR
.\.conda\env_isaacsim_5_1_0\python.exe -m unittest .\isaac_sim\tests\test_harim_transfer_orchestrator.py
```
