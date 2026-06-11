# Contributing

이 저장소는 Isaac Sim 데모 프로젝트이므로, 코드 변경과 시각 검증을 분리해서 협업합니다.

## 기본 원칙

- `.conda`, Isaac Sim cache, 렌더링 결과물, 로그는 커밋하지 않습니다.
- 저장소는 임의의 로컬 폴더에 clone할 수 있습니다.
- Isaac Sim 5.1 pip 환경은 프로젝트 폴더 안의 `.conda/env_isaacsim_5_1_0`에 둡니다.
- Ubuntu 24.04 동료는 `.sh` 스크립트를 사용하고, Windows 동료는 `.ps1` 스크립트를 사용합니다.
- NVIDIA EULA 동의는 각 실행자가 직접 확인합니다.

## 브랜치 규칙

작업 단위별로 브랜치를 만듭니다.

```text
feature/<short-name>
fix/<short-name>
docs/<short-name>
test/<short-name>
```

Codex 작업 브랜치가 필요하면 `codex/<short-name>`을 사용합니다.

## PR 전에 확인할 것

Ubuntu:

```bash
./.conda/env_isaacsim_5_1_0/bin/python -m py_compile ./isaac_sim/scripts/run_harim_pallet_demo.py ./isaac_sim/tests/test_harim_transfer_orchestrator.py
./.conda/env_isaacsim_5_1_0/bin/python -m unittest isaac_sim.tests.test_harim_transfer_orchestrator
./run_harim_demo.sh --headless --accept-eula --self-test-frames 2 --cycles 1
./run_harim_demo.sh --headless --accept-eula --self-test-frames 2 --cycles 1 --capture-path isaacsim_logs/reference_station_capture.png
```

Windows:

```powershell
.\.conda\env_isaacsim_5_1_0\python.exe -m py_compile .\isaac_sim\scripts\run_harim_pallet_demo.py .\isaac_sim\tests\test_harim_transfer_orchestrator.py
.\.conda\env_isaacsim_5_1_0\python.exe -m unittest isaac_sim.tests.test_harim_transfer_orchestrator
powershell -ExecutionPolicy Bypass -File .\run_harim_demo.ps1 -Headless -AcceptEula -SelfTestFrames 2 -Cycles 1
powershell -ExecutionPolicy Bypass -File .\run_harim_demo.ps1 -Headless -AcceptEula -SelfTestFrames 2 -Cycles 1 -CapturePath isaacsim_logs\reference_station_capture.png
```

AMR transfer 시퀀스까지 바꾼 경우:

```bash
./run_harim_demo.sh --headless --accept-eula --self-test-frames 260 --self-test-force-stack-complete --cycles 1 --move-speed 20
```

```powershell
powershell -ExecutionPolicy Bypass -File .\run_harim_demo.ps1 -Headless -AcceptEula -SelfTestFrames 260 -SelfTestForceStackComplete -Cycles 1 -MoveSpeed 20
```

GUI 동작이나 시각 연출을 바꾸면 PR 설명에 다음 중 하나를 남깁니다.

- 확인한 실행 명령
- 주요 로그 문구
- 스크린샷 또는 GIF 경로
- 아직 확인하지 못한 이유

## 코드 스타일

- 기존 단일 스크립트 구조를 크게 흔들지 않습니다.
- Isaac Sim import는 가능한 `main()` 내부에 둬서 lightweight unit test가 Isaac Sim 설치 없이도 최대한 돌 수 있게 합니다.
- 시각 에셋을 추가할 때는 임의 primitive를 늘리기보다 Isaac Sim 공식 sample/asset reference를 우선 사용합니다.
- GUI에서 확인해야 하는 변경은 headless 테스트만으로 완료 처리하지 않습니다.

## 커밋 메시지

명령형 한 줄을 기본으로 씁니다.

```text
Use example pallet asset for AMR transfer
Add GUI camera capture workflow
Fix lift prim alignment
```
