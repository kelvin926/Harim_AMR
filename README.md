# Harim AMR Isaac Sim Demo

Isaac Sim 5.1 기반으로 UR10 suction 팔레타이징 예제와 `iw_hub` AMR 팔레트 이송 시퀀스를 연결한 데모입니다.

현재 1차 목표는 실제 제어 검증보다 설명 가능한 시뮬레이션 데모입니다.

1. 컨베이어로 KLT/bin 유입
2. UR10 suction gripper가 pick
3. 예제 팔레트 위에 여러 층 적재
4. 적재 완료 감지
5. `iw_hub`가 멀리서 접근
6. 팔레트 lift-up
7. 목표 위치로 이송
8. lift-down 후 AMR 이탈

## 지원 환경

Isaac Sim 5.1은 x86_64 기준 Ubuntu 22.04/24.04와 Windows 10/11을 지원합니다. 이 저장소는 두 플랫폼을 모두 고려해 실행 래퍼를 제공합니다.

- Ubuntu 24.04: `setup_isaacsim_env.sh`, `run_harim_demo.sh`
- Windows 10/11: `setup_isaacsim_env.ps1`, `run_harim_demo.ps1`

NVIDIA RTX GPU, 적절한 NVIDIA driver, 인터넷 연결이 필요합니다. 첫 실행 시 Isaac asset 다운로드와 shader cache 생성 때문에 시간이 오래 걸릴 수 있습니다.

## 저장소 사용 방식

이 저장소에는 소스, 실행 스크립트, 문서, 테스트만 커밋합니다. 다음 로컬 실행 환경과 캐시는 커밋하지 않습니다.

- `.conda/`
- `.conda_pkgs/`
- `.pip_cache/`
- `.kit_cache/`
- `.omni_cache/`
- `.omni_user/`
- `isaacsim_logs/`
- `_video_frames/`
- 렌더링 결과 영상/GIF

## Ubuntu 24.04 설치 및 실행

아래 예시는 `~/projects/Harim_AMR`에 받는 경우입니다. 원하는 폴더로 바꿔도 됩니다.

```bash
git clone https://github.com/kelvin926/Harim_AMR.git ~/projects/Harim_AMR
cd ~/projects/Harim_AMR
chmod +x setup_isaacsim_env.sh run_harim_demo.sh
./setup_isaacsim_env.sh
```

GUI 실행:

```bash
./run_harim_demo.sh --accept-eula --cycles 1
```

짧은 headless 초기화 확인:

```bash
./run_harim_demo.sh --headless --accept-eula --self-test-frames 2 --cycles 1
```

AMR 이송 시퀀스 빠른 검증:

```bash
./run_harim_demo.sh --headless --accept-eula --self-test-frames 260 --self-test-force-stack-complete --cycles 1 --move-speed 20
```

`--accept-eula`는 NVIDIA Omniverse/Isaac Sim EULA 내용을 확인하고 동의한 경우에만 사용합니다.

## Windows 설치 및 실행

아래 예시는 `C:\Projects\Harim_AMR`에 받는 경우입니다. 원하는 폴더로 바꿔도 됩니다.

```powershell
git clone https://github.com/kelvin926/Harim_AMR.git C:\Projects\Harim_AMR
cd C:\Projects\Harim_AMR
powershell -ExecutionPolicy Bypass -File .\setup_isaacsim_env.ps1
```

GUI 실행:

```powershell
powershell -ExecutionPolicy Bypass -File .\run_harim_demo.ps1 -AcceptEula -Cycles 1
```

짧은 headless 초기화 확인:

```powershell
powershell -ExecutionPolicy Bypass -File .\run_harim_demo.ps1 -Headless -AcceptEula -SelfTestFrames 2 -Cycles 1
```

AMR 이송 시퀀스 빠른 검증:

```powershell
powershell -ExecutionPolicy Bypass -File .\run_harim_demo.ps1 -Headless -AcceptEula -SelfTestFrames 260 -SelfTestForceStackComplete -Cycles 1 -MoveSpeed 20
```

## 로컬 테스트

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

GitHub Actions에서는 Isaac Sim 전체를 설치하지 않고 `numpy`만 설치해 lightweight 테스트를 실행합니다. Isaac Sim stage loading과 렌더링은 로컬 GPU 환경에서 확인합니다.

## 협업 흐름

1. 작업 전 최신 `main`을 pull합니다.
2. 기능/수정 단위로 브랜치를 만듭니다.
   - 예: `feature/gui-camera-gif`
   - 예: `fix/pallet-alignment`
3. 코드 변경 후 최소 테스트를 실행합니다.
4. Pull Request를 열고 PR 템플릿의 체크리스트를 채웁니다.
5. GUI/Isaac Sim 동작을 바꾼 경우 스크린샷, GIF, 로그 요약 중 하나를 PR에 첨부합니다.
6. 리뷰 후 `main`에 merge합니다.

자세한 기여 규칙은 [CONTRIBUTING.md](CONTRIBUTING.md)를 봅니다.

## 주요 파일

- `run_harim_demo.sh`: Ubuntu/Linux 실행 래퍼
- `setup_isaacsim_env.sh`: Ubuntu/Linux Isaac Sim 5.1 pip 환경 설치 스크립트
- `run_harim_demo.ps1`: Windows PowerShell 실행 래퍼
- `setup_isaacsim_env.ps1`: Windows Isaac Sim 5.1 pip 환경 설치 스크립트
- `requirements-isaacsim.txt`: Isaac Sim pip 설치 requirements
- `requirements-ci.txt`: GitHub Actions lightweight test requirements
- `isaac_sim/scripts/run_harim_pallet_demo.py`: 데모 본체
- `isaac_sim/tests/test_harim_transfer_orchestrator.py`: Isaac Sim 없이 실행 가능한 FSM/소스 순서 테스트
- `isaac_sim/README.md`: Isaac Sim 데모 구현 상세
- `Todo.md`: 프로젝트 진행 메모
