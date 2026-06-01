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

## 저장소 사용 방식

이 저장소에는 소스, 실행 스크립트, 문서, 테스트만 커밋합니다.
다음 로컬 실행 환경과 캐시는 커밋하지 않습니다.

- `.conda/`
- `.conda_pkgs/`
- `.pip_cache/`
- `.kit_cache/`
- `.omni_cache/`
- `.omni_user/`
- `isaacsim_logs/`
- `_video_frames/`
- 렌더링 결과 영상/GIF

## 처음 받는 사람용 설치

Windows PowerShell 기준입니다. 아래 예시는 `C:\Projects\Harim_AMR`에 받는 경우이며, 원하는 폴더로 바꿔도 됩니다.

```powershell
git clone https://github.com/kelvin926/Harim_AMR.git C:\Projects\Harim_AMR
cd C:\Projects\Harim_AMR
powershell -ExecutionPolicy Bypass -File .\setup_isaacsim_env.ps1
```

설치 스크립트는 현재 프로젝트 폴더 안의 `.conda\env_isaacsim_5_1_0`에 Python 3.11 conda 환경을 만들고 Isaac Sim 5.1 pip 패키지를 설치합니다.
이미 환경이 있으면 덮어쓰지 않습니다.
다른 Python 실행 파일을 써야 하는 특수 환경에서는 `run_harim_demo.ps1 -PythonExe <path-to-python.exe>`로 지정할 수 있습니다.

## 실행

GUI 실행:

```powershell
cd C:\Projects\Harim_AMR
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

`-AcceptEula`는 NVIDIA Omniverse/Isaac Sim EULA 내용을 확인하고 동의한 경우에만 사용합니다.

## 로컬 테스트

Isaac Sim GUI를 띄우지 않는 가벼운 검증입니다.

```powershell
cd C:\Projects\Harim_AMR
.\.conda\env_isaacsim_5_1_0\python.exe -m py_compile .\isaac_sim\scripts\run_harim_pallet_demo.py .\isaac_sim\tests\test_harim_transfer_orchestrator.py
.\.conda\env_isaacsim_5_1_0\python.exe -m unittest isaac_sim.tests.test_harim_transfer_orchestrator
```

GitHub Actions에서는 Isaac Sim 전체를 설치하지 않고 `numpy`만 설치해 위 lightweight 테스트를 실행합니다.
Isaac Sim stage loading과 렌더링은 로컬 GPU 환경에서 확인합니다.

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

- `run_harim_demo.ps1`: Windows PowerShell 실행 래퍼
- `setup_isaacsim_env.ps1`: 새 협업자용 Isaac Sim 5.1 pip 환경 설치 스크립트
- `requirements-isaacsim.txt`: 다른 컴퓨터에서 동일 Isaac Sim pip 패키지를 설치하기 위한 requirements 파일
- `isaac_sim/scripts/run_harim_pallet_demo.py`: 데모 본체
- `isaac_sim/tests/test_harim_transfer_orchestrator.py`: Isaac Sim 없이 실행 가능한 FSM/소스 순서 테스트
- `isaac_sim/README.md`: Isaac Sim 데모 구현 상세
- `Todo.md`: 프로젝트 진행 메모
