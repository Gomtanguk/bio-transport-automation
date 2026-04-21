# bio-transport-automation

바이오뱅크 워크셀에서 RACK 이송과 튜브 이송을 ROS 2 Action 기반으로 통합 제어하는 프로젝트입니다. UI, 메인 오케스트레이터, 로봇 제어 노드, 인터페이스 패키지, Doosan 드라이버 소스를 한 저장소에서 관리합니다.

## Quick Summary

- Domain: 바이오뱅크 검체 이송 자동화
- Stack: ROS 2 Humble, Python, PySide6, Doosan ROS 2
- Key Packages: `bio_transport`, `bio_transport_interfaces`, `doosan-robot2`

## 구조

```text
bio-transport-automation/
├── docs/
│   ├── Flow_chart.png
│   └── archive/
├── src/
│   ├── bio_transport/
│   ├── bio_transport_interfaces/
│   └── doosan-robot2/
└── README.md
```

## 주요 패키지

- `bio_transport`
  - `bio_main`: 메인 오케스트레이터
  - `bio_sub`: 로봇 제어 액션 서버
  - `bio_ui`: PySide6 기반 UI
- `bio_transport_interfaces`
  - `BioCommand`, `RobotMove`, `TubeTransport` 액션 정의
- `doosan-robot2`
  - Doosan ROS 2 드라이버 및 관련 패키지

## 동작 개요

- UI가 RACK/TUBE/HOME/EMERGENCY 명령을 전송합니다.
- `bio_main`이 명령 파싱, 순서 제어, 하위 액션 호출을 담당합니다.
- `bio_sub`가 실제 로봇 동작과 HOME 복귀 로직을 수행합니다.
- 좌표와 스테이션 정보는 `src/bio_transport/bio_transport/rack_stations.py`를 기준으로 관리합니다.

## 알고리즘 개요

### 1. 명령 파싱과 디스패치

- UI 문자열 명령을 `RACK`, `TUBE`, `HOME`, `EMERGENCY` 유형으로 분류합니다.
- 명령 포맷과 소스/목적지 좌표를 검증한 뒤 적절한 액션 서버로 라우팅합니다.
- 중복 실행을 막기 위해 메인 오케스트레이터가 순차 실행과 상태 잠금을 담당합니다.

### 2. 스테이션 기반 위치 결정

- 랙과 튜브 위치는 사전 정의된 스테이션 테이블을 기준으로 해석합니다.
- 명령에서 받은 위치 이름을 실제 접근 포인트와 작업 좌표로 변환합니다.
- 이 과정에서 RACK 이송과 TUBE 이송이 서로 다른 좌표 체계를 쓰더라도 상위 로직에서는 동일한 명령 인터페이스를 유지합니다.

### 3. 액션 기반 작업 시퀀스

- 상위 노드는 `RobotMove`, `TubeTransport`, `BioCommand` 액션을 통해 하위 작업을 요청합니다.
- 하위 제어 노드는 pick, move, place, home 복귀 같은 로봇 작업 단계를 순서대로 실행합니다.
- 각 단계는 성공/실패 상태를 액션 결과로 반환해 상위 노드가 다음 흐름을 결정할 수 있게 합니다.

### 4. 예외와 복구

- SAFE 상태나 긴급 정지 이벤트를 감지하면 즉시 현재 작업 흐름을 중단합니다.
- 필요 시 HOME 복귀 시퀀스를 실행해 다음 작업이 가능한 초기 상태로 복원합니다.
- 이 구조 덕분에 작업 중 오류가 나도 UI와 상위 제어 로직은 실패 원인을 명확히 구분할 수 있습니다.

## 워크스페이스 빌드

```bash
source /opt/ros/humble/setup.bash
cd <workspace-root>
colcon build --symlink-install
source install/setup.bash
```

이 저장소 자체를 워크스페이스로 사용할 경우 `<workspace-root>`는 저장소 루트입니다.

## 실행 예시

통합 런치:

```bash
ros2 launch bio_transport bio_integrated.launch.py mode:=real host:=192.168.1.100 dry_run:=false
```

개별 실행:

```bash
ros2 run bio_transport bio_sub --ros-args -p dry_run:=true
ros2 run bio_transport bio_main
ros2 run bio_transport bio_ui
```

## 환경

- Ubuntu 22.04
- ROS 2 Humble
- Python 3.10 계열
- Doosan M0609 기준 설정
- PySide6 기반 UI

## 참고

- 흐름도는 [docs/Flow_chart.png](c:/rokey/bio-transport-automation/docs/Flow_chart.png) 에 있습니다.
- 데모 영상은 저장소 밖 링크나 별도 배포 채널로 공유하는 것을 권장합니다.
- 발표자료와 대용량 보관 파일은 로컬 `docs/archive/`에 `bio-transport-automation-*` 형식으로 정리했습니다.
- 라이선스 파일은 `src/bio_transport/LICENSE`를 참고하면 됩니다.
