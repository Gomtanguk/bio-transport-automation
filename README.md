# bio-transport-automation

바이오뱅크 워크셀에서 RACK 이송과 튜브 이송을 ROS 2 Action 기반으로 통합 제어하는 프로젝트입니다. UI, 메인 오케스트레이터, 로봇 제어 노드, 인터페이스 패키지, Doosan 드라이버 소스를 한 저장소에서 관리합니다.

## Quick Summary

- Domain: 바이오뱅크 검체 이송 자동화
- Stack: ROS 2 Humble, Python, PySide6, Doosan ROS 2
- Key Packages: `bio_transport`, `bio_transport_interfaces`, `doosan-robot2`

## Project Goal

- 작업자가 수동으로 처리하던 랙 이송과 튜브 이송 절차를 하나의 ROS 2 기반 제어 흐름으로 통합합니다.
- UI 입력, 명령 해석, 좌표 변환, 로봇 동작, 예외 복구를 분리된 노드 구조로 설계합니다.
- 실제 장비 환경에서도 재사용할 수 있도록 Action 기반 인터페이스와 스테이션 테이블 중심 구조를 유지합니다.

## Problem Statement

- 바이오뱅크 워크셀에서는 이송 대상 종류에 따라 작업 순서와 좌표 체계가 달라집니다.
- 단순 스크립트 기반 제어로는 UI 연동, 작업 상태 추적, 중복 실행 방지, 긴급 정지 복구가 불안정해지기 쉽습니다.
- 이 프로젝트는 상위 오케스트레이터가 작업 흐름을 관리하고, 하위 노드가 실제 동작을 수행하는 구조로 이런 문제를 해결합니다.

## Key Features

- PySide6 기반 UI에서 `RACK`, `TUBE`, `HOME`, `EMERGENCY` 명령 입력
- 메인 오케스트레이터의 순차 실행 제어와 상태 잠금
- Action 기반 로봇 작업 요청과 결과 회수
- 스테이션 이름을 실제 접근 좌표로 변환하는 테이블 기반 구조
- 긴급 정지 및 HOME 복귀를 포함한 복구 흐름

## Repository Structure

```text
bio-transport-automation/
├── docs/
│   ├── Flow_chart.png
│   └── archive/
├── src/
│   ├── bio_transport/
│   ├── bio_transport_interfaces/
│   └── doosan-robot2/
├── requirements.txt
└── README.md
```

## Main Packages

- `bio_transport`
  - `bio_main`: 메인 오케스트레이터
  - `bio_sub`: 로봇 제어 액션 서버
  - `bio_ui`: PySide6 기반 UI
- `bio_transport_interfaces`
  - `BioCommand`, `RobotMove`, `TubeTransport` 액션 정의
- `doosan-robot2`
  - Doosan ROS 2 드라이버 및 관련 패키지

## System Flow

1. UI가 작업 명령을 입력합니다.
2. `bio_main`이 명령 타입을 판별하고 유효성을 검사합니다.
3. 스테이션 이름을 실제 접근 좌표와 작업 좌표로 변환합니다.
4. `bio_main`이 하위 액션 서버에 작업을 요청합니다.
5. `bio_sub`가 pick, move, place, home 복귀 순서로 실제 동작을 수행합니다.
6. 작업 성공 또는 실패 결과를 상위로 반환하고 다음 상태를 결정합니다.

## Technical Highlights

### 1. Command Parsing and Dispatch

- UI 문자열 명령을 `RACK`, `TUBE`, `HOME`, `EMERGENCY` 유형으로 분류합니다.
- 명령 포맷과 소스/목적지 좌표를 검증한 뒤 적절한 액션 서버로 라우팅합니다.
- 중복 실행을 막기 위해 메인 오케스트레이터가 순차 실행과 상태 잠금을 담당합니다.

### 2. Station-Based Coordinate Resolution

- 랙과 튜브 위치는 사전 정의된 스테이션 테이블을 기준으로 해석합니다.
- 명령에서 받은 위치 이름을 실제 접근 포인트와 작업 좌표로 변환합니다.
- 서로 다른 좌표 체계를 상위 명령 인터페이스 하나로 묶어 운용할 수 있습니다.

### 3. Action-Oriented Task Sequencing

- 상위 노드는 `RobotMove`, `TubeTransport`, `BioCommand` 액션을 통해 하위 작업을 요청합니다.
- 하위 제어 노드는 pick, move, place, home 복귀 같은 로봇 작업 단계를 순서대로 실행합니다.
- 각 단계는 성공/실패 상태를 액션 결과로 반환해 상위 흐름 제어가 가능하도록 합니다.

### 4. Safety and Recovery

- SAFE 상태나 긴급 정지 이벤트를 감지하면 즉시 현재 작업 흐름을 중단합니다.
- 필요 시 HOME 복귀 시퀀스를 실행해 다음 작업이 가능한 초기 상태로 복원합니다.
- 오류 원인을 UI와 상위 제어 로직에서 구분할 수 있도록 실패 결과를 명시적으로 반환합니다.

## Build

```bash
source /opt/ros/humble/setup.bash
cd <workspace-root>
colcon build --symlink-install
source install/setup.bash
```

이 저장소 자체를 워크스페이스로 사용할 경우 `<workspace-root>`는 저장소 루트입니다.

## Run

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

## Environment

- Ubuntu 22.04
- ROS 2 Humble
- Python 3.10 계열
- Doosan M0609 기준 설정
- PySide6 기반 UI

## Portfolio Notes

- 흐름도는 [docs/Flow_chart.png](c:/rokey/bio-transport-automation/docs/Flow_chart.png)에 있습니다.
- 데모 영상은 저장소 밖 링크나 별도 배포 채널로 공유하는 것을 권장합니다.
- 발표자료와 대용량 보관 파일은 로컬 `docs/archive/`에 `bio-transport-automation-*` 형식으로 정리했습니다.
- 라이선스 파일은 `src/bio_transport/LICENSE`를 참고하면 됩니다.
