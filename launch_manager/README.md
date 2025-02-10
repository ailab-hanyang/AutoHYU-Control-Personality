# AutoHYU Launch Manager

AutoHYU-Control 프로젝트의 다양한 ROS 노드들을 쉽게 실행하고 관리할 수 있는 GUI 도구입니다.

## 주요 기능

- 환경 설정 관리 (MAP, CAR, MODE, PROJ)
- 개별 실행/종료 버튼을 통한 프로세스 제어
- LED 상태 표시기를 통한 모니터링 (초록색: 실행 중, 어두운 초록색: 정지됨, 빨간색: 오류)
- 오류 감지 및 로깅
- 프로그램 종료 시 자동 프로세스 정리
- GUI를 통한 설정 파일 관리

## 요구사항

- Python 3
- PyQt5
- ROS Noetic

## 설치

PyQt5 설치:
```bash
pip3 install PyQt5
```

## 사용법

1. 프로그램 실행:
```bash
./launch_manager_gui.py
```

2. 환경 설정:
   - 드롭다운 메뉴에서 MAP, CAR, MODE, PROJ 옵션 선택
   - 선택된 설정은 실행 명령어의 변수를 대체하는데 사용됨

3. 작업 디렉토리:
   - 상단에 작업 디렉토리 표시
   - 텍스트 박스와 Apply 버튼으로 수정 가능
   - 변경사항은 설정 파일에 자동 저장

4. 프로그램 실행:
   - 각 프로그램마다 체크박스, LED 상태 표시기, 실행/종료 버튼 제공
   - LED 색상 의미:
     - 초록색: 프로그램 실행 중
     - 어두운 초록색: 프로그램 정지됨
     - 빨간색: 오류 발생
   - 일괄 작업 가능:
     - 'Select All'/'Deselect All': 전체 선택/해제
     - 'Launch Selected': 선택된 프로그램 모두 실행
     - 'Kill Selected': 선택된 프로그램 모두 종료

5. 설정 관리:
   - Open Config: 텍스트 에디터로 설정 파일 열기
   - Load Config: 설정 다시 불러오기 (실행 중인 프로그램 종료 여부 확인)

## 설정 파일

`launch_config.ini` 파일은 다음과 같은 섹션들로 구성됩니다:

```ini
[Default Path]
working_directory = ~/git/AutoHYU-Control

[Environment]
# 환경 변수는 소문자로 정의하지만, 실행 명령어에서는 대문자로 자동 변환되어 사용됨
# 예: map -> ${MAP}, car -> ${CAR}
map = kcity, konkuk, grandpark, speedway, hanyang
car = ailab, hmg
mode = morai, real, carmaker, carla
proj = local_cartesian, utm

[Default Command]
command_1 = source ~/.bashrc
command_2 = source /opt/ros/noetic/setup.bash
command_3 = source devel/setup.bash
command_4 = rosclean purge -y || true

[프로그램 섹션]
# 환경 변수는 대문자로 참조 (예: ${MAP}, ${CAR}, ${MODE}, ${PROJ})
command = roslaunch --screen path/to/launch/file.launch
default_commands = 1,2,3,4  # 사용할 기본 명령어 지정
```

## 주의사항

1. ROS 환경:
   - ROS Noetic이 설치되어 있어야 합니다
   - 프로그램이 자동으로 ROS 설정 파일들을 불러옵니다

2. 프로세스 관리:
   - 각 프로그램은 독립된 터미널 창에서 실행됩니다
   - 프로세스 상태와 오류를 지속적으로 모니터링합니다
   - 프로그램 종료 시 모든 프로세스가 정상적으로 종료됩니다

3. 오류 처리:
   - 오류 메시지는 메시지 박스로 표시됩니다
   - 오류 로그는 `/tmp/launch_manager_[섹션명].err`에 저장됩니다
   - 오류 발생 시 LED 표시기가 빨간색으로 변합니다

4. 기본 명령어:
   - 기본 명령어는 우선순위에 따라 번호가 매겨집니다
   - 각 프로그램마다 필요한 기본 명령어를 지정할 수 있습니다
   - 프로그램 실행 전 환경 정리가 수행됩니다
