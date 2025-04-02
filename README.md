# 개발 환경
- Ubuntu 22.04 LTS, ESP-IDF v5.5.0
- clone 시 vscode + esp-idf extention + docker 설치 필요

## 🔌 USB 디바이스 사용 가이드 (ESP32)

### 🪟 Windows (WSL2) 환경

#### 1. usbipd-win 설치

Windows에서는 [usbipd-win](https://github.com/dorssel/usbipd-win) 도구를 설치해야 합니다.

- 👉 [Releases 페이지](https://github.com/dorssel/usbipd-win/releases)에서 최신 `.msi` 설치 파일 다운로드 및 설치

#### 2. USB 장치 목록 확인

ESP32 보드를 연결한 후 PowerShell에서 다음 명령을 실행:

```powershell
usbipd list
```

예시 출력:

```
BUSID  VID:PID    DEVICE
2-1    10C4:EA60  Silicon Labs CP210x UART Bridge
```

#### 3. WSL에 장치 attach

```powershell
usbipd wsl attach --busid 2-1
```

> `--distribution` 옵션을 통해 특정 WSL 배포판에 연결할 수 있습니다.

```powershell
usbipd wsl attach --busid 2-1 --distribution Ubuntu-22.04
```

#### 4. 자동 연결 설정 (선택)

ESP32 보드를 연결할 때마다 자동으로 WSL에 attach 되게 하려면 다음 명령을 실행:

```powershell
usbipd wsl attach --busid 2-1 --auto-attach
```

> 이후 동일한 보드를 연결하면 자동으로 attach 됩니다.  

> 자동 연결을 해제하려면:

```powershell
usbipd wsl detach --busid 2-1 --auto-attach
```

#### 5. Dev Container에 USB 전달

`.devcontainer/devcontainer.json` 파일에 다음 내용을 포함해야 합니다:

```json
"runArgs": [
  "--privileged",
  "--device=/dev/ttyUSB0"
]
```

그 후 Dev Container를 **재시작(build)** 하세요.

#### 6. 연결 확인

Dev Container 내부 터미널에서 다음 명령으로 장치가 연결되었는지 확인:

```bash
ls -l /dev/ttyUSB*
```

---

### 🐧 Ubuntu (Native Docker) 환경

#### 1. USB 장치 확인

보드를 연결한 후 다음 명령 실행:

```bash
ls -l /dev/ttyUSB*
```

예시:

```
crw-rw---- 1 root dialout 188, 0 Apr 3 14:21 /dev/ttyUSB0
```

#### 2. 사용자 dialout 그룹 추가

현재 사용자가 USB 장치에 접근할 수 있도록 `dialout` 그룹에 추가:

```bash
sudo usermod -aG dialout $USER
```

> 로그아웃 후 재로그인 필요

#### 3. Dev Container 설정

`.devcontainer/devcontainer.json` 파일에 다음을 추가:

```json
"runArgs": [
  "--privileged",
  "--device=/dev/ttyUSB0"
]
```

이후 Dev Container를 재시작하면 `/dev/ttyUSB0`가 정상 연결됩니다.

---


# SDK 구조

```
├── lib
│   ├── liboperation
│   ├── librelay
│   ├── libsensor
│   ├── libserver
│   ├── libutil
│   ├── libtc
├── CMakeLists.txt
├── main
│   ├── CMakeLists.txt
│   └── main.c
└── README.md
```

### liboperation
- FreeRTOS 작동에 관련한 함수 모음
- 각 task에서 실행되는 함수들이 포함
- mode 관련 함수들 포함

### librelay
- 릴레이 제어 관련 함수 모음

### libsensor
- 센서관련 함수 모음
- 진동센서, 온도센서, 전류센서 등이 포함

### libserver
- ASW IoT core MQTT 를 사용한 서버와의 통신 관련 함수 모음

### libutil
- 여러 계산함수, 사전 정의 등이 포함 (ex. CRC 계산함수, error 타입 정의 등)

### libtc
- temperature control (TC) 의 버튼 입력 및 세그먼트 디스플레이 제어  관련된 함수 모음

## Task 종류
### server_task
- ASW IoT core MQTT 를 사용한 서버와의 통신 담당 task
- sensor_task 에서 얻은 센서 데이터들과 릴레이 제어 값을 서버로 전송
- 서버에서 오는 릴레이 제어 값 control_relay_task에 전달
- 서버에서 오는 각종 커맨드 알맞은 task에 전달

### operation_task
- 현재 모드 체크 및 모드 전환 판단 task
- 모드 전환 트리거
    - UART 콘솔 명령
    - GPIO 핀 입력: 전력 이상 감지
    - 서버에서 명령 수신
    - 센서 상태 판단 결과

### tc_task
- temperature control (TC) 의 버튼 입력 및 세그먼트 디스플레이 제어 담당 task
- tc의 버튼 (총 4개)을 사용자가 입력하면 입력을 순차적으로 기억하고, 버튼 입력이 끝나면 해당 동작을 수행
- 예를 들어, 사용자가 온도 세팅을 다시 하면 해당 온도에 맞게 설정
- 인터럽트 감시 등의 역할 수행

### control_relay_task
- 릴레이 제어 task
- 온도 센서 값에 따라 릴레이 제어
- server_task에서 받은 릴레이 제어 값에 따라 릴레이 제어

### sensor_task
- 센서값 획득, 제어, 처리 task
- 온도 센서 값 획득 (1초 주기)
- 진동 센서
    - 컴프레서 릴레이가 on/off 2번 반복될 때마다 한 번
    - ffc 수행 (컴프레서 릴레이 on/off 나눠서)
- 전류 센서 값 획득 (5초 주기)

### consol_task
- mcu 설정 변경, 디버깅, 커맨드 입력 등을 위한 consol task

## Operation Mode
### Debug Mode
- AS 혹은 설정 변경 시 필요
- uart 혹은 usb 연결 시 해당 모드로 전환
- consol_task 활성화

### Default Mode
- 기본 동작 모드
- consol_task 비활성화

### Safe Mode
- 전력 이상(끊김, 불안정) 또는 시스템 문제상황 발생 시 해당 모드로 전환
- 전력 이상(끊김, 불안정) 시
    - server_task: 이상 상황과 현재 냉장고 내부 온도만 전송
    - tc_task: tc 리셋버튼, 디스플레이에 이상상황 발생 출력만 수행
    - control_relay_task: 온도 센서 값에 따라 릴레이 제어만 수행
    - sensor_task: 냉장고 내부 온도 센서만 작동
    - consol_task: 비활성화
- 각종 장치 이상 시
    - 상황에 맞게 동작 필요 (미정)