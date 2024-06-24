# CO2-Watchdog-Notification-System

CO2 Watchdog Notification System은 CO2와 소리 수준을 실시간으로 모니터링하고 이를 시각적으로 표현하며, CO2 데이터를 블루투스를 통해 원격으로 전송하는 시스템입니다. 이 시스템은 nRF52840 DK 보드를 기반으로 개발되었으며, Zephyr RTOS를 사용합니다.

## 주요 기능
- CO2 수준 모니터링 (UART)
- 소리 수준 모니터링 (ADC)
- CO2 수준 시각화 (배터리 디스플레이)
- 소리 수준 시각화 (도트 매트릭스 LED)
- CO2 데이터 원격 전송 (Bluetooth)
- 시스템 초기화 (버튼 입력)

## 요구사항
- nRF52840 DK 보드
- Rich Shield
    - CO2 센서
    - 소리 센서
    - 배터리 디스플레이
    - 도트 매트릭스 LED
- Zephyr RTOS
- nRF Connect SDK
- Visual Studio Code (추천)
- 스마트폰 (Bluetooth 기능 필요)
- nRF Connect for Mobile 앱 (Android/iOS)


## 설정 및 빌드
1. **nRF Connect SDK 및 VS Code 확장팩 설치**
    - [nRF Connect for VS Code](https://marketplace.visualstudio.com/items?itemName=nordic-semiconductor.nrf-connect) 확장팩을 설치합니다. 이 확장팩을 설치하면 필요한 nRF Connect SDK와 툴체인도 함께 설치됩니다.

2. **프로젝트 클론**
    ```bash
    git clone https://github.com/MinjooShin/CO2-Watchdog-Notification-System.git
    cd CO2-Watchdog-Notification-System
    ```

3. **Visual Studio Code 설정**
    - VS Code에서 `nRF Connect` 확장팩을 사용하여 프로젝트를 열어 빌드합니다.

4. **보드 연결 및 빌드**
    - nRF52840 DK 보드를 연결하고, 프로젝트를 빌드 및 플래시합니다.
    ```bash
    west build -b nrf52840dk_nrf52840
    west flash
    ```

4. **Bluetooth 원격 전송**
    - CO2 데이터는 블루투스를 통해 원격으로 전송됩니다. 연결 상태는 LED로 확인할 수 있습니다.

5. **Bluetooth 앱 사용**
    - 스마트폰에 nRF Connect for Mobile 앱을 설치합니다 (Android/iOS).
    - 앱을 열고, CO2_Watchdog을 검색하여 연결합니다.
    - 연결된 후, UUID "87654321-4321-8765-4321-876543218765"를 찾아 Enable 시킵니다.
    - Enable 되면, CO2 데이터를 실시간으로 모니터링할 수 있습니다. (데이터 파서 설정을 통해 데이터를 편한 형식으로 볼 수 있습니다.)

## 사용법
1. **시스템 초기화**
    - 보드의 버튼1을 눌러 시스템을 초기화합니다. 초기화 후 CO2와 소리 수준을 모니터링할 준비가 됩니다.

2. **CO2 수준 모니터링**
    - CO2 센서를 통해 UART로 데이터를 수신합니다. 수신된 데이터는 배터리 디스플레이에 시각화됩니다.

3. **소리 수준 모니터링**
    - 소리 센서를 통해 ADC로 데이터를 수신합니다. 수신된 데이터는 도트 매트릭스 LED에 동적 그래프로 시각화됩니다.

4. **Bluetooth 원격 전송**
    - CO2 데이터는 블루투스를 통해 원격으로 전송됩니다. 연결 상태는 LED로 확인할 수 있습니다.

## 참고 자료
- [Zephyr Project](https://zephyrproject.org/)
- [nRF Connect SDK](https://developer.nordicsemi.com/nRF_Connect_SDK/doc/latest/nrf/index.html)
- [Visual Studio Code](https://code.visualstudio.com/)
- [nRF Connect for Mobile](https://www.nordicsemi.com/Software-and-tools/Development-Tools/nRF-Connect-for-mobile)
- [Nordic DevAcademy](https://academy.nordicsemi.com/)

## 기여 방법
1. 이 프로젝트를 포크합니다.
2. 새 브랜치를 만듭니다 (`git checkout -b feature/fooBar`).
3. 변경 사항을 커밋합니다 (`git commit -am 'Add some fooBar'`).
4. 브랜치에 푸시합니다 (`git push origin feature/fooBar`).
5. 풀 리퀘스트를 생성합니다.

## 데모
[CO2 Watchdog Notification System 데모 영상](https://youtu.be/EPjfsvNmhLU)

## 문의
- 이 프로젝트와 관련된 문의사항은 [이슈](https://github.com/MinjooShin/CO2-Watchdog-Notification-System/issues)를 통해 남겨주세요.
