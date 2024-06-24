# CO2-Watchdog-Notification-System

CO2 Watchdog Notification System�� CO2�� �Ҹ� ������ �ǽð����� ����͸��ϰ� �̸� �ð������� ǥ���ϸ�, CO2 �����͸� ��������� ���� �������� �����ϴ� �ý����Դϴ�. �� �ý����� nRF52840 DK ���带 ������� ���ߵǾ�����, Zephyr RTOS�� ����մϴ�.

## �ֿ� ���
- CO2 ���� ����͸� (UART)
- �Ҹ� ���� ����͸� (ADC)
- CO2 ���� �ð�ȭ (���͸� ���÷���)
- �Ҹ� ���� �ð�ȭ (��Ʈ ��Ʈ���� LED)
- CO2 ������ ���� ���� (Bluetooth)
- �ý��� �ʱ�ȭ (��ư �Է�)

## �䱸����
- nRF52840 DK ����
- Rich Shield
    - CO2 ����
    - �Ҹ� ����
    - ���͸� ���÷���
    - ��Ʈ ��Ʈ���� LED
- Zephyr RTOS
- nRF Connect SDK
- Visual Studio Code (��õ)
- ����Ʈ�� (Bluetooth ��� �ʿ�)
- nRF Connect for Mobile �� (Android/iOS)


## ���� �� ����
1. **nRF Connect SDK �� VS Code Ȯ���� ��ġ**
    - [nRF Connect for VS Code](https://marketplace.visualstudio.com/items?itemName=nordic-semiconductor.nrf-connect) Ȯ������ ��ġ�մϴ�. �� Ȯ������ ��ġ�ϸ� �ʿ��� nRF Connect SDK�� ��ü�ε� �Բ� ��ġ�˴ϴ�.

2. **������Ʈ Ŭ��**
    ```bash
    git clone https://github.com/MinjooShin/CO2-Watchdog-Notification-System.git
    cd CO2-Watchdog-Notification-System
    ```

3. **Visual Studio Code ����**
    - VS Code���� `nRF Connect` Ȯ������ ����Ͽ� ������Ʈ�� ���� �����մϴ�.

4. **���� ���� �� ����**
    - nRF52840 DK ���带 �����ϰ�, ������Ʈ�� ���� �� �÷����մϴ�.
    ```bash
    west build -b nrf52840dk_nrf52840
    west flash
    ```

4. **Bluetooth ���� ����**
    - CO2 �����ʹ� ��������� ���� �������� ���۵˴ϴ�. ���� ���´� LED�� Ȯ���� �� �ֽ��ϴ�.

5. **Bluetooth �� ���**
    - ����Ʈ���� nRF Connect for Mobile ���� ��ġ�մϴ� (Android/iOS).
    - ���� ����, CO2_Watchdog�� �˻��Ͽ� �����մϴ�.
    - ����� ��, UUID "87654321-4321-8765-4321-876543218765"�� ã�� Enable ��ŵ�ϴ�.
    - Enable �Ǹ�, CO2 �����͸� �ǽð����� ����͸��� �� �ֽ��ϴ�. (������ �ļ� ������ ���� �����͸� ���� �������� �� �� �ֽ��ϴ�.)

## ����
1. **�ý��� �ʱ�ȭ**
    - ������ ��ư1�� ���� �ý����� �ʱ�ȭ�մϴ�. �ʱ�ȭ �� CO2�� �Ҹ� ������ ����͸��� �غ� �˴ϴ�.

2. **CO2 ���� ����͸�**
    - CO2 ������ ���� UART�� �����͸� �����մϴ�. ���ŵ� �����ʹ� ���͸� ���÷��̿� �ð�ȭ�˴ϴ�.

3. **�Ҹ� ���� ����͸�**
    - �Ҹ� ������ ���� ADC�� �����͸� �����մϴ�. ���ŵ� �����ʹ� ��Ʈ ��Ʈ���� LED�� ���� �׷����� �ð�ȭ�˴ϴ�.

4. **Bluetooth ���� ����**
    - CO2 �����ʹ� ��������� ���� �������� ���۵˴ϴ�. ���� ���´� LED�� Ȯ���� �� �ֽ��ϴ�.

## ���� �ڷ�
- [Zephyr Project](https://zephyrproject.org/)
- [nRF Connect SDK](https://developer.nordicsemi.com/nRF_Connect_SDK/doc/latest/nrf/index.html)
- [Visual Studio Code](https://code.visualstudio.com/)
- [nRF Connect for Mobile](https://www.nordicsemi.com/Software-and-tools/Development-Tools/nRF-Connect-for-mobile)
- [Nordic DevAcademy](https://academy.nordicsemi.com/)

## �⿩ ���
1. �� ������Ʈ�� ��ũ�մϴ�.
2. �� �귣ġ�� ����ϴ� (`git checkout -b feature/fooBar`).
3. ���� ������ Ŀ���մϴ� (`git commit -am 'Add some fooBar'`).
4. �귣ġ�� Ǫ���մϴ� (`git push origin feature/fooBar`).
5. Ǯ ������Ʈ�� �����մϴ�.

## ����
[CO2 Watchdog Notification System ���� ����](https://youtu.be/EPjfsvNmhLU)

## ����
- �� ������Ʈ�� ���õ� ���ǻ����� [�̽�](https://github.com/MinjooShin/CO2-Watchdog-Notification-System/issues)�� ���� �����ּ���.
