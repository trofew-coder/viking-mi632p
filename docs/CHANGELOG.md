# Журнал работ

## 2026-11-27 — Протокол v3: $CMD*CS XOR, UART /dev/ttyAMA0

### Изменения
- **PROTOCOL.md**: формат кадра `$CMD;ARGS*CS\n` с XOR-контрольной суммой, протокол v3.
- **firmware/mega2560/src/main.cpp**: парсер `$CMD*CS`, проверка XOR, игнор мусора, HB_TIMEOUT = 2000ms, SENS 5000ms, 115200 baud.
- **pi/web_control.py**: порт `/dev/ttyAMA0`, функция `_xor_checksum()`, HB каждые 250ms.
- **docs/PINOUT.md**: UART на GPIO 14/15 + делитель 1к+2к (Mega TX3 5В → Pi RX 3.3В).
- **docs/PROTOCOL.md**: обновлён до v3, добавлен формат `$CMD*CS`.
- **docs/TEAM_LOG.md**: первая запись о переходе на протокол v3.

### Убрано
- PL2303 USB-UART мост — заменён на прямой `/dev/ttyAMA0`.
- `firmware/uno/` — порт под Uno больше не нужен (прямой UART).

### Требуется на Pi
- Отключить Bluetooth: `dtoverlay=disable-bt` в `/boot/config.txt`
- `sudo systemctl stop serial-getty@serial.service`
- Подключить делитель 1к+2к между Pi GPIO14/15 и Mega Serial3 (пины 14/15).

---

## 2026-11-24 — Сборка и прошивка Mega 2560

- Локальная прошивка `firmware/mega2560` собрана PlatformIO для `megaatmega2560`.
- На Raspberry Pi 5 плата обнаружена на `/dev/ttyUSB0`; загрузчик подтвердил ATmega2560 (signature `0x1e9801`).
- Загружено 13 982 байта; проверка flash-памяти прошла успешно.
- Прошита локальная версия `main.cpp` с реле на пинах 9/10 и сервоприводами на 8/53.
- `viking.service` временно остановлен для освобождения порта и после прошивки запущен снова; сервис активен.
- PlatformIO 6.2.0 установлен в `/home/igor/.venvs/platformio`.
- Прошитая локальная правка не закоммичена и не отправлена в GitHub; checkout на Pi остаётся на GitHub `main` (`51e4e21`). Проверена загрузка прошивки, но не поведение косилки под нагрузкой.

## 2026-11-24 — Обновление веб-интерфейса на Raspberry Pi

- Raspberry Pi синхронизирован с GitHub `main` (`028287d`); обновлён `pi/web_control.py`.
- Запущенная копия `/home/igor/web_control.py` обновлена из GitHub; `viking.service` перезапущен.
- Проверены HTTP-ответы `200` для `/` и `/api/status`; SHA-256 копии в репозитории и запущенного файла совпадает.
- Новый интерфейс открыт в браузере по адресу `http://192.168.2.46:5000/`. Команды управления не отправлялись; на момент проверки отображались `SAFE / HEARTBEAT_LOST`, скорости 0 и нож `OFF`.
