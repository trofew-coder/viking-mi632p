# viking-mi632p
Беспилотная газонокосилка на базе Viking iMow MI 632 P. Raspberry Pi 5 + Arduino Mega 2560. План: RTK-GPS, LiDAR, AI-камера.

## Структура
- `firmware/mega2560/`: прошивка Mega (PlatformIO)
- `firmware/remote_d1mini/`: внешний пульт D1 mini + энкодер
- `pi/`: Python на Raspberry Pi, конфиг, systemd, udev
- `docs/`: протокол, пины, открытые вопросы

Статус: код перенесён с Pi (состояние на 21.11.2026). Wi-Fi D1 mini: скопировать `firmware/remote_d1mini/include/secrets.h.example` в `secrets.h` и заполнить. Пароли и Wi-Fi в репозиторий не кладём.
