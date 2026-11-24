# viking-mi632p
Беспилотная газонокосилка на базе Viking iMow MI 632 P. Raspberry Pi 5 + Arduino Mega 2560. План: RTK-GPS, LiDAR, AI-камера.

## Структура
- `firmware/mega2560/`: прошивка Mega (PlatformIO)
- `firmware/remote_d1mini/`: внешний пульт D1 mini + энкодер
- `pi/`: Python на Raspberry Pi, конфиг, systemd, udev
- `docs/`: протокол, пины, открытые вопросы

Статус: каркас, код пока не перенесён. Пароли и Wi-Fi в репозиторий не кладём.
