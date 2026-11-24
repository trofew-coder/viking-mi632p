# Протокол Pi ↔ Mega (115200, текст, поля через «;»)
Pi → Mega: `HB`, `DRV;L;R`, `BLD;0/1`, `S1;угол`, `S2;угол`, `R1;0/1`, `R2;0/1`, `TIME;чч;мм`, `ROT;n`, `BTN`, `RST`
Mega → Pi: `ACK;...`, `SENS;bF;bR;tL;tR;mV` (каждые 200 мс), `SAFE;причина`, `ERR;код`
Особенности: Mega перезагружается при открытии порта (DTR) и не имеет часов, время шлёт Pi.
Пульт → Pi: `POST /remote` JSON `{"rot":N,"event":"ROTATE|CLICK|LONG_PRESS"}`, состояние `GET /remote/state`.
