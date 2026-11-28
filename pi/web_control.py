#!/usr/bin/env python3
"""web_control.py — управление Viking MI 632 P через Flask + прямой UART /dev/ttyAMA0"""
import serial, threading, time, logging, datetime, os
from dataclasses import dataclass, asdict
from typing import Optional, Callable
from flask import Flask, render_template_string, jsonify, request

# Порт — прямой аппаратный UART Pi (GPIO 14 TX / GPIO 15 RX)
# Mega TX3 (5В) → делитель 1к+2к → Pi GPIO15 (RX, 3.3В)
SERIAL_PORT = "/dev/ttyAMA0"
BAUD_RATE = 115200
HEARTBEAT_INTERVAL_S = 0.25  # 250ms — приоритет HB
RECONNECT_DELAY_S = 1.0
TIME_SYNC_INTERVAL_S = 60
TIMEZONE_OFFSET_HOURS = 3
SERIAL_INIT_DELAY_S = 2.2  # задержка после открытия Serial для загрузчика Arduino


def _xor_checksum(text: str) -> str:
    """XOR всех байтов строки, вернуть 2 hex-символа"""
    cs = 0
    for b in text.encode('utf-8'):
        cs ^= b
    return format(cs, '02X')


logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
log = logging.getLogger("RobotLink")


@dataclass
class Sensors:
    bump_front: bool = False
    bump_rear: bool = False
    tilt_left: bool = False
    tilt_right: bool = False
    battery_mv: int = 0

    @property
    def battery_v(self) -> float:
        return self.battery_mv / 1000.0

    def __str__(self):
        return f"Bump[F:{self.bump_front} R:{self.bump_rear}] Tilt[L:{self.tilt_left} R:{self.tilt_right}] BAT:{self.battery_v:.1f}V"


class RobotLink:
    def __init__(self, port: str = SERIAL_PORT, baud: int = BAUD_RATE):
        self.port, self.baud = port, baud
        self.serial: Optional[serial.Serial] = None
        self.sensors = Sensors()
        self._running = False
        self._hb_thread: Optional[threading.Thread] = None
        self._rx_thread: Optional[threading.Thread] = None
        self._time_thread: Optional[threading.Thread] = None
        self._write_lock = threading.Lock()  # Только на write!
        self._on_sensors: Optional[Callable[[Sensors], None]] = None
        self._on_safety: Optional[Callable[[str], None]] = None
        self.system_safe = True
        self.safe_reason = ""
        self.current_speed_l = 0
        self.current_speed_r = 0
        self._bladeOn = False
        self.clock_h = 0
        self.clock_m = 0

    def connect(self) -> bool:
        try:
            # ttyAMA0: timeout=0.05, БЕЗ reset_input_buffer() — бросает OSError
            self.serial = serial.Serial(
                port=self.port, baudrate=self.baud, timeout=0.05, write_timeout=0.1,
                dsrdtr=False
            )
            time.sleep(SERIAL_INIT_DELAY_S)
            log.info(f"Serial открыт: {self.port} @ {self.baud}")
            return True
        except serial.SerialException as e:
            log.warning(f"Не удалось открыть {self.port}: {e}")
            return False

    def disconnect(self):
        self._running = False
        if self.serial and self.serial.is_open:
            try: self.serial.close()
            except Exception: pass

    def _ensure_connected(self) -> bool:
        if self.serial and self.serial.is_open:
            return True
        log.warning("Потеряно соединение, переподключение...")
        self.disconnect()
        for _ in range(5):
            if self.connect(): return True
            time.sleep(RECONNECT_DELAY_S)
        return False

    def _send_raw(self, line: str) -> bool:
        """Отправить команду в протоколе v3: $CMD*CS\n"""
        with self._write_lock:
            if not self._ensure_connected(): return False
            try:
                frame = f"${line}*{_xor_checksum(line)}\n"
                self.serial.write(frame.encode("utf-8"))
                return True
            except serial.SerialException as e:
                log.error(f"Ошибка записи: {e}")
                try: self.serial.close()
                except Exception: pass
                return False

    def drive(self, speed_left: int, speed_right: int) -> bool:
        self.current_speed_l = speed_left
        self.current_speed_r = speed_right
        return self._send_raw(f"DRV;{max(-255, min(255, int(speed_left)))};{max(-255, min(255, int(speed_right)))}")
    def stop(self) -> bool: return self.drive(0, 0)

    def set_blade(self, on: bool) -> bool:
        self._bladeOn = on
        return self._send_raw(f"BLD;{1 if on else 0}")
    def blade_on(self) -> bool: return self.set_blade(True)
    def blade_off(self) -> bool: return self.set_blade(False)

    def on_sensors(self, cb): self._on_sensors = cb
    def on_safety(self, cb): self._on_safety = cb

    def emergency_stop(self, reason: str):
        log.warning(f"АВАРИЙНЫЙ СТОП на Arduino: {reason}")
        if self._on_safety: self._on_safety(reason)
        self.system_safe = False
        self.safe_reason = reason
        self.stop()
        self.blade_off()

    def _parse_line(self, line: str):
        # Протокол v3: ответ Mega без $ и *CS (только команды с маркером)
        cmd = line.strip()
        if not cmd: return

        if cmd.startswith("SENS;"):
            parts = cmd.split(';')
            if len(parts) == 6:
                self.sensors.bump_front = int(parts[1]) == 1
                self.sensors.bump_rear = int(parts[2]) == 1
                self.sensors.tilt_left = int(parts[3]) == 1
                self.sensors.tilt_right = int(parts[4]) == 1
                self.sensors.battery_mv = int(parts[5])
                if self._on_sensors: self._on_sensors(self.sensors)
        elif cmd.startswith("SAFE;"):
            parts = cmd.split(';')
            reason = parts[1] if len(parts) > 1 else "UNKNOWN"
            self.emergency_stop(reason)
        elif cmd.startswith("ACK;"):
            pass  # Подтверждение команды

    def _rx_loop(self):
        """Чтение UART без Lock — readline с timeout=0.05"""
        while self._running:
            try:
                if self.serial and self.serial.is_open:
                    line = self.serial.readline()
                    if line:
                        text = line.decode("utf-8", errors="ignore").strip()
                        if text: self._parse_line(text)
                else:
                    time.sleep(0.02)
            except Exception:
                time.sleep(0.02)

    def _heartbeat_loop(self):
        while self._running:
            self._send_raw("HB")
            time.sleep(HEARTBEAT_INTERVAL_S)

    def _send_time(self):
        now = datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=TIMEZONE_OFFSET_HOURS)))
        self._send_raw(f"TIME;{now.hour:02d};{now.minute:02d}")

    def _time_loop(self):
        while self._running:
            self._send_time()
            time.sleep(TIME_SYNC_INTERVAL_S)

    def start(self):
        if not self.connect(): raise RuntimeError("Не удалось подключиться к Arduino")
        self._running = True
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._hb_thread = threading.Thread(target=self._heartbeat_loop, daemon=True)
        self._time_thread = threading.Thread(target=self._time_loop, daemon=True)
        self._rx_thread.start()
        self._hb_thread.start()
        self._time_thread.start()
        log.info("Связь с Arduino запущена (heartbeat + rx + time)")

    def stop_all(self):
        self._running = False
        self.blade_off()
        self.stop()
        time.sleep(0.3)
        self.disconnect()


# --- Flask ---
app = Flask(__name__)
link = RobotLink()
remote_state = {"rot": 0, "event": "", "clicks": 0, "last_at": None, "age": -1}


@app.route('/')
def index():
    return render_template_string(HTML_TEMPLATE)


@app.route('/api/status')
def status():
    return jsonify({**asdict(link.sensors),
                    "blade_on": link._bladeOn,
                    "system_safe": link.system_safe,
                    "safe_reason": link.safe_reason,
                    "speed_l": link.current_speed_l,
                    "speed_r": link.current_speed_r,
                    "clock": f"{link.clock_h:02d}:{link.clock_m:02d}"})


@app.route('/api/drive', methods=['POST'])
def drive_post():
    d = request.get_json()
    ok = link.drive(d.get('speed_l', 0), d.get('speed_r', 0))
    return jsonify({"ok": ok})


@app.route('/api/blade', methods=['POST'])
def blade_post():
    d = request.get_json()
    on = d.get('on', False)
    if on: ok = link.blade_on()
    else: ok = link.blade_off()
    return jsonify({"ok": ok})


@app.route('/remote', methods=['POST'])
def remote_post():
    d = request.get_json()
    rot = d.get('rot', 0)
    event = d.get('event', 'ROTATE')
    remote_state['rot'] = rot
    remote_state['event'] = event
    remote_state['last_at'] = time.time()
    if event == 'CLICK': remote_state['clicks'] += 1
    link._send_raw(f"ROT;{rot}" if event == "ROTATE" else event)
    return jsonify({"ok": True})


@app.route('/remote/state')
def remote_state_get():
    age = (time.time() - remote_state['last_at']) if remote_state['last_at'] else -1
    return jsonify({"rot": remote_state['rot'], "event": remote_state['event'],
                    "clicks": remote_state['clicks'], "last_at": remote_state['last_at'], 'age': age})


# --- HTML ---
HTML_TEMPLATE = '''<!DOCTYPE html>
<html lang="ru">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Viking MI 632 P - Управление</title>
<style>
* { box-sizing: border-box; margin: 0; padding: 0; }
html, body { height: 100%; overflow: hidden; }
body { background: #131519; color: #e8eaed; font-family: 'Segoe UI', Arial, sans-serif; }
.app { display: grid; grid-template-rows: auto 1fr auto; gap: 6px; height: 100vh; padding: 6px; }
.header { display: grid; grid-template-columns: 1fr 1fr; gap: 8px; }
.dash, .log { background: #1e2126; border-radius: 10px; padding: 10px 12px; overflow: auto; }
.dash { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; }
.dash-item { font-size: 14px; }
.dash-item .label { font-size: 11px; color: #8b8f96; text-transform: uppercase; }
.dash-item .val { font-size: 20px; font-weight: 600; margin: 2px 0; }
.ok { color: #2ecc71; } .bad { color: #e74c3c; }
.joy { grid-column: 1 / -1; display: grid; grid-template-columns: auto 1fr auto; gap: 8px; align-items: center; }
.joy-btn { background: #2c2f36; border: none; color: #e8eaed; border-radius: 8px; padding: 10px; font-size: 18px; cursor: pointer; }
.joy-btn:active { background: #3d4048; }
.joy-range { width: 100%; accent-color: #3498db; }
.log { font-family: monospace; font-size: 12px; color: #8b8f96; }
.joy-center { text-align: center; padding: 4px; font-size: 12px; color: #8b8f96; }
.joy-slider-group { display: flex; flex-direction: column; gap: 4px; }
.joy-slider-group label { font-size: 10px; color: #8b8f96; }
</style>
</head>
<body>
<div class="app">
  <div class="header">
    <div class="dash" id="dash"></div>
    <div class="log" id="log"></div>
  </div>
  <div class="joy">
    <button class="joy-btn" onmousedown="sendJoy('up')" onmouseup="sendJoy('stop')" ontouchstart="sendJoy('up')" ontouchend="sendJoy('stop')">▲</button>
    <div class="joy-slider-group">
      <label>Газ</label>
      <input type="range" class="joy-range" id="throttle" min="-100" max="100" value="0">
    </div>
    <button class="joy-btn" onmousedown="sendJoy('down')" onmouseup="sendJoy('stop')" ontouchstart="sendJoy('down')" ontouchend="sendJoy('stop')">▼</button>
  </div>
  <div class="joy" style="grid-template-columns: auto 1fr auto;">
    <button class="joy-btn" onmousedown="sendJoy('left')" onmouseup="sendJoy('stop')" ontouchstart="sendJoy('left')" ontouchend="sendJoy('stop')">◀</button>
    <div class="joy-center" id="centerLabel">STOP</div>
    <button class="joy-btn" onmousedown="sendJoy('right')" onmouseup="sendJoy('stop')" ontouchstart="sendJoy('right')" ontouchend="sendJoy('stop')">▶</button>
  </div>
</div>

<script>
const API = '/api';
let logEntries = [];
function updateStatus() {
  fetch(API + '/status').then(r => r.json()).then(d => {
    const dash = document.getElementById('dash');
    const safeCls = d.system_safe ? 'ok' : 'bad';
    dash.innerHTML = `
      <div class="dash-item"><div class="label">Батарея</div><div class="val">${d.battery_v.toFixed(1)}V</div></div>
      <div class="dash-item"><div class="label">Статус</div><div class="val ${safeCls}">${d.system_safe ? 'OK' : d.safe_reason}</div></div>
      <div class="dash-item"><div class="label">Скорость L/R</div><div class="val">${d.speed_l} / ${d.speed_r}</div></div>
      <div class="dash-item"><div class="label">Нож</div><div class="val">${d.blade_on ? 'ON' : 'OFF'}</div></div>
    `;
    const log = document.getElementById('log');
    logEntries.push(`${d.clock} | ${d.speed_l};${d.speed_r} | ${d.system_safe ? 'OK' : d.safe_reason}`);
    if (logEntries.length > 20) logEntries.shift();
    log.innerHTML = logEntries.join('<br>');
    log.scrollTop = log.scrollHeight;
  });
}
function sendJoy(dir) {
  const throttle = parseInt(document.getElementById('throttle').value) || 0;
  let l = 0, r = 0;
  if (dir === 'up') { l = throttle; r = throttle; }
  else if (dir === 'down') { l = -throttle; r = -throttle; }
  else if (dir === 'left') { l = -Math.abs(throttle); r = Math.abs(throttle); }
  else if (dir === 'right') { l = Math.abs(throttle); r = -Math.abs(throttle); }
  else { l = 0; r = 0; }
  document.getElementById('centerLabel').textContent = dir.toUpperCase();
  fetch(API + '/drive', { method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify({speed_l: l, speed_r: r}) });
}
setInterval(updateStatus, 500);
updateStatus();
</script>
</body>
</html>'''


if __name__ == '__main__':
    try:
        link.start()
        print("Веб-интерфейс запущен: http://192.168.2.46:5000")
        app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)
    except KeyboardInterrupt:
        print("\nОстановка...")
    finally:
        link.stop_all()
