#!/usr/bin/env python3
import serial, threading, time, logging, datetime
from dataclasses import dataclass, asdict
from typing import Optional, Callable
from flask import Flask, render_template_string, jsonify, request

# Порт — PL2303 USB-UART адаптер к Mega Serial3 (пины 14 TX3 / 15 RX3)
SERIAL_PORT = "/dev/ttyUSB1"
BAUD_RATE = 57600 # PL2303 115200 мусорит, 9600 не работает — 57600 компромисс
HEARTBEAT_INTERVAL_S = 1.0 # PL2303 не успевает на 0.15s
RECONNECT_DELAY_S = 1.0
TIME_SYNC_INTERVAL_S = 60
TIMEZONE_OFFSET_HOURS = 3

# FIX: DS_DTR=False чтобы тесты не перезагружали Mega (CH340 дергает DTR)
SERIAL_INIT_DELAY_S = 2.2  # задержка после открытия Serial для загрузчика Arduino

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
        self._lock = threading.Lock()
        self._on_sensors: Optional[Callable[[Sensors], None]] = None
        self._on_safety: Optional[Callable[[str], None]] = None
        self.system_safe = True
        self.safe_reason = ""
        self.current_speed_l = 0
        self.current_speed_r = 0
        self._bladeOn = False  # ФИКС: флаг отделён от метода blade_on()
        self.clock_h = 0
        self.clock_m = 0

    def connect(self) -> bool:
        try:
            # dsrdtr=False — не сбрасывать Mega при открытии порта (CH340 DTR reset)
            self.serial = serial.Serial(
                port=self.port, baudrate=self.baud, timeout=0.1, write_timeout=0.5,
                dsrdtr=False
            )
            # Задержка для CH340/Arduino: при открытии порта DTR reset → Mega перезагружается
            # Первые 2 секунды загрузчик глушит UART, нужно ждать
            time.sleep(SERIAL_INIT_DELAY_S)
            self.serial.reset_input_buffer()
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
        with self._lock:
            if not self._ensure_connected(): return False
            try:
                self.serial.write((line + "\n").encode("utf-8"))
                return True
            except serial.SerialException as e:
                log.error(f"Ошибка записи: {e}")
                self.serial.close()
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

    def servo1(self, angle: int) -> bool: return self._send_raw(f"S1;{angle}")
    def servo2(self, angle: int) -> bool: return self._send_raw(f"S2;{angle}")
    def relay1(self, on: bool) -> bool: return self._send_raw(f"R1;{1 if on else 0}")
    def relay2(self, on: bool) -> bool: return self._send_raw(f"R2;{1 if on else 0}")

    def esc(self, us_left: int, us_right: int) -> bool:
        """Ручная калибровка ESC: us_left, us_right в диапазоне 1000-2000"""
        return self._send_raw(f"ESC;{us_left};{us_right}")

    def _send_time(self):
        now_utc = datetime.datetime.now(datetime.timezone.utc)
        now_msk = now_utc + datetime.timedelta(hours=TIMEZONE_OFFSET_HOURS)
        cmd = f"TIME;{now_msk.hour:02d};{now_msk.minute:02d}"
        self._send_raw(cmd)

    def _parse_line(self, line: str):
        parts = line.split(";")
        if not parts: return
        cmd = parts[0]
        if cmd == "SENS" and len(parts) >= 6:
            self.sensors.bump_front = parts[1] == "1"
            self.sensors.bump_rear = parts[2] == "1"
            self.sensors.tilt_left = parts[3] == "1"
            self.sensors.tilt_right = parts[4] == "1"
            try: self.sensors.battery_mv = int(parts[5])
            except ValueError: pass
            if self._on_sensors: self._on_sensors(self.sensors)
        elif cmd == "ACK":
            log.debug(f"ACK: {parts[1] if len(parts) > 1 else '?'}")
        elif cmd == "SAFE":
            reason = parts[1] if len(parts) > 1 else "unknown"
            self.system_safe = False
            self.safe_reason = reason
            log.warning(f"АВАРИЙНЫЙ СТОП на Arduino: {reason}")
            if self._on_safety: self._on_safety(reason)
        elif cmd == "ERR":
            log.error(f"Arduino error: {line}")

    def _rx_loop(self):
        while self._running:
            if not self._ensure_connected():
                time.sleep(RECONNECT_DELAY_S)
                continue
            try:
                if self.serial.in_waiting > 0:
                    raw = self.serial.readline()
                    if raw:
                        line = raw.decode("utf-8", errors="ignore").strip()
                        if line: self._parse_line(line)
                else: time.sleep(0.01)
            except serial.SerialException:
                log.warning("Serial разорван при чтении")
                try: self.serial.close()
                except Exception: pass

    def _heartbeat_loop(self):
        while self._running:
            self._send_raw("HB")
            time.sleep(HEARTBEAT_INTERVAL_S)

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

    def on_sensors(self, cb): self._on_sensors = cb
    def on_safety(self, cb): self._on_safety = cb

app = Flask(__name__)
link = RobotLink()

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
.topbar { display: flex; align-items: center; gap: 14px; background: #1c1f24; border: 1px solid #2a2e35; border-radius: 8px; padding: 8px 12px; }
.brand { font-size: 20px; font-weight: 700; letter-spacing: 1px; white-space: nowrap; }
.brand .hl { color: #4fc3f7; }
#statusBox { flex: 1; }
.status-ready, .status-safe { padding: 6px 14px; border-radius: 6px; font-weight: 700; font-size: 15px; letter-spacing: 1px; }
.status-ready { background: #1b5e20; color: #b9f6ca; }
.status-safe { background: #b71c1c; color: #ffcdd2; }
.top-small { font-size: 14px; color: #90a4ae; white-space: nowrap; }
.main { display: grid; grid-template-columns: 290px 1fr 350px; gap: 8px; min-height: 0; }
.panel { background: #1c1f24; border: 1px solid #2a2e35; border-radius: 8px; padding: 8px 12px; display: flex; flex-direction: column; }
.panel h3 { font-size: 12px; text-transform: uppercase; letter-spacing: 1px; color: #7c8794; border-bottom: 1px solid #2a2e35; padding-bottom: 4px; margin-bottom: 8px; }
.tele-row { display: flex; justify-content: space-between; padding: 4px 0; font-size: 14px; border-bottom: 1px dashed #282c33; }
.tele-row .k { color: #9aa4af; }
.tele-row .v { font-weight: 600; }
.motor-block { margin-bottom: 10px; }
.motor-head { display: flex; justify-content: space-between; font-size: 14px; margin-bottom: 2px; }
.motor-head b { color: #4fc3f7; }
.speed-slider { width: 100%; accent-color: #4fc3f7; height: 26px; cursor: pointer; }
.stop-wrap { display: flex; gap: 8px; margin-top: 4px; }
.btn-stop { flex: 1; background: #c62828; color: #fff; border: none; border-radius: 8px; font-size: 22px; font-weight: 800; padding: 12px; cursor: pointer; letter-spacing: 2px; }
.btn-stop:hover { background: #b71c1c; }
.aux-block { margin-bottom: 10px; }
.btn-pair { display: grid; grid-template-columns: 1fr 1fr; gap: 6px; }
.btn { border-radius: 6px; border: none; font-size: 14px; font-weight: 700; padding: 8px; cursor: pointer; }
.btn-on { background: #1b5e20; color: #b9f6ca; }
.btn-off { background: #b71c1c; color: #ffcdd2; }
.btn-out { background: #1c1f24; border: 1px solid #4fc3f7; color: #4fc3f7; }
.btn-out.active { background: #0d47a1; color: #fff; }
.esc-block { margin-bottom: 10px; }
.esc-row { display: flex; gap: 8px; align-items: center; margin-bottom: 4px; }
.esc-input { width: 60px; background: #131519; border: 1px solid #2a2e35; color: #4fc3f7; border-radius: 4px; padding: 4px; text-align: center; font-size: 14px; }
.esc-label { font-size: 12px; color: #90a4ae; min-width: 60px; }
.esc-apply { background: #0d47a1; color: #fff; border: none; border-radius: 4px; padding: 4px 10px; font-size: 12px; cursor: pointer; }
.remotebar { background: #1c1f24; border: 1px solid #2a2e35; border-radius: 8px; padding: 6px 12px; display: flex; align-items: center; gap: 18px; font-size: 14px; }
.remotebar .rot { font-size: 22px; font-weight: 800; color: #ffb74d; margin-left: 4px; }
.remotebar .dim { color: #90a4ae; }
</style>
</head>
<body>
<div class="app">
  <div class="topbar">
    <div class="brand">🤖 Viking <span class="hl">MI 632 P</span></div>
    <div id="statusBox"><div class="status-ready" id="statusText">STATUS: READY</div></div>
    <div class="top-small" id="clockTop">🕐 <span id="clock">--:--</span></div>
    <div class="top-small">🎮 Мост 1920x1080</div>
  </div>
  <div class="main">
    <div class="panel">
      <h3>📊 Телеметрия</h3>
      <div class="tele-row"><span class="k">Батарея</span><span class="v" id="battery">--</span><span class="v">V</span></div>
      <div class="tele-row"><span class="k">Бамперы</span><span class="v">Перед: <span id="bumpFront">0</span> · Зад: <span id="bumpRear">0</span></span></div>
      <div class="tele-row"><span class="k">Наклон</span><span class="v">Лев: <span id="tiltLeft">0</span> · Прав: <span id="tiltRight">0</span></span></div>
      <div class="tele-row"><span class="k">Скорости</span><span class="v">L: <span id="speedL">0</span> · R: <span id="speedR">0</span></span></div>
      <div class="tele-row"><span class="k">Нож</span><span class="v" id="bladeStatus">OFF</span></div>
    </div>
    <div class="panel">
      <h3>🎮 Управление моторами</h3>
      <div class="motor-block">
        <div class="motor-head"><span>Левый мотор</span><b id="speedLVal">0</b></div>
        <input type="range" class="speed-slider" id="sliderL" min="-255" max="255" value="0" oninput="updateSpeed()">
      </div>
      <div class="motor-block">
        <div class="motor-head"><span>Правый мотор</span><b id="speedRVal">0</b></div>
        <input type="range" class="speed-slider" id="sliderR" min="-255" max="255" value="0" oninput="updateSpeed()">
      </div>
      <div class="stop-wrap">
        <button class="btn-stop" onclick="stopMotors()">🛑 СТОП</button>
      </div>
    </div>
    <div class="panel">
      <div class="aux-block">
        <h3>🔪 Нож</h3>
        <div class="btn-pair">
          <button class="btn btn-on" onclick="bladeOn()">ВКЛ</button>
          <button class="btn btn-off" onclick="bladeOff()">ВЫКЛ</button>
        </div>
      </div>
      <div class="esc-block">
        <h3>⚡ ESC калибровка</h3>
        <div class="esc-row">
          <span class="esc-label">Левый:</span>
          <input type="number" class="esc-input" id="escL" value="1500" min="1000" max="2000">
          <input type="number" class="esc-input" id="escR" value="1500" min="1000" max="2000">
          <span class="esc-label">Правый:</span>
          <button class="esc-apply" onclick="applyESC()">Применить</button>
        </div>
        <div style="font-size:11px;color:#90a4ae;">Нейтраль=1500, макс назад=1000, макс вперёд=2000</div>
      </div>
      <div class="aux-block">
        <h3>🛠 Сервы</h3>
        <div class="servo-row">
          <div class="servo-head"><span>Серва 1</span><b id="servo1Val">90</b>°</div>
          <input type="range" class="speed-slider" id="servo1Slider" min="0" max="180" value="90" oninput="updateServo1()">
        </div>
        <div class="servo-row">
          <div class="servo-head"><span>Серва 2</span><b id="servo2Val">90</b>°</div>
          <input type="range" class="speed-slider" id="servo2Slider" min="0" max="180" value="90" oninput="updateServo2()">
        </div>
      </div>
      <div class="aux-block">
        <h3>🔌 Реле</h3>
        <div class="btn-pair">
          <button class="btn btn-out" id="relay1Btn" onclick="toggleRelay1()">Реле 1: ВЫКЛ</button>
          <button class="btn btn-out" id="relay2Btn" onclick="toggleRelay2()">Реле 2: ВЫКЛ</button>
        </div>
      </div>
    </div>
  </div>
  <div class="remotebar">
    <span>Внешний пульт (D1 mini + энкодер)</span>
    <span>ROT: <span class="rot" id="remoteRot">0</span></span>
    <span class="dim">Событие: <span id="remoteEvent">-</span></span>
    <span class="dim">Нажатий: <span id="remoteClicks">0</span></span>
    <span class="dim" id="remoteAge">пульт не на связи</span>
  </div>
</div>
<script>
let relay1State = false;
let relay2State = false;
function updateSpeed() {
  let l = document.getElementById('sliderL').value;
  let r = document.getElementById('sliderR').value;
  document.getElementById('speedLVal').innerText = l;
  document.getElementById('speedRVal').innerText = r;
  fetch('/api/drive?l=' + l + '&r=' + r);
}
function stopMotors() {
  document.getElementById('sliderL').value = 0;
  document.getElementById('sliderR').value = 0;
  document.getElementById('speedLVal').innerText = 0;
  document.getElementById('speedRVal').innerText = 0;
  fetch('/api/stop');
}
function bladeOn() { fetch('/api/blade/on'); }
function bladeOff() { fetch('/api/blade/off'); }
function applyESC() {
  let l = document.getElementById('escL').value;
  let r = document.getElementById('escR').value;
  fetch('/api/esc?l=' + l + '&r=' + r);
}
function updateServo1() {
  let val = document.getElementById('servo1Slider').value;
  document.getElementById('servo1Val').innerText = val;
  fetch('/api/servo1?angle=' + val);
}
function updateServo2() {
  let val = document.getElementById('servo2Slider').value;
  document.getElementById('servo2Val').innerText = val;
  fetch('/api/servo2?angle=' + val);
}
function toggleRelay1() {
  relay1State = !relay1State;
  let b = document.getElementById('relay1Btn');
  b.innerText = 'Реле 1: ' + (relay1State ? 'ВКЛ' : 'ВЫКЛ');
  b.className = 'btn ' + (relay1State ? 'btn-out active' : 'btn-out');
  fetch('/api/relay1?on=' + relay1State);
}
function toggleRelay2() {
  relay2State = !relay2State;
  let b = document.getElementById('relay2Btn');
  b.innerText = 'Реле 2: ' + (relay2State ? 'ВКЛ' : 'ВЫКЛ');
  b.className = 'btn ' + (relay2State ? 'btn-out active' : 'btn-out');
  fetch('/api/relay2?on=' + relay2State);
}
function updateStatus() {
  fetch('/api/status')
  .then(r => r.json())
  .then(data => {
    let statusText = document.getElementById('statusText');
    if (data.system_safe) {
      statusText.className = 'status-ready';
      statusText.textContent = 'STATUS: READY';
    } else {
      statusText.className = 'status-safe';
      statusText.textContent = 'STATUS: SAFE - ' + data.safe_reason;
    }
    document.getElementById('battery').innerText = data.battery_v.toFixed(1);
    document.getElementById('bumpFront').innerText = data.bump_front ? '1' : '0';
    document.getElementById('bumpRear').innerText = data.bump_rear ? '1' : '0';
    document.getElementById('tiltLeft').innerText = data.tilt_left ? '1' : '0';
    document.getElementById('tiltRight').innerText = data.tilt_right ? '1' : '0';
    document.getElementById('clock').innerText = data.clock;
    document.getElementById('speedL').innerText = data.speed_l;
    document.getElementById('speedR').innerText = data.speed_r;
    document.getElementById('bladeStatus').innerText = data.blade_on ? 'ON' : 'OFF';
  });
}
setInterval(updateStatus, 1000);
function updateRemote() {
  fetch('/remote/state').then(r => r.json()).then(d => {
    document.getElementById('remoteRot').innerText = d.rot;
    document.getElementById('remoteEvent').innerText = d.event;
    document.getElementById('remoteClicks').innerText = d.clicks;
    document.getElementById('remoteAge').innerText = d.age < 0 ? 'пульт ещё не отвечал' : ('обновлено ' + d.age.toFixed(1) + ' с назад');
  }).catch(() => {});
}
setInterval(updateRemote, 500);
updateRemote();
updateStatus();
</script>
</body>
</html>'''

@app.route('/')
def index():
    return render_template_string(HTML_TEMPLATE)

@app.route('/api/status')
def status():
    return jsonify({
        'system_safe': link.system_safe,
        'safe_reason': link.safe_reason,
        'battery_v': link.sensors.battery_v,
        'bump_front': link.sensors.bump_front,
        'bump_rear': link.sensors.bump_rear,
        'tilt_left': link.sensors.tilt_left,
        'tilt_right': link.sensors.tilt_right,
        'clock': f"{link.clock_h:02d}:{link.clock_m:02d}",
        'speed_l': link.current_speed_l,
        'speed_r': link.current_speed_r,
        'blade_on': link._bladeOn  # ФИКС: читаем приватный флаг
    })

@app.route('/api/drive')
def drive():
    l = int(request.args.get('l', 0))
    r = int(request.args.get('r', 0))
    link.drive(l, r)
    return jsonify({'ok': True})

@app.route('/api/stop')
def stop():
    link.stop()
    return jsonify({'ok': True})

@app.route('/api/blade/on')
def blade_on():
    link.blade_on()
    return jsonify({'ok': True})

@app.route('/api/blade/off')
def blade_off():
    link.blade_off()
    return jsonify({'ok': True})

@app.route('/api/servo1')
def servo1():
    angle = int(request.args.get('angle', 90))
    link.servo1(angle)
    return jsonify({'ok': True})

@app.route('/api/servo2')
def servo2():
    angle = int(request.args.get('angle', 90))
    link.servo2(angle)
    return jsonify({'ok': True})

@app.route('/api/relay1')
def relay1():
    on = request.args.get('on', 'false').lower() == 'true'
    link.relay1(on)
    return jsonify({'ok': True})

@app.route('/api/relay2')
def relay2():
    on = request.args.get('on', 'false').lower() == 'true'
    link.relay2(on)
    return jsonify({'ok': True})

@app.route('/api/esc')
def esc():
    l = int(request.args.get('l', 1500))
    r = int(request.args.get('r', 1500))
    link.esc(l, r)
    return jsonify({'ok': True})

remote_state = {'rot': 0, 'event': 'NONE', 'last_at': 0.0, 'clicks': 0}

@app.route('/remote', methods=['POST'])
def remote_post():
    data = request.get_json(silent=True) or {}
    try:
        rot = int(data.get('rot', 0))
    except (TypeError, ValueError):
        rot = 0
    event = str(data.get('event', 'ROTATE'))[:16]
    remote_state['rot'] = rot
    remote_state['event'] = event
    remote_state['last_at'] = time.time()
    if event == 'CLICK':
        remote_state['clicks'] += 1
        link._send_raw('BTN')
    elif event == 'LONG_PRESS':
        link._send_raw('RST')
    else:
        link._send_raw('ROT;%d' % rot)
    return jsonify({'ok': True})

@app.route('/remote/state')
def remote_state_get():
    age = (time.time() - remote_state['last_at']) if remote_state['last_at'] else -1
    return jsonify({'rot': remote_state['rot'], 'event': remote_state['event'],
                    'clicks': remote_state['clicks'], 'last_at': remote_state['last_at'], 'age': age})

if __name__ == '__main__':
    try:
        link.start()
        print("Веб-интерфейс запущен: http://192.168.2.46:5000")
        app.run(host='0.0.0.0', port=5000, debug=False)
    except KeyboardInterrupt:
        print("\nОстановка...")
    finally:
        link.stop_all()
