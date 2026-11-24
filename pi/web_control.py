#!/usr/bin/env python3
import serial, threading, time, logging, datetime
from dataclasses import dataclass, asdict
from typing import Optional, Callable
from flask import Flask, render_template_string, jsonify, request

SERIAL_PORT = "/dev/ttyUSB0"
BAUD_RATE = 115200
HEARTBEAT_INTERVAL_S = 0.15
RECONNECT_DELAY_S = 1.0
TIME_SYNC_INTERVAL_S = 60
TIMEZONE_OFFSET_HOURS = 3

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
        self.blade_on = False
        self.clock_h = 0
        self.clock_m = 0
    
    def connect(self) -> bool:
        try:
            self.serial = serial.Serial(port=self.port, baudrate=self.baud, timeout=0.1, write_timeout=0.5)
            time.sleep(2.0)
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
    def blade_on(self) -> bool:
        self.blade_on = True
        return self._send_raw("BLD;1")
    def blade_off(self) -> bool:
        self.blade_on = False
        return self._send_raw("BLD;0")
    def servo1(self, angle: int) -> bool: return self._send_raw(f"S1;{angle}")
    def servo2(self, angle: int) -> bool: return self._send_raw(f"S2;{angle}")
    def relay1(self, on: bool) -> bool: return self._send_raw(f"R1;{1 if on else 0}")
    def relay2(self, on: bool) -> bool: return self._send_raw(f"R2;{1 if on else 0}")
    
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

HTML_TEMPLATE = '''
<!DOCTYPE html>
<html lang="ru">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Viking MI 632 P - Управление</title>
    <link href="https://cdn.jsdelivr.net/npm/bootstrap@5.3.0/dist/css/bootstrap.min.css" rel="stylesheet">
    <style>
        body { background: #1a1a1a; color: #fff; font-family: Arial, sans-serif; }
        .status-ready { background: #28a745; color: white; padding: 20px; border-radius: 10px; text-align: center; font-size: 24px; font-weight: bold; }
        .status-safe { background: #dc3545; color: white; padding: 20px; border-radius: 10px; text-align: center; font-size: 24px; font-weight: bold; }
        .sensor-box { background: #333; padding: 15px; border-radius: 8px; margin: 10px 0; }
        .control-btn { margin: 5px; }
        .speed-slider { width: 100%; }
        .card { background: #2a2a2a; border: 1px solid #444; margin: 10px 0; }
        .card-header { background: #333; color: #fff; }
    </style>
</head>
<body>
    <div class="container-fluid p-3">
        <h2 class="text-center mb-4">🤖 Viking MI 632 P</h2>
        
        <div id="statusBox" class="mb-3">
            <div class="status-ready" id="statusText">STATUS: READY</div>
        </div>
        
        <div class="row">
            <div class="col-md-6">
                <div class="card">
                    <div class="card-header">📊 Телеметрия</div>
                    <div class="card-body">
                        <div class="sensor-box">
                            <strong> Батарея:</strong> <span id="battery">--</span> V
                        </div>
                        <div class="sensor-box">
                            <strong>🚧 Бамперы:</strong> Перед: <span id="bumpFront">0</span> | Зад: <span id="bumpRear">0</span>
                        </div>
                        <div class="sensor-box">
                            <strong> Наклон:</strong> Лев: <span id="tiltLeft">0</span> | Прав: <span id="tiltRight">0</span>
                        </div>
                        <div class="sensor-box">
                            <strong>🕐 Время:</strong> <span id="clock">--:--</span>
                        </div>
                        <div class="sensor-box">
                            <strong>️ Скорости:</strong> L: <span id="speedL">0</span> | R: <span id="speedR">0</span>
                        </div>
                        <div class="sensor-box">
                            <strong>🔪 Нож:</strong> <span id="bladeStatus">OFF</span>
                        </div>
                    </div>
                </div>
            </div>
            
            <div class="col-md-6">
                <div class="card">
                    <div class="card-header">🎮 Управление моторами</div>
                    <div class="card-body">
                        <label>Левый мотор: <span id="speedLVal">0</span></label>
                        <input type="range" class="speed-slider" id="sliderL" min="-255" max="255" value="0" oninput="updateSpeed()">
                        <label>Правый мотор: <span id="speedRVal">0</span></label>
                        <input type="range" class="speed-slider" id="sliderR" min="-255" max="255" value="0" oninput="updateSpeed()">
                        <button class="btn btn-danger w-100 mt-2" onclick="stopMotors()">🛑 СТОП</button>
                    </div>
                </div>
                
                <div class="card">
                    <div class="card-header">🔪 Нож</div>
                    <div class="card-body">
                        <button class="btn btn-success w-100" onclick="bladeOn()">ВКЛ</button>
                        <button class="btn btn-danger w-100 mt-2" onclick="bladeOff()">ВЫКЛ</button>
                    </div>
                </div>
                
                <div class="card">
                    <div class="card-header"> Сервы</div>
                    <div class="card-body">
                        <label>Серва 1: <span id="servo1Val">90</span>°</label>
                        <input type="range" class="speed-slider" id="servo1Slider" min="0" max="180" value="90" oninput="updateServo1()">
                        <label>Серва 2: <span id="servo2Val">90</span>°</label>
                        <input type="range" class="speed-slider" id="servo2Slider" min="0" max="180" value="90" oninput="updateServo2()">
                    </div>
                </div>
                
                <div class="card">
                    <div class="card-header">🔌 Реле</div>
                    <div class="card-body">
                        <button class="btn btn-outline-success control-btn" id="relay1Btn" onclick="toggleRelay1()">Реле 1: ВЫКЛ</button>
                        <button class="btn btn-outline-success control-btn" id="relay2Btn" onclick="toggleRelay2()">Реле 2: ВЫКЛ</button>
                    </div>
                </div>
            </div>
        </div>
    </div>
    
    <div class="container mt-3 mb-4">
        <div class="card">
            <div class="card-header">Внешний пульт (D1 mini + энкодер)</div>
            <div class="card-body">
                <div style="font-size:42px;font-weight:bold;">ROT: <span id="remoteRot">0</span></div>
                <div>Событие: <span id="remoteEvent">-</span> &nbsp; Нажатий: <span id="remoteClicks">0</span> &nbsp; <span id="remoteAge">пульт не на связи</span></div>
            </div>
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
            document.getElementById('relay1Btn').innerText = 'Реле 1: ' + (relay1State ? 'ВКЛ' : 'ВЫКЛ');
            document.getElementById('relay1Btn').className = 'btn control-btn ' + (relay1State ? 'btn-success' : 'btn-outline-success');
            fetch('/api/relay1?on=' + relay1State);
        }
        
        function toggleRelay2() {
            relay2State = !relay2State;
            document.getElementById('relay2Btn').innerText = 'Реле 2: ' + (relay2State ? 'ВКЛ' : 'ВЫКЛ');
            document.getElementById('relay2Btn').className = 'btn control-btn ' + (relay2State ? 'btn-success' : 'btn-outline-success');
            fetch('/api/relay2?on=' + relay2State);
        }
        
        function updateStatus() {
            fetch('/api/status')
                .then(r => r.json())
                .then(data => {
                    let statusBox = document.getElementById('statusBox');
                    let statusText = document.getElementById('statusText');
                    if (data.system_safe) {
                        statusBox.innerHTML = '<div class="status-ready">STATUS: READY</div>';
                    } else {
                        statusBox.innerHTML = '<div class="status-safe">STATUS: SAFE - ' + data.safe_reason + '</div>';
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
</html>
'''

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
        'blade_on': link.blade_on
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
