#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
===============================================================================
GPS-DENIED PRECISION LANDING SYSTEM - INAV MSP RC OVERRIDE
===============================================================================
Target Hardware:
  - Flight Controller : SpeedyBee F722 V3 (INAV 9.0+ firmware)
  - Companion Computer: Raspberry Pi 5 (4GB)
  - Camera            : Global Shutter (IMX296 veya ZW LRCP)
  - Beacon            : 940nm IR Laser

Communication Protocol:
  - MSP V2 (MultiWii Serial Protocol Version 2)
  - MSP_SET_RAW_RC (Function ID: 200)
  - Baudrate: 115200 (UART3)

Control Method:
  - MSP RC Override mode (INAV native support)
  - Pilot switch ile manual/auto geçiş
  - 5Hz altında otomatik failsafe (RC'ye dönüş)

Glide Slope Principle:
  - Kamera drone ile birlikte hareket eder (gimbal YOK)
  - Lazer ekranda aşağıda → Drone ileri gider (pitch down)
  - Drone pitch yaptıkça lazer ekranda yukarı kayar
  - Lazer merkezde → Doğru glide slope açısı

Authors: Ramazan & Claude
Date   : 2025-12-24
Version: 1.0.0
===============================================================================
"""

# =============================================================================
# IMPORTS
# =============================================================================
import time
import threading
import logging
import socket
from collections import deque

# Flask - Web arayüzü için
from flask import Flask, Response, request, jsonify

# Yerel modüller
from config import *
from msp_protocol import MSPProtocol
from laser_detector import LaserDetector
from pid_controller import PIDController
from state_machine import StateMachine, SystemState

# =============================================================================
# LOGGING CONFIGURATION
# =============================================================================
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s | %(levelname)-8s | %(module)-15s | %(message)s',
    datefmt='%H:%M:%S'
)
logger = logging.getLogger(__name__)

# Flask loglarını kapat (çok gürültülü)
flask_log = logging.getLogger('werkzeug')
flask_log.setLevel(logging.ERROR)

# =============================================================================
# FLASK APPLICATION
# =============================================================================
app = Flask(__name__)

# =============================================================================
# GLOBAL INSTANCES
# =============================================================================
# Bu objeler main() içinde initialize edilecek
msp: MSPProtocol = None
detector: LaserDetector = None
pid_x: PIDController = None
pid_y: PIDController = None
state_machine: StateMachine = None

# Thread-safe telemetri verisi
telemetry_lock = threading.Lock()
telemetry_data = {
    'altitude': 0.0,
    'armed': False,
    'mode': 'UNKNOWN',
    'laser_pos': None,
    'laser_detected': False,
    'system_state': 'IDLE',
    'pid_output': (0, 0),
    'rc_channels': [1500] * 8
}

# =============================================================================
# UTILITY FUNCTIONS
# =============================================================================

def get_local_ip() -> str:
    """
    Yerel IP adresini al.
    
    Web arayüzüne erişim için kullanıcıya gösterilecek.
    
    Returns:
        str: IP adresi (örn: '192.168.1.100')
    """
    try:
        # UDP socket ile dış ağa bağlanmaya çalış (gerçekte bağlanmaz)
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(('8.8.8.8', 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return '127.0.0.1'


def print_banner():
    """
    Başlangıç banner'ını yazdır.
    """
    print("=" * 70)
    print("🎯 GPS-DENIED PRECISION LANDING SYSTEM")
    print("   INAV MSP RC Override - v1.0.0")
    print("=" * 70)
    print(f"📡 FC Port     : {FC_UART_PORT} @ {FC_UART_BAUDRATE} baud")
    print(f"📷 Camera      : {CAMERA_RES_X}x{CAMERA_RES_Y} @ {CAMERA_FPS}fps")
    print(f"🔬 FOV         : {CAMERA_FOV_H}° horizontal")
    print(f"🎛️  PID         : Kp={PID_KP}, Ki={PID_KI}, Kd={PID_KD}")
    print(f"🚀 Max Speed   : Horizontal={MAX_HORIZONTAL_SPEED}m/s, "
          f"Descent={MAX_DESCENT_SPEED}m/s")
    print(f"⏱️  Laser Detect: {LASER_DETECTION_TIME}s confirm, "
          f"{LASER_LOST_TIMEOUT}s timeout")
    print(f"📡 MSP Rate    : {MSP_SEND_RATE_HZ}Hz (min 5Hz for failsafe)")
    print("=" * 70)
    print("✅ INAV MSP RC Override (no multiplexer needed)")
    print("✅ Pilot override via AUX switch")
    print("✅ Auto failsafe: MSP <5Hz → RC takeover")
    print("✅ Global shutter camera (no motion blur)")
    print("=" * 70)


def print_ppm_table():
    """
    Yüksekliğe göre piksel/metre tablosunu yazdır.
    
    Lazer tespiti için referans değerler.
    """
    import math
    
    print("\n📊 Piksel/Metre Referans Tablosu:")
    print("-" * 50)
    print(f"{'Yükseklik':>10} | {'Yer Genişliği':>12} | {'PPM':>10} | {'Lazer':>8}")
    print("-" * 50)
    
    for alt in [1, 2, 5, 10, 15, 20]:
        # FOV'dan yer genişliği hesapla
        fov_rad = math.radians(CAMERA_FOV_H)
        ground_width = 2 * alt * math.tan(fov_rad / 2)
        
        # Piksel/metre oranı
        ppm = CAMERA_RES_X / ground_width
        
        # 2cm çaplı lazer noktası kaç piksel
        laser_px = 0.02 * ppm
        
        print(f"{alt:>8}m | {ground_width:>10.1f}m | {ppm:>8.1f} | {laser_px:>6.1f}px")
    
    print("-" * 50)


# =============================================================================
# MAIN CONTROL LOOP
# =============================================================================

def control_loop():
    """
    Ana kontrol döngüsü.
    
    Bu fonksiyon ayrı bir thread'de çalışır ve şunları yapar:
    1. Kameradan frame al
    2. Lazer tespiti yap
    3. State machine'i güncelle
    4. PID hesapla
    5. MSP RC komutları gönder
    
    Döngü hızı: MSP_SEND_RATE_HZ (varsayılan 20Hz = 50ms)
    """
    global telemetry_data
    
    logger.info("Control loop başlatıldı")
    
    # Döngü zamanlaması
    loop_period = 1.0 / MSP_SEND_RATE_HZ  # 20Hz → 0.05s = 50ms
    last_loop_time = time.time()
    
    # PID reset için
    last_laser_time = 0
    
    while True:
        try:
            loop_start = time.time()
            
            # -----------------------------------------------------------------
            # 1. KAMERADAN FRAME AL ve LAZER TESPİT ET
            # -----------------------------------------------------------------
            frame = detector.capture_frame()
            laser_pos, processed_frame = detector.detect_laser(frame)
            
            # Lazer bulundu mu?
            laser_found = laser_pos is not None
            
            # -----------------------------------------------------------------
            # 2. STATE MACHINE GÜNCELLE
            # -----------------------------------------------------------------
            # Telemetriden yükseklik al
            with telemetry_lock:
                current_alt = telemetry_data['altitude']
            
            # State machine'e bildir
            state_machine.update(
                laser_detected=laser_found,
                laser_position=laser_pos,
                altitude=current_alt
            )
            
            current_state = state_machine.get_state()
            
            # -----------------------------------------------------------------
            # 3. PID HESAPLA (sadece TRACKING veya APPROACH durumunda)
            # -----------------------------------------------------------------
            roll_output = 0
            pitch_output = 0
            throttle_output = 0
            
            if current_state in [SystemState.TRACKING, SystemState.APPROACH]:
                if laser_found:
                    # Lazer pozisyonunu normalize et (-1 ile +1 arası)
                    # Ekran merkezi (0,0), sol üst (-1,-1), sağ alt (+1,+1)
                    center_x = CAMERA_RES_X / 2
                    center_y = CAMERA_RES_Y / 2
                    
                    # Hata hesapla (lazer merkeze ne kadar uzak)
                    error_x = (laser_pos[0] - center_x) / center_x  # -1 to +1
                    error_y = (laser_pos[1] - center_y) / center_y  # -1 to +1
                    
                    # PID kontrolcülerden çıkış al
                    roll_output = pid_x.compute(error_x)    # Sola/sağa hareket
                    pitch_output = pid_y.compute(error_y)  # İleri/geri hareket
                    
                    # İniş hızı (yüksekliğe göre ayarla)
                    if current_alt > PRECISION_START_HEIGHT:
                        throttle_output = 0  # Henüz iniş yok
                    elif current_alt > LANDING_THRESHOLD_HEIGHT:
                        # Lineer interpolasyon: yükseklik azaldıkça yavaşla
                        descent_factor = current_alt / PRECISION_START_HEIGHT
                        throttle_output = -MAX_DESCENT_SPEED * (1 - descent_factor * 0.5)
                    else:
                        throttle_output = -MIN_DESCENT_SPEED  # Final iniş
                    
                    last_laser_time = time.time()
                else:
                    # Lazer kayıp - eski değerleri kullanma, sıfırla
                    roll_output = 0
                    pitch_output = 0
                    throttle_output = 0
                    
                    # PID integrallerini sıfırla (wind-up önleme)
                    if time.time() - last_laser_time > 0.5:
                        pid_x.reset()
                        pid_y.reset()
            
            # -----------------------------------------------------------------
            # 4. MSP RC KOMUTLARI GÖNDER
            # -----------------------------------------------------------------
            if current_state == SystemState.IDLE:
                # Sistem pasif - komut gönderme
                pass
            
            elif current_state == SystemState.SEARCHING:
                # Arama modu - hover (nötr değerler)
                msp.send_rc_override(
                    roll=1500,
                    pitch=1500,
                    throttle=1500,
                    yaw=1500
                )
            
            elif current_state in [SystemState.TRACKING, SystemState.APPROACH]:
                # Lazer takibi - PID çıkışlarını RC değerlerine dönüştür
                # RC değer aralığı: 1000-2000, merkez: 1500
                # PID çıkışı: -1 ile +1 arası (normalized)
                
                rc_roll = int(1500 + roll_output * RC_RANGE)
                rc_pitch = int(1500 + pitch_output * RC_RANGE)
                rc_throttle = int(1500 + throttle_output * RC_RANGE)
                rc_yaw = 1500  # Yaw değişmez
                
                # Limitleri uygula
                rc_roll = max(RC_MIN, min(RC_MAX, rc_roll))
                rc_pitch = max(RC_MIN, min(RC_MAX, rc_pitch))
                rc_throttle = max(RC_MIN, min(RC_MAX, rc_throttle))
                
                msp.send_rc_override(
                    roll=rc_roll,
                    pitch=rc_pitch,
                    throttle=rc_throttle,
                    yaw=rc_yaw
                )
            
            elif current_state == SystemState.LANDING:
                # Final iniş - sadece yavaş alçal
                msp.send_rc_override(
                    roll=1500,
                    pitch=1500,
                    throttle=int(1500 - MIN_DESCENT_SPEED * RC_RANGE),
                    yaw=1500
                )
            
            elif current_state == SystemState.LOST:
                # Lazer kayıp - hover moduna geç (LOITER)
                msp.send_rc_override(
                    roll=1500,
                    pitch=1500,
                    throttle=1500,
                    yaw=1500
                )
            
            # -----------------------------------------------------------------
            # 5. TELEMETRİ GÜNCELLE
            # -----------------------------------------------------------------
            with telemetry_lock:
                telemetry_data['laser_pos'] = laser_pos
                telemetry_data['laser_detected'] = laser_found
                telemetry_data['system_state'] = current_state.name
                telemetry_data['pid_output'] = (roll_output, pitch_output)
                telemetry_data['rc_channels'] = [
                    int(1500 + roll_output * RC_RANGE),
                    int(1500 + pitch_output * RC_RANGE),
                    int(1500 + throttle_output * RC_RANGE),
                    1500
                ]
            
            # -----------------------------------------------------------------
            # 6. DÖNGÜ ZAMANLAMASI
            # -----------------------------------------------------------------
            loop_elapsed = time.time() - loop_start
            sleep_time = loop_period - loop_elapsed
            
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                # Döngü çok yavaş!
                logger.warning(f"Control loop gecikmesi: {loop_elapsed*1000:.1f}ms "
                             f"(hedef: {loop_period*1000:.1f}ms)")
        
        except Exception as e:
            logger.error(f"Control loop hatası: {e}")
            time.sleep(0.1)


def telemetry_loop():
    """
    Telemetri okuma döngüsü.
    
    FC'den gelen MSP mesajlarını okur:
    - MSP_ALTITUDE (yükseklik)
    - MSP_STATUS (arm durumu, mod)
    
    Döngü hızı: 10Hz
    """
    global telemetry_data
    
    logger.info("Telemetry loop başlatıldı")
    
    while True:
        try:
            # FC'den yükseklik oku
            altitude = msp.request_altitude()
            
            # FC'den durum oku
            armed, mode = msp.request_status()
            
            # Thread-safe güncelle
            with telemetry_lock:
                if altitude is not None:
                    telemetry_data['altitude'] = altitude
                if armed is not None:
                    telemetry_data['armed'] = armed
                    telemetry_data['mode'] = mode
            
            time.sleep(0.1)  # 10Hz
        
        except Exception as e:
            logger.error(f"Telemetry loop hatası: {e}")
            time.sleep(0.5)


# =============================================================================
# FLASK ROUTES
# =============================================================================

@app.route('/')
def index():
    """Ana sayfa - HTML arayüz"""
    return HTML_TEMPLATE


@app.route('/video')
def video_feed():
    """Video stream endpoint"""
    def generate():
        while True:
            try:
                # İşlenmiş frame'i al
                frame = detector.get_processed_frame()
                if frame is not None:
                    # JPEG olarak encode et
                    import cv2
                    _, buffer = cv2.imencode('.jpg', frame, 
                                            [cv2.IMWRITE_JPEG_QUALITY, 50])
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + 
                           buffer.tobytes() + b'\r\n')
                time.sleep(0.05)  # 20fps
            except Exception as e:
                logger.error(f"Video stream hatası: {e}")
                time.sleep(0.1)
    
    return Response(generate(), 
                   mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/status')
def get_status():
    """Sistem durumu JSON endpoint"""
    with telemetry_lock:
        return jsonify({
            'altitude': telemetry_data['altitude'],
            'armed': telemetry_data['armed'],
            'mode': telemetry_data['mode'],
            'laser_detected': telemetry_data['laser_detected'],
            'laser_pos': telemetry_data['laser_pos'],
            'state': telemetry_data['system_state'],
            'pid_output': telemetry_data['pid_output'],
            'rc_channels': telemetry_data['rc_channels'],
            'timestamp': time.time()
        })


@app.route('/enable', methods=['POST'])
def toggle_enable():
    """Sistemi aktif/pasif yap"""
    current_state = state_machine.get_state()
    
    if current_state == SystemState.IDLE:
        state_machine.enable()
        return jsonify({'enabled': True, 'state': 'SEARCHING'})
    else:
        state_machine.disable()
        return jsonify({'enabled': False, 'state': 'IDLE'})


@app.route('/param', methods=['POST'])
def set_param():
    """PID ve diğer parametreleri ayarla"""
    data = request.json
    
    if 'kp' in data:
        pid_x.set_gains(kp=float(data['kp']))
        pid_y.set_gains(kp=float(data['kp']))
        logger.info(f"PID Kp güncellendi: {data['kp']}")
    
    if 'ki' in data:
        pid_x.set_gains(ki=float(data['ki']))
        pid_y.set_gains(ki=float(data['ki']))
        logger.info(f"PID Ki güncellendi: {data['ki']}")
    
    if 'kd' in data:
        pid_x.set_gains(kd=float(data['kd']))
        pid_y.set_gains(kd=float(data['kd']))
        logger.info(f"PID Kd güncellendi: {data['kd']}")
    
    if 'threshold' in data:
        detector.set_threshold(int(data['threshold']))
        logger.info(f"Lazer threshold güncellendi: {data['threshold']}")
    
    return jsonify({'ok': True})


# =============================================================================
# HTML TEMPLATE
# =============================================================================
HTML_TEMPLATE = '''
<!DOCTYPE html>
<html>
<head>
    <title>Precision Landing - INAV MSP</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body { 
            font-family: 'Segoe UI', sans-serif; 
            background: #1a1a2e; 
            color: #eee;
            padding: 10px;
        }
        .container { max-width: 800px; margin: 0 auto; }
        h1 { text-align: center; color: #00d4ff; margin-bottom: 10px; }
        .video-container {
            position: relative;
            width: 100%;
            background: #000;
            border-radius: 8px;
            overflow: hidden;
            margin-bottom: 15px;
        }
        .video-container img {
            width: 100%;
            display: block;
        }
        .status-bar {
            padding: 15px;
            border-radius: 8px;
            margin-bottom: 15px;
            font-size: 18px;
            font-weight: bold;
            text-align: center;
        }
        .status-idle { background: #444; }
        .status-searching { background: #2d5a27; }
        .status-tracking { background: #0066cc; }
        .status-approach { background: #cc6600; }
        .status-landing { background: #cc0066; }
        .status-lost { background: #cc0000; }
        
        .grid { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; }
        .card {
            background: #16213e;
            padding: 15px;
            border-radius: 8px;
        }
        .card h3 { color: #00d4ff; margin-bottom: 10px; }
        .value { font-size: 24px; font-weight: bold; }
        
        .controls { margin-top: 15px; }
        .btn {
            width: 100%;
            padding: 15px;
            font-size: 18px;
            border: none;
            border-radius: 8px;
            cursor: pointer;
            margin-bottom: 10px;
        }
        .btn-enable { background: #00cc66; color: #000; }
        .btn-disable { background: #cc0000; color: #fff; }
        
        .slider-group { margin: 10px 0; }
        .slider-group label { display: block; margin-bottom: 5px; }
        .slider-group input { width: 100%; }
        .slider-value { float: right; color: #00d4ff; }
        
        .telemetry { font-family: monospace; font-size: 12px; }
    </style>
</head>
<body>
    <div class="container">
        <h1>🎯 Precision Landing</h1>
        
        <div class="video-container">
            <img src="/video" alt="Camera Feed">
        </div>
        
        <div id="status" class="status-bar status-idle">
            ⏸️ Sistem Pasif
        </div>
        
        <div class="grid">
            <div class="card">
                <h3>📍 Yükseklik</h3>
                <div id="altitude" class="value">0.0 m</div>
            </div>
            <div class="card">
                <h3>🔴 Lazer</h3>
                <div id="laser" class="value">---</div>
            </div>
            <div class="card">
                <h3>🎮 RC Channels</h3>
                <div id="rc" class="telemetry">
                    R: 1500 | P: 1500<br>
                    T: 1500 | Y: 1500
                </div>
            </div>
            <div class="card">
                <h3>📊 PID Output</h3>
                <div id="pid" class="telemetry">
                    X: 0.00 | Y: 0.00
                </div>
            </div>
        </div>
        
        <div class="controls">
            <button id="enableBtn" class="btn btn-enable" onclick="toggleEnable()">
                ▶️ Sistemi Başlat
            </button>
            
            <div class="card">
                <h3>⚙️ PID Ayarları</h3>
                <div class="slider-group">
                    <label>Kp: <span id="kpVal" class="slider-value">0.10</span></label>
                    <input type="range" id="kp" min="0" max="1" step="0.01" value="0.1">
                </div>
                <div class="slider-group">
                    <label>Ki: <span id="kiVal" class="slider-value">0.00</span></label>
                    <input type="range" id="ki" min="0" max="0.5" step="0.01" value="0">
                </div>
                <div class="slider-group">
                    <label>Kd: <span id="kdVal" class="slider-value">0.00</span></label>
                    <input type="range" id="kd" min="0" max="0.5" step="0.01" value="0">
                </div>
                <div class="slider-group">
                    <label>Threshold: <span id="threshVal" class="slider-value">200</span></label>
                    <input type="range" id="thresh" min="100" max="255" step="5" value="200">
                </div>
            </div>
        </div>
    </div>
    
    <script>
        let systemEnabled = false;
        
        function toggleEnable() {
            fetch('/enable', {method: 'POST'})
                .then(r => r.json())
                .then(d => {
                    systemEnabled = d.enabled;
                    updateEnableBtn();
                });
        }
        
        function updateEnableBtn() {
            const btn = document.getElementById('enableBtn');
            if (systemEnabled) {
                btn.textContent = '⏹️ Sistemi Durdur';
                btn.className = 'btn btn-disable';
            } else {
                btn.textContent = '▶️ Sistemi Başlat';
                btn.className = 'btn btn-enable';
            }
        }
        
        // Slider handlers
        ['kp', 'ki', 'kd', 'thresh'].forEach(id => {
            document.getElementById(id).oninput = function() {
                const valId = id === 'thresh' ? 'threshVal' : id + 'Val';
                document.getElementById(valId).textContent = 
                    id === 'thresh' ? this.value : parseFloat(this.value).toFixed(2);
                
                let data = {};
                data[id === 'thresh' ? 'threshold' : id] = 
                    id === 'thresh' ? parseInt(this.value) : parseFloat(this.value);
                
                fetch('/param', {
                    method: 'POST',
                    headers: {'Content-Type': 'application/json'},
                    body: JSON.stringify(data)
                });
            };
        });
        
        // Status güncelleme
        setInterval(() => {
            fetch('/status')
                .then(r => r.json())
                .then(d => {
                    // Altitude
                    document.getElementById('altitude').textContent = 
                        d.altitude.toFixed(1) + ' m';
                    
                    // Laser
                    if (d.laser_detected && d.laser_pos) {
                        document.getElementById('laser').textContent = 
                            `X:${d.laser_pos[0]} Y:${d.laser_pos[1]}`;
                    } else {
                        document.getElementById('laser').textContent = '---';
                    }
                    
                    // RC Channels
                    const rc = d.rc_channels;
                    document.getElementById('rc').innerHTML = 
                        `R: ${rc[0]} | P: ${rc[1]}<br>T: ${rc[2]} | Y: ${rc[3]}`;
                    
                    // PID Output
                    const pid = d.pid_output;
                    document.getElementById('pid').textContent = 
                        `X: ${pid[0].toFixed(2)} | Y: ${pid[1].toFixed(2)}`;
                    
                    // Status bar
                    const status = document.getElementById('status');
                    const state = d.state;
                    status.className = 'status-bar status-' + state.toLowerCase();
                    
                    const stateText = {
                        'IDLE': '⏸️ Sistem Pasif',
                        'SEARCHING': '🔍 Lazer Aranıyor...',
                        'TRACKING': '🔴 Lazer Takip Ediliyor',
                        'APPROACH': '📍 Yaklaşma',
                        'LANDING': '🛬 İniş Yapılıyor',
                        'LOST': '⚠️ Lazer Kayıp!'
                    };
                    status.textContent = stateText[state] || state;
                    status.textContent += ` | Alt: ${d.altitude.toFixed(1)}m`;
                    
                    // Enable button sync
                    systemEnabled = (state !== 'IDLE');
                    updateEnableBtn();
                });
        }, 200);
    </script>
</body>
</html>
'''


# =============================================================================
# MAIN ENTRY POINT
# =============================================================================

def main():
    """
    Ana program giriş noktası.
    """
    global msp, detector, pid_x, pid_y, state_machine
    
    # Banner yazdır
    print_banner()
    print_ppm_table()
    
    # -------------------------------------------------------------------------
    # 1. MSP PROTOKOL BAŞLAT
    # -------------------------------------------------------------------------
    logger.info("MSP protokolü başlatılıyor...")
    msp = MSPProtocol(
        port=FC_UART_PORT,
        baudrate=FC_UART_BAUDRATE
    )
    
    if not msp.connect():
        logger.error("FC bağlantısı başarısız! Simülasyon modunda devam ediliyor.")
        # Simülasyon modu için dummy MSP
        msp = MSPProtocol(simulation=True)
    else:
        logger.info("✅ FC bağlantısı başarılı")
        
        # FC bilgilerini oku
        fc_info = msp.request_fc_info()
        if fc_info:
            logger.info(f"   FC: {fc_info.get('name', 'Unknown')}")
            logger.info(f"   Version: {fc_info.get('version', 'Unknown')}")
    
    # -------------------------------------------------------------------------
    # 2. LAZER DEDEKTÖR BAŞLAT
    # -------------------------------------------------------------------------
    logger.info("Lazer dedektör başlatılıyor...")
    detector = LaserDetector(
        resolution=(CAMERA_RES_X, CAMERA_RES_Y),
        threshold=LASER_THRESHOLD,
        min_area=LASER_MIN_AREA,
        max_area=LASER_MAX_AREA
    )
    
    if not detector.start():
        logger.error("Kamera başlatılamadı!")
        return
    
    logger.info("✅ Kamera başarılı")
    
    # -------------------------------------------------------------------------
    # 3. PID KONTROLCÜLERİ BAŞLAT
    # -------------------------------------------------------------------------
    logger.info("PID kontrolcüleri başlatılıyor...")
    
    pid_x = PIDController(
        kp=PID_KP, ki=PID_KI, kd=PID_KD,
        output_min=-1.0, output_max=1.0,
        name="PID_X"
    )
    
    pid_y = PIDController(
        kp=PID_KP, ki=PID_KI, kd=PID_KD,
        output_min=-1.0, output_max=1.0,
        name="PID_Y"
    )
    
    logger.info("✅ PID kontrolcüleri hazır")
    
    # -------------------------------------------------------------------------
    # 4. STATE MACHINE BAŞLAT
    # -------------------------------------------------------------------------
    logger.info("State machine başlatılıyor...")
    state_machine = StateMachine(
        detection_time=LASER_DETECTION_TIME,
        lost_timeout=LASER_LOST_TIMEOUT,
        start_height=PRECISION_START_HEIGHT,
        landing_height=LANDING_THRESHOLD_HEIGHT
    )
    logger.info("✅ State machine hazır")
    
    # -------------------------------------------------------------------------
    # 5. THREAD'LERİ BAŞLAT
    # -------------------------------------------------------------------------
    logger.info("Thread'ler başlatılıyor...")
    
    # Control loop thread
    control_thread = threading.Thread(
        target=control_loop,
        daemon=True,
        name="ControlLoop"
    )
    control_thread.start()
    
    # Telemetry loop thread
    telemetry_thread = threading.Thread(
        target=telemetry_loop,
        daemon=True,
        name="TelemetryLoop"
    )
    telemetry_thread.start()
    
    logger.info("✅ Thread'ler başlatıldı")
    
    # -------------------------------------------------------------------------
    # 6. WEB SUNUCUSU BAŞLAT
    # -------------------------------------------------------------------------
    local_ip = get_local_ip()
    print("\n" + "=" * 70)
    print(f"🌐 Web Arayüzü: http://{local_ip}:{WEB_SERVER_PORT}")
    print("=" * 70)
    print("\n⚠️ İlk Uçuş Kontrol Listesi:")
    print("   1. INAV Configurator'da MSP RC Override aktif mi?")
    print("   2. AUX switch MSP RC OVERRIDE moduna atandı mı?")
    print("   3. msp_override_channels = 15 (kanal 1-4)")
    print("   4. Pervaneler TAKILMADAN test et!")
    print("   5. RC kumanda her zaman elinizde olsun!")
    print("=" * 70 + "\n")
    
    # Flask sunucusu başlat (blocking)
    app.run(
        host='0.0.0.0',
        port=WEB_SERVER_PORT,
        debug=False,
        threaded=True
    )


if __name__ == '__main__':
    main()
