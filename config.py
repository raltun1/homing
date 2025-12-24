#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
===============================================================================
CONFIG.PY - Sistem Konfigürasyonu
===============================================================================
Tüm ayarlanabilir parametreler bu dosyada tanımlıdır.
Değişiklik yapmak için bu dosyayı düzenleyin.

Gruplar:
  1. Flight Controller (FC) Ayarları
  2. Kamera Ayarları
  3. Lazer Tespit Ayarları
  4. PID Kontrolcü Ayarları
  5. Hız Limitleri
  6. Güvenlik Parametreleri
  7. RC Kanal Ayarları
  8. Web Sunucu Ayarları
===============================================================================
"""

# =============================================================================
# 1. FLIGHT CONTROLLER (FC) AYARLARI
# =============================================================================

# UART portu (Raspberry Pi GPIO 14/15 → UART3)
FC_UART_PORT = '/dev/serial0'

# Baudrate (INAV varsayılan: 115200)
# Not: INAV Configurator'da aynı değer ayarlanmalı
FC_UART_BAUDRATE = 115200

# MSP mesaj gönderme hızı (Hz)
# ÖNEMLİ: 5Hz altında FC otomatik olarak RC receiver'a döner (failsafe)
# Önerilen: 20Hz (50ms aralık)
MSP_SEND_RATE_HZ = 20

# MSP timeout (saniye)
# Bu süre içinde cevap gelmezse bağlantı kopuk sayılır
MSP_TIMEOUT = 1.0


# =============================================================================
# 2. KAMERA AYARLARI
# =============================================================================

# Kamera çözünürlüğü (piksel)
# IMX296: 1456x1088
# ZW LRCP: 1920x1080
CAMERA_RES_X = 1456
CAMERA_RES_Y = 1088

# Frame rate (fps)
# IMX296: 60fps max
# ZW LRCP: 90fps max
CAMERA_FPS = 60

# Görüş alanı (Field of View) - derece
# Bu değer kalibre edilmeli!
# Geniş açı M12 lens: ~150°
# Dar açı 6mm lens: ~60°
CAMERA_FOV_H = 150  # Yatay FOV
CAMERA_FOV_V = 120  # Dikey FOV (tahmin)

# Kamera device ID
# RPI CSI: 0
# USB: genellikle 0 veya 1
CAMERA_DEVICE_ID = 0


# =============================================================================
# 3. LAZER TESPİT AYARLARI
# =============================================================================

# Parlaklık eşiği (0-255)
# 940nm IR lazer için yüksek değer gerekli
# Düşük değer: Daha fazla false positive
# Yüksek değer: Lazer kaçırılabilir
LASER_THRESHOLD = 200

# Minimum lazer alanı (piksel²)
# Çok küçük noktalar gürültü olabilir
LASER_MIN_AREA = 5

# Maksimum lazer alanı (piksel²)
# Çok büyük alanlar lazer değil başka bir ışık kaynağı
LASER_MAX_AREA = 500

# Dairesellik eşiği (0-1)
# 1.0 = mükemmel daire
# Lazer noktası genellikle dairesel
LASER_CIRCULARITY_MIN = 0.5

# Lazer onay süresi (saniye)
# Bu süre boyunca lazer görülmeli ki geçerli sayılsın
# Anlık false positive'leri filtreler
LASER_DETECTION_TIME = 2.0

# Lazer kayıp timeout (saniye)
# Bu süre lazer görülmezse LOST durumuna geç
LASER_LOST_TIMEOUT = 3.0


# =============================================================================
# 4. PID KONTROLCÜ AYARLARI
# =============================================================================

# Orantısal kazanç (P)
# Yüksek değer: Hızlı tepki, ama salınım riski
# Düşük değer: Yavaş tepki, ama stabil
# Önerilen başlangıç: 0.1
PID_KP = 0.1

# İntegral kazanç (I)
# Sabit hataları (offset) düzeltir
# Dikkat: Wind-up problemi olabilir
# Önerilen başlangıç: 0.0 (kapalı)
PID_KI = 0.0

# Türev kazanç (D)
# Ani değişimleri sönümler
# Gürültüye hassas olabilir
# Önerilen başlangıç: 0.0 (kapalı)
PID_KD = 0.0

# PID integral limiti (anti-windup)
# İntegral teriminin maksimum değeri
PID_INTEGRAL_MAX = 0.5


# =============================================================================
# 5. HIZ LİMİTLERİ
# =============================================================================

# Maksimum yatay hız (m/s)
# İlk testler için düşük tut!
MAX_HORIZONTAL_SPEED = 0.4

# Maksimum iniş hızı (m/s)
# Normal approach için
MAX_DESCENT_SPEED = 0.25

# Minimum iniş hızı (m/s)
# Final landing için (yavaş ve kontrollü)
MIN_DESCENT_SPEED = 0.08

# Deadzone (piksel)
# Lazer bu kadar piksel merkezden uzaksa hareket etme
# Salınımı önler
DEADZONE_PIXELS = 40


# =============================================================================
# 6. GÜVENLİK PARAMETRELERİ
# =============================================================================

# Precision landing başlama yüksekliği (metre)
# Bu yüksekliğin altında precision landing aktif
PRECISION_START_HEIGHT = 15.0

# Final iniş yüksekliği (metre)
# Bu yüksekliğin altında sadece yavaş alçal
LANDING_THRESHOLD_HEIGHT = 0.8

# Otomatik reset gecikmesi (saniye)
# Landing tamamlandıktan sonra sistem bu süre bekler
AUTO_RESET_DELAY = 5.0


# =============================================================================
# 7. RC KANAL AYARLARI
# =============================================================================

# RC kanal aralığı
RC_MIN = 1000  # Minimum PWM değeri
RC_MAX = 2000  # Maksimum PWM değeri
RC_MID = 1500  # Orta (nötr) değer

# RC range (merkez etrafındaki hareket aralığı)
# PID çıkışı -1 ile +1 arasında
# RC_RANGE=300 → 1500±300 = 1200-1800
RC_RANGE = 300

# Kanal sıralaması (INAV varsayılan: AETR)
# 0: Aileron (Roll)
# 1: Elevator (Pitch)
# 2: Throttle
# 3: Rudder (Yaw)
RC_CHANNEL_ROLL = 0
RC_CHANNEL_PITCH = 1
RC_CHANNEL_THROTTLE = 2
RC_CHANNEL_YAW = 3


# =============================================================================
# 8. WEB SUNUCU AYARLARI
# =============================================================================

# Web sunucu portu
WEB_SERVER_PORT = 5000

# Video stream kalitesi (JPEG quality: 1-100)
VIDEO_QUALITY = 50

# Video stream FPS
VIDEO_STREAM_FPS = 20


# =============================================================================
# 9. DEBUG VE LOG AYARLARI
# =============================================================================

# Debug modu
# True: Ekstra log çıktısı
DEBUG_MODE = False

# Log dosyası
LOG_FILE = '/home/pi/precision_landing.log'

# Konsola log yaz
LOG_TO_CONSOLE = True

# Dosyaya log yaz
LOG_TO_FILE = False
