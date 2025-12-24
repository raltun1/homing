# 🎯 GPS-Denied Precision Landing System

**INAV MSP RC Override ile 940nm IR Lazer Takipli Precision Landing**

## 📋 Özet

Bu sistem, GPS sinyalinin olmadığı ortamlarda drone'un 940nm IR lazer beacon'a
takip ederek hassas iniş yapmasını sağlar. Raspberry Pi 5 companion computer
üzerinde çalışır ve F722 flight controller ile MSP V2 protokolü üzerinden
haberleşir.

## 🔧 Donanım Gereksinimleri

| Bileşen | Model | Not |
|---------|-------|-----|
| Flight Controller | SpeedyBee F722 V3 | INAV 9.0+ firmware |
| Companion Computer | Raspberry Pi 5 4GB | 8GB önerilir |
| Kamera | innomaker IMX296 / ZW LRCP | Global shutter önemli |
| IR Lazer | 940nm Lazer + Driver | Yer istasyonunda |

## 📡 Bağlantı Şeması

```
Raspberry Pi 5              SpeedyBee F722
┌─────────────┐            ┌─────────────┐
│ GPIO14 (TX)─┼────────────┼─UART3 RX    │
│ GPIO15 (RX)─┼────────────┼─UART3 TX    │
│ GND────────┼────────────┼─GND         │
└─────────────┘            └─────────────┘
```

**ÖNEMLİ:** 3.3V logic level. GND bağlantısı zorunlu!

## 🚀 Hızlı Başlangıç

### 1. Raspberry Pi Kurulumu

```bash
# Sistemi güncelle
sudo apt update && sudo apt upgrade -y

# Gerekli paketler
sudo apt install -y python3-pip python3-opencv python3-picamera2

# Python paketleri
pip install flask pyserial numpy --break-system-packages

# Serial port izinleri
sudo usermod -a -G dialout $USER

# UART aktifleştir
sudo raspi-config
# Interface Options → Serial Port → Login: NO, Hardware: YES
```

### 2. INAV Konfigürasyonu

INAV Configurator'da:

```bash
# CLI komutları
set msp_override_channels = 15
save
```

Modes sekmesinde:
- "MSP RC OVERRIDE" modunu AUX5'e ata
- Switch HIGH = RPI kontrol, LOW = RC kontrol

### 3. Çalıştırma

```bash
cd precision_landing_inav
python3 main.py
```

Web arayüzüne tarayıcıdan eriş:
```
http://<RPI_IP>:5000
```

## 📁 Dosya Yapısı

```
precision_landing_inav/
├── main.py              # Ana program ve Flask sunucusu
├── config.py            # Tüm konfigürasyon parametreleri
├── msp_protocol.py      # MSP V2 protokol implementasyonu
├── laser_detector.py    # Lazer tespit (OpenCV)
├── pid_controller.py    # PID kontrolcü
├── state_machine.py     # Durum makinesi (FSM)
├── requirements.txt     # Python bağımlılıkları
└── README.md           # Bu dosya
```

## ⚙️ Konfigürasyon

`config.py` dosyasında tüm ayarlanabilir parametreler bulunur:

### FC Bağlantısı
```python
FC_UART_PORT = '/dev/serial0'
FC_UART_BAUDRATE = 115200
MSP_SEND_RATE_HZ = 20  # min 5Hz!
```

### Kamera
```python
CAMERA_RES_X = 1456
CAMERA_RES_Y = 1088
CAMERA_FOV_H = 150  # Derece
```

### PID
```python
PID_KP = 0.1  # Başlangıç değeri, test ile ayarla
PID_KI = 0.0  # Başta kapalı tut
PID_KD = 0.0  # Başta kapalı tut
```

### Güvenlik
```python
LASER_DETECTION_TIME = 2.0  # Onay süresi
LASER_LOST_TIMEOUT = 3.0    # Kayıp timeout
```

## 🔄 Durum Makinesi

```
IDLE → SEARCHING → TRACKING → APPROACH → LANDING → COMPLETE
                       ↑                    │
                       └────── LOST ────────┘
```

| Durum | Açıklama |
|-------|----------|
| IDLE | Sistem pasif |
| SEARCHING | Lazer aranıyor |
| TRACKING | Lazer bulundu, onay bekleniyor |
| APPROACH | Lazere doğru yaklaşma |
| LANDING | Final iniş (<0.8m) |
| LOST | Lazer kayıp |
| COMPLETE | İniş tamamlandı |

## 🛡️ Güvenlik Özellikleri

1. **MSP Failsafe**: MSP <5Hz → Otomatik RC takeover
2. **Pilot Override**: AUX switch ile anında manual kontrol
3. **Lazer Kayıp**: 3 saniye sonra hover modu
4. **Yükseklik Limiti**: 15m üzerinde precision landing pasif

## 📊 Web Arayüzü

- **Video Stream**: Canlı kamera görüntüsü + lazer overlay
- **Durum Monitör**: Yükseklik, lazer pozisyonu, sistem durumu
- **PID Ayarları**: Real-time slider ile ayarlama
- **Enable/Disable**: Tek tuşla sistem kontrolü

## 🧪 Test Prosedürü

### Masa Üstü Test (Pervanesiz)
1. FC'yi masaya sabitle
2. RPI bağlantısını yap
3. Lazer kaynağını hazırla
4. Web arayüzünü aç
5. "Sistemi Başlat" butonuna bas
6. Lazeri kameraya göster
7. RC channel değişimlerini Configurator'da izle

### İlk Uçuş Testi
1. Geniş, açık alan seç
2. Düşük yükseklikte (<5m) başla
3. Pilot her zaman RC kumandayı hazır tut
4. Önce hover'da MSP override test et
5. Sonra yavaşça precision landing test et

## ⚠️ Önemli Uyarılar

- **ASLA** pervaneli test öncesi masada MSP override test etmeden uçuş yapma
- **HER ZAMAN** RC kumanda elinizde olsun
- MSP gönderme hızı **minimum 5Hz** olmalı
- İlk testlerde **düşük PID değerleri** kullan
- 940nm lazer **gözle görünmez** - dikkatli ol!

## 🐛 Sorun Giderme

### FC Bağlanmıyor
```bash
# Port kontrolü
ls -la /dev/serial*

# İzin kontrolü
groups $USER  # 'dialout' olmalı

# UART test
minicom -D /dev/serial0 -b 115200
```

### Kamera Açılmıyor
```bash
# Picamera2 test
libcamera-hello

# USB kamera test
v4l2-ctl --list-devices
```

### MSP Override Çalışmıyor
- INAV Configurator'da "MSP RC OVERRIDE" modu atandı mı?
- `msp_override_channels = 15` ayarlandı mı?
- AUX switch doğru pozisyonda mı?

## 📚 Referanslar

- [INAV GitHub](https://github.com/iNavFlight/inav)
- [MSP V2 Protocol](https://github.com/iNavFlight/inav/wiki/MSP-V2)
- [The Devana Project](https://thedevanaproject.com) - MSP RC Override tutorial
- [OpenCV Documentation](https://docs.opencv.org)

## 👥 Yazarlar

- **Ramazan** - Sistem tasarımı ve donanım
- **Claude** - Yazılım implementasyonu

## 📄 Lisans

Bu proje eğitim amaçlıdır. Kendi sorumluluğunuzda kullanın.

---

**🎯 GPS-Denied Precision Landing - v1.0.0**
