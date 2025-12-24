#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
===============================================================================
LASER_DETECTOR.PY - Lazer Tespit Modülü
===============================================================================
940nm IR lazer beacon tespiti için görüntü işleme modülü.

Algoritma:
  1. Frame yakala (kamera veya CSI)
  2. Grayscale'e çevir
  3. Threshold uygula (lazer çok parlak)
  4. Kontur bul
  5. Dairesellik ve alan filtrele
  6. En uygun noktayı seç (centroid)

Desteklenen Kameralar:
  - Raspberry Pi Camera (picamera2)
  - USB UVC kameralar (OpenCV)
  - CSI kameralar (libcamera)

Optimizasyonlar:
  - Global shutter kamera önerilir (hareket bulanıklığı yok)
  - IR filtresi çıkarılmış kamera daha iyi (940nm için)
  - Düşük çözünürlük = daha hızlı işlem

Not:
  940nm ışık insan gözüne görünmez!
  Mono/IR kamera kullanılmalı.
===============================================================================
"""

import time
import threading
import logging
from typing import Optional, Tuple, List

import numpy as np

# OpenCV
try:
    import cv2
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False
    logging.warning("OpenCV kurulu değil!")

# Picamera2 (Raspberry Pi Camera Module)
try:
    from picamera2 import Picamera2
    PICAMERA2_AVAILABLE = True
except ImportError:
    PICAMERA2_AVAILABLE = False
    logging.info("picamera2 kurulu değil - USB kamera kullanılacak")

# Config import
from config import *


class LaserDetector:
    """
    Lazer tespit sınıfı.
    
    Kameradan görüntü alır ve 940nm IR lazer noktasını tespit eder.
    
    Attributes:
        resolution (tuple): Kamera çözünürlüğü (width, height)
        threshold (int): Parlaklık eşiği (0-255)
        min_area (int): Minimum lazer alanı (piksel²)
        max_area (int): Maksimum lazer alanı (piksel²)
    
    Example:
        >>> detector = LaserDetector((1456, 1088), threshold=200)
        >>> detector.start()
        >>> frame = detector.capture_frame()
        >>> pos, processed = detector.detect_laser(frame)
        >>> print(f"Lazer pozisyonu: {pos}")
    """
    
    def __init__(self, resolution: Tuple[int, int] = (1456, 1088),
                 threshold: int = 200,
                 min_area: int = 5,
                 max_area: int = 500,
                 circularity_min: float = 0.5):
        """
        Lazer dedektör nesnesini oluştur.
        
        Args:
            resolution: Kamera çözünürlüğü (width, height)
            threshold: Parlaklık eşiği (0-255)
            min_area: Minimum lazer alanı (piksel²)
            max_area: Maksimum lazer alanı (piksel²)
            circularity_min: Minimum dairesellik (0-1)
        """
        self.resolution = resolution
        self.threshold = threshold
        self.min_area = min_area
        self.max_area = max_area
        self.circularity_min = circularity_min
        
        # Kamera instance
        self._camera = None
        self._camera_type = None  # 'picamera2' veya 'opencv'
        
        # Son işlenmiş frame (web stream için)
        self._processed_frame = None
        self._frame_lock = threading.Lock()
        
        # İstatistikler
        self._frame_count = 0
        self._detection_count = 0
        self._fps = 0.0
        self._last_fps_time = time.time()
        self._fps_frame_count = 0
        
        # Logger
        self._logger = logging.getLogger(__name__)
    
    # =========================================================================
    # CAMERA INITIALIZATION
    # =========================================================================
    
    def start(self) -> bool:
        """
        Kamerayı başlat.
        
        Önce picamera2 dener, yoksa OpenCV USB kamera kullanır.
        
        Returns:
            bool: Başarılı ise True
        """
        # Picamera2 dene (RPI Camera Module)
        if PICAMERA2_AVAILABLE:
            try:
                self._logger.info("Picamera2 başlatılıyor...")
                self._camera = Picamera2()
                
                # Video konfigürasyonu
                config = self._camera.create_video_configuration(
                    main={"size": self.resolution, "format": "RGB888"}
                )
                self._camera.configure(config)
                self._camera.start()
                
                # Kameranın hazır olmasını bekle
                time.sleep(2)
                
                self._camera_type = 'picamera2'
                self._logger.info(f"✅ Picamera2 başlatıldı: {self.resolution}")
                return True
                
            except Exception as e:
                self._logger.warning(f"Picamera2 başlatılamadı: {e}")
                self._camera = None
        
        # OpenCV ile USB kamera dene
        if CV2_AVAILABLE:
            try:
                self._logger.info("USB kamera başlatılıyor...")
                self._camera = cv2.VideoCapture(CAMERA_DEVICE_ID)
                
                if not self._camera.isOpened():
                    raise Exception("Kamera açılamadı")
                
                # Çözünürlük ayarla
                self._camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.resolution[0])
                self._camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.resolution[1])
                
                # FPS ayarla
                self._camera.set(cv2.CAP_PROP_FPS, CAMERA_FPS)
                
                # Buffer boyutunu küçük tut (düşük latency için)
                self._camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                
                self._camera_type = 'opencv'
                
                # Gerçek çözünürlüğü oku
                actual_w = int(self._camera.get(cv2.CAP_PROP_FRAME_WIDTH))
                actual_h = int(self._camera.get(cv2.CAP_PROP_FRAME_HEIGHT))
                
                self._logger.info(f"✅ USB kamera başlatıldı: {actual_w}x{actual_h}")
                return True
                
            except Exception as e:
                self._logger.error(f"USB kamera başlatılamadı: {e}")
                self._camera = None
        
        self._logger.error("Hiçbir kamera başlatılamadı!")
        return False
    
    def stop(self):
        """
        Kamerayı durdur ve kaynakları serbest bırak.
        """
        if self._camera:
            if self._camera_type == 'picamera2':
                self._camera.stop()
            elif self._camera_type == 'opencv':
                self._camera.release()
            
            self._camera = None
            self._logger.info("Kamera durduruldu")
    
    # =========================================================================
    # FRAME CAPTURE
    # =========================================================================
    
    def capture_frame(self) -> Optional[np.ndarray]:
        """
        Kameradan bir frame yakala.
        
        Returns:
            np.ndarray: BGR formatında frame veya None
        """
        if not self._camera:
            return None
        
        try:
            if self._camera_type == 'picamera2':
                # Picamera2 RGB888 formatında döndürür
                frame = self._camera.capture_array()
                # RGB → BGR (OpenCV formatı)
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                
            elif self._camera_type == 'opencv':
                ret, frame = self._camera.read()
                if not ret:
                    return None
            
            self._frame_count += 1
            
            # FPS hesapla
            self._fps_frame_count += 1
            now = time.time()
            elapsed = now - self._last_fps_time
            if elapsed >= 1.0:
                self._fps = self._fps_frame_count / elapsed
                self._fps_frame_count = 0
                self._last_fps_time = now
            
            return frame
            
        except Exception as e:
            self._logger.error(f"Frame yakalama hatası: {e}")
            return None
    
    # =========================================================================
    # LASER DETECTION
    # =========================================================================
    
    def detect_laser(self, frame: np.ndarray) -> Tuple[Optional[Tuple[int, int]], np.ndarray]:
        """
        Frame içinde lazer noktasını tespit et.
        
        Algoritma:
        1. Grayscale'e çevir
        2. Threshold uygula (lazer çok parlak)
        3. Kontur bul
        4. Alan ve dairesellik filtrele
        5. En iyi adayı seç
        
        Args:
            frame: BGR formatında input frame
        
        Returns:
            Tuple[position, processed_frame]:
                - position: (x, y) lazer merkezi veya None
                - processed_frame: Görselleştirilmiş frame
        """
        if frame is None:
            return (None, None)
        
        # Orijinal frame'in kopyası (görselleştirme için)
        display_frame = frame.copy()
        
        # ---------------------------------------------------------------------
        # 1. GRAYSCALE'E ÇEVİR
        # ---------------------------------------------------------------------
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # ---------------------------------------------------------------------
        # 2. THRESHOLD UYGULA
        # ---------------------------------------------------------------------
        _, binary = cv2.threshold(gray, self.threshold, 255, cv2.THRESH_BINARY)
        
        # ---------------------------------------------------------------------
        # 3. GÜRÜLTÜ AZALTMA
        # ---------------------------------------------------------------------
        kernel = np.ones((3, 3), np.uint8)
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)
        
        # ---------------------------------------------------------------------
        # 4. KONTUR BUL
        # ---------------------------------------------------------------------
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, 
                                       cv2.CHAIN_APPROX_SIMPLE)
        
        # ---------------------------------------------------------------------
        # 5. ADAY FİLTRELEME
        # ---------------------------------------------------------------------
        candidates = []
        
        for contour in contours:
            # Alan hesapla
            area = cv2.contourArea(contour)
            
            # Alan filtresi
            if area < self.min_area or area > self.max_area:
                continue
            
            # Dairesellik hesapla
            perimeter = cv2.arcLength(contour, True)
            if perimeter == 0:
                continue
            
            circularity = 4 * np.pi * area / (perimeter * perimeter)
            
            # Dairesellik filtresi
            if circularity < self.circularity_min:
                continue
            
            # Centroid hesapla
            M = cv2.moments(contour)
            if M["m00"] == 0:
                continue
            
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            
            # Bounding box
            x, y, w, h = cv2.boundingRect(contour)
            
            # Adayı kaydet
            candidates.append({
                'center': (cx, cy),
                'area': area,
                'circularity': circularity,
                'bbox': (x, y, w, h),
                'contour': contour
            })
        
        # ---------------------------------------------------------------------
        # 6. EN İYİ ADAYI SEÇ
        # ---------------------------------------------------------------------
        laser_pos = None
        best_candidate = None
        
        if candidates:
            candidates.sort(key=lambda c: c['circularity'], reverse=True)
            best_candidate = candidates[0]
            laser_pos = best_candidate['center']
            self._detection_count += 1
        
        # ---------------------------------------------------------------------
        # 7. GÖRSELLEŞTİRME
        # ---------------------------------------------------------------------
        center_x = self.resolution[0] // 2
        center_y = self.resolution[1] // 2
        
        # Merkez çarpı işareti
        cv2.line(display_frame, (center_x - 20, center_y), 
                (center_x + 20, center_y), (0, 255, 0), 2)
        cv2.line(display_frame, (center_x, center_y - 20), 
                (center_x, center_y + 20), (0, 255, 0), 2)
        
        # Deadzone çemberi
        cv2.circle(display_frame, (center_x, center_y), 
                  DEADZONE_PIXELS, (0, 255, 0), 1)
        
        # Tüm adayları çiz (gri)
        for candidate in candidates:
            cx, cy = candidate['center']
            cv2.circle(display_frame, (cx, cy), 5, (128, 128, 128), 1)
        
        # En iyi adayı vurgula (kırmızı)
        if best_candidate:
            cx, cy = best_candidate['center']
            x, y, w, h = best_candidate['bbox']
            
            cv2.rectangle(display_frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
            cv2.circle(display_frame, (cx, cy), 8, (0, 0, 255), -1)
            cv2.line(display_frame, (center_x, center_y), (cx, cy), (0, 0, 255), 2)
            
            error_x = cx - center_x
            error_y = cy - center_y
            cv2.putText(display_frame, f"dX:{error_x} dY:{error_y}", 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        
        # FPS göster
        cv2.putText(display_frame, f"FPS: {self._fps:.1f}", 
                   (10, self.resolution[1] - 10), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        # Threshold göster
        cv2.putText(display_frame, f"Thresh: {self.threshold}", 
                   (self.resolution[0] - 120, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        
        # İşlenmiş frame'i sakla
        with self._frame_lock:
            self._processed_frame = display_frame.copy()
        
        return (laser_pos, display_frame)
    
    # =========================================================================
    # UTILITY METHODS
    # =========================================================================
    
    def set_threshold(self, threshold: int):
        """Threshold değerini güncelle."""
        self.threshold = max(0, min(255, threshold))
        self._logger.info(f"Threshold güncellendi: {self.threshold}")
    
    def set_area_limits(self, min_area: int, max_area: int):
        """Alan limitlerini güncelle."""
        self.min_area = max(1, min_area)
        self.max_area = max(self.min_area + 1, max_area)
        self._logger.info(f"Alan limitleri güncellendi: {self.min_area}-{self.max_area}")
    
    def get_processed_frame(self) -> Optional[np.ndarray]:
        """Son işlenmiş frame'i al."""
        with self._frame_lock:
            if self._processed_frame is not None:
                return self._processed_frame.copy()
        return None
    
    def get_statistics(self) -> dict:
        """İstatistikleri döndür."""
        return {
            'frame_count': self._frame_count,
            'detection_count': self._detection_count,
            'fps': self._fps,
            'detection_rate': (self._detection_count / self._frame_count * 100) 
                            if self._frame_count > 0 else 0
        }
    
    def __del__(self):
        """Destructor - kamerayı kapat."""
        self.stop()


# =============================================================================
# KALMAN FILTER
# =============================================================================

class KalmanTracker:
    """
    Lazer pozisyonu için Kalman filtresi.
    
    Gürültülü ölçümleri yumuşatır ve kısa süreli kayıplarda
    pozisyon tahmini yapar.
    """
    
    def __init__(self, process_noise: float = 0.01, 
                 measurement_noise: float = 0.1):
        """
        Kalman tracker oluştur.
        
        Args:
            process_noise: Süreç gürültüsü
            measurement_noise: Ölçüm gürültüsü
        """
        self.process_noise = process_noise
        self.measurement_noise = measurement_noise
        
        # Durum: [x, y, vx, vy]
        self.state = np.zeros(4)
        
        # Kovaryans matrisi
        self.P = np.eye(4) * 1000
        
        # Durum geçiş matrisi (sabit hız modeli)
        self.F = np.array([
            [1, 0, 1, 0],
            [0, 1, 0, 1],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ], dtype=np.float64)
        
        # Ölçüm matrisi (sadece x, y ölçüyoruz)
        self.H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ], dtype=np.float64)
        
        # Süreç gürültüsü matrisi
        self.Q = np.eye(4) * process_noise
        
        # Ölçüm gürültüsü matrisi
        self.R = np.eye(2) * measurement_noise
        
        # İlk ölçüm alındı mı?
        self._initialized = False
    
    def predict(self) -> Tuple[float, float]:
        """Bir sonraki pozisyonu tahmin et."""
        self.state = self.F @ self.state
        self.P = self.F @ self.P @ self.F.T + self.Q
        return (self.state[0], self.state[1])
    
    def update(self, measurement: Tuple[float, float]) -> Tuple[float, float]:
        """Ölçümle durumu güncelle."""
        if not self._initialized:
            self.state[0] = measurement[0]
            self.state[1] = measurement[1]
            self._initialized = True
            return measurement
        
        self.predict()
        
        z = np.array(measurement)
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        y = z - self.H @ self.state
        self.state = self.state + K @ y
        I = np.eye(4)
        self.P = (I - K @ self.H) @ self.P
        
        return (self.state[0], self.state[1])
    
    def reset(self):
        """Filtreyi sıfırla."""
        self.state = np.zeros(4)
        self.P = np.eye(4) * 1000
        self._initialized = False


# =============================================================================
# TEST
# =============================================================================

if __name__ == '__main__':
    logging.basicConfig(level=logging.DEBUG)
    
    print("Laser Detector Test")
    print("=" * 50)
    
    detector = LaserDetector(
        resolution=(640, 480),
        threshold=200,
        min_area=5,
        max_area=500
    )
    
    if detector.start():
        print("✅ Kamera başlatıldı")
        print("\n📷 Test başlıyor (ESC ile çık)...")
        
        while True:
            frame = detector.capture_frame()
            if frame is None:
                continue
            
            pos, processed = detector.detect_laser(frame)
            
            if pos:
                print(f"🔴 Lazer: X={pos[0]}, Y={pos[1]}")
            
            cv2.imshow('Laser Detection', processed)
            
            if cv2.waitKey(1) & 0xFF == 27:
                break
        
        cv2.destroyAllWindows()
        stats = detector.get_statistics()
        print(f"\n📊 Stats: {stats}")
        detector.stop()
        print("\n✅ Test tamamlandı")
    else:
        print("❌ Kamera başlatılamadı")
