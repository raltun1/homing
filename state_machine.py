#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
===============================================================================
STATE_MACHINE.PY - Finite State Machine (FSM)
===============================================================================
Precision landing sistemi için durum makinesi.

Durumlar (States):
  IDLE      → Sistem pasif, beklemede
  SEARCHING → Lazer aranıyor
  TRACKING  → Lazer bulundu, takip ediliyor (onay bekleniyor)
  APPROACH  → Lazer onaylandı, yaklaşma başladı
  LANDING   → Final iniş (düşük irtifa)
  LOST      → Lazer kayboldu
  COMPLETE  → İniş tamamlandı

Geçişler (Transitions):
  IDLE → SEARCHING      : enable() çağrıldığında
  SEARCHING → TRACKING  : Lazer tespit edildiğinde
  TRACKING → APPROACH   : Lazer LASER_DETECTION_TIME süre görüldüğünde
  APPROACH → LANDING    : Yükseklik LANDING_THRESHOLD altına düştüğünde
  APPROACH → LOST       : Lazer LASER_LOST_TIMEOUT süre kaybolduğunda
  LANDING → COMPLETE    : İniş sensörü tetiklendiğinde
  LOST → TRACKING       : Lazer tekrar görüldüğünde
  * → IDLE              : disable() çağrıldığında

                    ┌─────────┐
                    │  IDLE   │
                    └────┬────┘
                         │ enable()
                    ┌────▼────┐
                    │SEARCHING│
                    └────┬────┘
                         │ laser detected
                    ┌────▼────┐
              ┌─────│TRACKING │◄────────┐
              │     └────┬────┘         │
              │          │ confirmed    │ laser found
              │     ┌────▼────┐         │
              │     │APPROACH │─────────┤
              │     └────┬────┘         │
              │          │ low alt      │
              │     ┌────▼────┐    ┌────┴───┐
              │     │ LANDING │    │  LOST  │
              │     └────┬────┘    └────────┘
              │          │ touched down
              │     ┌────▼────┐
              └────►│COMPLETE │
                    └─────────┘
===============================================================================
"""

import time
import logging
import threading
from enum import Enum, auto
from typing import Optional, Tuple, Callable


class SystemState(Enum):
    """
    Sistem durumları.
    
    Her durum sistemin farklı bir davranış modunu temsil eder.
    """
    IDLE = auto()       # Pasif, beklemede
    SEARCHING = auto()  # Lazer aranıyor
    TRACKING = auto()   # Lazer bulundu, onay bekleniyor
    APPROACH = auto()   # Yaklaşma modu
    LANDING = auto()    # Final iniş
    LOST = auto()       # Lazer kayıp
    COMPLETE = auto()   # İniş tamamlandı


class StateMachine:
    """
    Finite State Machine (FSM) sınıfı.
    
    Precision landing sistemi için durum yönetimi sağlar.
    
    Attributes:
        detection_time (float): Lazer onay süresi
        lost_timeout (float): Lazer kayıp timeout
        start_height (float): Precision landing başlama yüksekliği
        landing_height (float): Final landing yüksekliği
    
    Example:
        >>> fsm = StateMachine(detection_time=2.0, lost_timeout=3.0)
        >>> fsm.enable()
        >>> fsm.update(laser_detected=True, laser_position=(320, 240), altitude=10.0)
        >>> print(fsm.get_state())
    """
    
    def __init__(self, detection_time: float = 2.0,
                 lost_timeout: float = 3.0,
                 start_height: float = 15.0,
                 landing_height: float = 0.8):
        """
        State machine oluştur.
        
        Args:
            detection_time: Lazer onay süresi (saniye)
                           Lazer bu süre boyunca görülmeli
            lost_timeout: Lazer kayıp timeout (saniye)
                         Lazer bu süre görülmezse LOST durumuna geç
            start_height: Precision landing başlama yüksekliği (metre)
            landing_height: Final landing yüksekliği (metre)
        """
        # Parametreler
        self.detection_time = detection_time
        self.lost_timeout = lost_timeout
        self.start_height = start_height
        self.landing_height = landing_height
        
        # Mevcut durum
        self._state = SystemState.IDLE
        self._prev_state = SystemState.IDLE
        
        # Zamanlayıcılar
        self._laser_first_seen = 0.0      # Lazer ilk görüldüğü zaman
        self._laser_last_seen = 0.0       # Lazer son görüldüğü zaman
        self._state_enter_time = 0.0      # Duruma giriş zamanı
        
        # Lazer pozisyonu
        self._laser_position: Optional[Tuple[int, int]] = None
        
        # Yükseklik
        self._altitude = 0.0
        
        # Callback fonksiyonları
        self._on_state_change: Optional[Callable] = None
        
        # Thread safety
        self._lock = threading.Lock()
        
        # Logger
        self._logger = logging.getLogger(__name__)
    
    # =========================================================================
    # STATE ACCESS
    # =========================================================================
    
    def get_state(self) -> SystemState:
        """
        Mevcut durumu al.
        
        Returns:
            SystemState: Mevcut durum
        """
        with self._lock:
            return self._state
    
    def get_state_name(self) -> str:
        """
        Mevcut durum adını al.
        
        Returns:
            str: Durum adı
        """
        with self._lock:
            return self._state.name
    
    def get_state_info(self) -> dict:
        """
        Durum bilgilerini al.
        
        Returns:
            dict: Durum detayları
        """
        with self._lock:
            now = time.time()
            
            return {
                'state': self._state.name,
                'prev_state': self._prev_state.name,
                'state_duration': now - self._state_enter_time,
                'laser_detected': self._laser_position is not None,
                'laser_position': self._laser_position,
                'altitude': self._altitude,
                'laser_visible_time': (now - self._laser_first_seen 
                                       if self._laser_first_seen > 0 else 0),
                'laser_lost_time': (now - self._laser_last_seen 
                                   if self._laser_last_seen > 0 else 0)
            }
    
    # =========================================================================
    # STATE TRANSITIONS
    # =========================================================================
    
    def _change_state(self, new_state: SystemState):
        """
        Durumu değiştir.
        
        Args:
            new_state: Yeni durum
        """
        if new_state == self._state:
            return
        
        old_state = self._state
        self._prev_state = old_state
        self._state = new_state
        self._state_enter_time = time.time()
        
        self._logger.info(f"🔄 Durum değişti: {old_state.name} → {new_state.name}")
        
        # Callback çağır
        if self._on_state_change:
            try:
                self._on_state_change(old_state, new_state)
            except Exception as e:
                self._logger.error(f"State change callback hatası: {e}")
    
    def enable(self):
        """
        Sistemi aktif et.
        
        IDLE → SEARCHING geçişi yapar.
        """
        with self._lock:
            if self._state == SystemState.IDLE:
                self._reset_timers()
                self._change_state(SystemState.SEARCHING)
                self._logger.info("✅ Sistem aktif edildi")
    
    def disable(self):
        """
        Sistemi pasif yap.
        
        Herhangi bir durumdan → IDLE geçişi yapar.
        """
        with self._lock:
            self._reset_timers()
            self._change_state(SystemState.IDLE)
            self._logger.info("⏹️ Sistem durduruldu")
    
    def _reset_timers(self):
        """
        Tüm zamanlayıcıları sıfırla.
        """
        self._laser_first_seen = 0.0
        self._laser_last_seen = 0.0
        self._laser_position = None
    
    # =========================================================================
    # UPDATE
    # =========================================================================
    
    def update(self, laser_detected: bool, 
               laser_position: Optional[Tuple[int, int]] = None,
               altitude: float = 0.0):
        """
        Durumu güncelle.
        
        Her frame'de çağrılmalı.
        
        Args:
            laser_detected: Lazer tespit edildi mi?
            laser_position: Lazer pozisyonu (x, y) veya None
            altitude: Mevcut yükseklik (metre)
        """
        with self._lock:
            now = time.time()
            
            # Değerleri güncelle
            self._altitude = altitude
            if laser_detected:
                self._laser_position = laser_position
                self._laser_last_seen = now
                
                # İlk görülme zamanı
                if self._laser_first_seen == 0:
                    self._laser_first_seen = now
            else:
                self._laser_position = None
            
            # Durum makinesini işle
            self._process_state(laser_detected, now)
    
    def _process_state(self, laser_detected: bool, now: float):
        """
        Durum mantığını işle.
        
        Args:
            laser_detected: Lazer tespit edildi mi?
            now: Şimdiki zaman
        """
        state = self._state
        
        # -----------------------------------------------------------------
        # IDLE: Pasif durumda bekle
        # -----------------------------------------------------------------
        if state == SystemState.IDLE:
            # enable() ile aktif edilene kadar bekle
            pass
        
        # -----------------------------------------------------------------
        # SEARCHING: Lazer ara
        # -----------------------------------------------------------------
        elif state == SystemState.SEARCHING:
            if laser_detected:
                # Lazer bulundu → TRACKING
                self._change_state(SystemState.TRACKING)
        
        # -----------------------------------------------------------------
        # TRACKING: Lazer onayı bekle
        # -----------------------------------------------------------------
        elif state == SystemState.TRACKING:
            if not laser_detected:
                # Lazer kayboldu → Tekrar aramaya dön
                self._reset_timers()
                self._change_state(SystemState.SEARCHING)
            
            elif now - self._laser_first_seen >= self.detection_time:
                # Lazer yeterince uzun süre görüldü → APPROACH
                self._logger.info(f"✅ Lazer onaylandı ({self.detection_time}s)")
                self._change_state(SystemState.APPROACH)
        
        # -----------------------------------------------------------------
        # APPROACH: Lazere doğru yaklaş
        # -----------------------------------------------------------------
        elif state == SystemState.APPROACH:
            if laser_detected:
                # Lazer görünüyor, yüksekliği kontrol et
                if self._altitude <= self.landing_height:
                    # Yeterince alçaldı → LANDING
                    self._logger.info(f"🛬 Final iniş yüksekliği ({self.landing_height}m)")
                    self._change_state(SystemState.LANDING)
            
            else:
                # Lazer kayıp - timeout kontrolü
                lost_duration = now - self._laser_last_seen
                
                if lost_duration >= self.lost_timeout:
                    # Çok uzun süre kayıp → LOST
                    self._logger.warning(f"⚠️ Lazer kayıp ({self.lost_timeout}s)")
                    self._change_state(SystemState.LOST)
        
        # -----------------------------------------------------------------
        # LANDING: Final iniş
        # -----------------------------------------------------------------
        elif state == SystemState.LANDING:
            # Yere temas kontrolü (şimdilik basit yükseklik kontrolü)
            if self._altitude <= 0.1:
                self._logger.info("🎉 İniş tamamlandı!")
                self._change_state(SystemState.COMPLETE)
            
            # Lazer kayıp kontrolü
            if not laser_detected:
                lost_duration = now - self._laser_last_seen
                if lost_duration >= self.lost_timeout:
                    self._logger.warning("⚠️ Final inişte lazer kayıp!")
                    self._change_state(SystemState.LOST)
        
        # -----------------------------------------------------------------
        # LOST: Lazer kayıp, bekle veya kurtarma
        # -----------------------------------------------------------------
        elif state == SystemState.LOST:
            if laser_detected:
                # Lazer tekrar bulundu → TRACKING
                self._logger.info("🔴 Lazer tekrar bulundu")
                self._laser_first_seen = now
                self._change_state(SystemState.TRACKING)
        
        # -----------------------------------------------------------------
        # COMPLETE: İniş tamamlandı
        # -----------------------------------------------------------------
        elif state == SystemState.COMPLETE:
            # Otomatik reset (opsiyonel)
            pass
    
    # =========================================================================
    # CALLBACKS
    # =========================================================================
    
    def set_on_state_change(self, callback: Callable):
        """
        Durum değişikliği callback'i ayarla.
        
        Args:
            callback: Callback fonksiyonu (old_state, new_state parametreleri)
        
        Example:
            >>> def on_change(old, new):
            ...     print(f"State: {old.name} → {new.name}")
            >>> fsm.set_on_state_change(on_change)
        """
        self._on_state_change = callback
    
    # =========================================================================
    # UTILITY
    # =========================================================================
    
    def is_active(self) -> bool:
        """
        Sistem aktif mi?
        
        Returns:
            bool: IDLE değilse True
        """
        with self._lock:
            return self._state != SystemState.IDLE
    
    def is_tracking(self) -> bool:
        """
        Lazer takip ediliyor mu?
        
        Returns:
            bool: TRACKING veya APPROACH durumundaysa True
        """
        with self._lock:
            return self._state in [SystemState.TRACKING, SystemState.APPROACH]
    
    def is_landing(self) -> bool:
        """
        İniş modunda mı?
        
        Returns:
            bool: LANDING durumundaysa True
        """
        with self._lock:
            return self._state == SystemState.LANDING
    
    def is_complete(self) -> bool:
        """
        İniş tamamlandı mı?
        
        Returns:
            bool: COMPLETE durumundaysa True
        """
        with self._lock:
            return self._state == SystemState.COMPLETE
    
    def get_laser_tracking_time(self) -> float:
        """
        Lazer ne kadar süredir görülüyor?
        
        Returns:
            float: Süre (saniye)
        """
        with self._lock:
            if self._laser_first_seen > 0:
                return time.time() - self._laser_first_seen
            return 0.0
    
    def __repr__(self) -> str:
        """String representation."""
        return f"StateMachine(state={self._state.name})"


# =============================================================================
# TEST
# =============================================================================

if __name__ == '__main__':
    """
    Test script - FSM simülasyonu.
    """
    logging.basicConfig(level=logging.DEBUG)
    
    print("State Machine Test")
    print("=" * 50)
    
    # FSM oluştur
    fsm = StateMachine(
        detection_time=2.0,
        lost_timeout=3.0,
        start_height=15.0,
        landing_height=0.8
    )
    
    # Callback ayarla
    def on_state_change(old_state, new_state):
        print(f"   [CALLBACK] {old_state.name} → {new_state.name}")
    
    fsm.set_on_state_change(on_state_change)
    
    print(f"📐 Başlangıç durumu: {fsm.get_state().name}")
    
    # Simülasyon
    print("\n🎮 Simülasyon başlıyor...")
    
    # Senaryo: 
    # 1. Sistem aktif et
    # 2. Lazer bul
    # 3. Onay bekle
    # 4. Yaklaş
    # 5. İniş
    
    # 1. Aktif et
    print("\n[1] Sistem aktif ediliyor...")
    fsm.enable()
    print(f"   Durum: {fsm.get_state().name}")
    
    # 2. Lazer yok (arama)
    print("\n[2] Lazer aranıyor...")
    for _ in range(5):
        fsm.update(laser_detected=False, altitude=10.0)
        time.sleep(0.1)
    print(f"   Durum: {fsm.get_state().name}")
    
    # 3. Lazer bulundu
    print("\n[3] Lazer bulundu...")
    fsm.update(laser_detected=True, laser_position=(320, 240), altitude=10.0)
    print(f"   Durum: {fsm.get_state().name}")
    
    # 4. Onay süresi (2 saniye)
    print("\n[4] Lazer onay süresi bekleniyor (2s)...")
    for i in range(25):
        fsm.update(laser_detected=True, laser_position=(320, 240), altitude=10.0)
        time.sleep(0.1)
        if i % 5 == 0:
            print(f"   {i/10:.1f}s - Durum: {fsm.get_state().name}")
    
    # 5. Yaklaşma (yükseklik azalıyor)
    print("\n[5] Yaklaşma...")
    altitude = 10.0
    while altitude > 0.5:
        altitude -= 0.3
        fsm.update(laser_detected=True, laser_position=(320, 240), altitude=altitude)
        print(f"   Alt: {altitude:.1f}m - Durum: {fsm.get_state().name}")
        time.sleep(0.1)
    
    # 6. Final iniş
    print("\n[6] Final iniş...")
    fsm.update(laser_detected=True, laser_position=(320, 240), altitude=0.05)
    print(f"   Durum: {fsm.get_state().name}")
    
    # Durum bilgileri
    print("\n📊 Durum Bilgileri:")
    info = fsm.get_state_info()
    for key, value in info.items():
        print(f"   {key}: {value}")
    
    # Devre dışı bırak
    print("\n[7] Sistem kapatılıyor...")
    fsm.disable()
    print(f"   Durum: {fsm.get_state().name}")
    
    print("\n✅ Test tamamlandı")
