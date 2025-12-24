#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
===============================================================================
PID_CONTROLLER.PY - PID Kontrolcü Modülü
===============================================================================
Discrete PID kontrolcü implementasyonu.

PID Formülü:
  u(t) = Kp * e(t) + Ki * ∫e(t)dt + Kd * de(t)/dt

Özellikler:
  - Anti-windup (integral sınırlama)
  - Derivative kick önleme
  - Output sınırlama
  - Otomatik zaman hesaplama

Kullanım:
  >>> pid = PIDController(kp=0.1, ki=0.01, kd=0.05)
  >>> output = pid.compute(error)

Not:
  - Pozitif error → Pozitif output (varsayılan)
  - reverse=True ile ters çevrilebilir
===============================================================================
"""

import time
import logging
from typing import Optional


class PIDController:
    """
    PID Kontrolcü sınıfı.
    
    Attributes:
        kp (float): Proportional gain
        ki (float): Integral gain
        kd (float): Derivative gain
        output_min (float): Minimum çıkış değeri
        output_max (float): Maksimum çıkış değeri
    
    Example:
        >>> pid = PIDController(kp=0.1, ki=0.01, kd=0.05)
        >>> while True:
        ...     error = setpoint - measurement
        ...     output = pid.compute(error)
        ...     actuator.set(output)
    """
    
    def __init__(self, kp: float = 1.0, ki: float = 0.0, kd: float = 0.0,
                 output_min: float = -1.0, output_max: float = 1.0,
                 integral_max: float = None,
                 sample_time: float = None,
                 reverse: bool = False,
                 name: str = "PID"):
        """
        PID kontrolcü oluştur.
        
        Args:
            kp: Proportional gain (P terimi)
                - Yüksek değer: Hızlı tepki, salınım riski
                - Düşük değer: Yavaş tepki, stabil
            
            ki: Integral gain (I terimi)
                - Sabit hataları (offset) düzeltir
                - Yüksek değer: Wind-up riski
                - 0: Integral kapalı
            
            kd: Derivative gain (D terimi)
                - Ani değişimleri sönümler
                - Gürültüye hassas olabilir
                - 0: Derivative kapalı
            
            output_min: Minimum çıkış değeri
            output_max: Maksimum çıkış değeri
            integral_max: Maksimum integral değeri (anti-windup)
                         None ise output_max kullanılır
            sample_time: Örnekleme süresi (saniye)
                        None ise otomatik hesaplanır
            reverse: True ise çıkış ters çevrilir
            name: Debug için kontrolcü adı
        """
        # Kazançlar
        self.kp = kp
        self.ki = ki
        self.kd = kd
        
        # Output limitleri
        self.output_min = output_min
        self.output_max = output_max
        
        # Integral limiti (anti-windup)
        self.integral_max = integral_max if integral_max else abs(output_max)
        
        # Örnekleme süresi
        self.sample_time = sample_time  # None = otomatik
        
        # Yön
        self.reverse = reverse
        
        # İsim (debug için)
        self.name = name
        
        # İç durumlar
        self._integral = 0.0          # Integral biriktirici
        self._prev_error = 0.0        # Önceki hata (derivative için)
        self._prev_measurement = None # Önceki ölçüm (derivative kick önleme)
        self._last_time = None        # Son hesaplama zamanı
        self._first_run = True        # İlk çalışma mı?
        
        # Logger
        self._logger = logging.getLogger(__name__)
    
    # =========================================================================
    # MAIN COMPUTE
    # =========================================================================
    
    def compute(self, error: float, measurement: float = None) -> float:
        """
        PID çıkışını hesapla.
        
        Args:
            error: Hata değeri (setpoint - measurement)
            measurement: Ölçüm değeri (derivative kick önleme için, opsiyonel)
        
        Returns:
            float: PID çıkışı (output_min ile output_max arasında)
        
        Note:
            - Bu fonksiyon düzenli aralıklarla çağrılmalı
            - sample_time verilmemişse otomatik hesaplanır
        """
        # Zaman farkını hesapla
        current_time = time.time()
        
        if self._last_time is None:
            # İlk çalışma
            dt = self.sample_time if self.sample_time else 0.05  # 50ms varsayılan
            self._first_run = True
        else:
            dt = current_time - self._last_time
            if dt <= 0:
                dt = 0.001  # Minimum dt
        
        self._last_time = current_time
        
        # Yön düzeltme
        if self.reverse:
            error = -error
        
        # ---------------------------------------------------------------------
        # P TERİMİ (Proportional)
        # ---------------------------------------------------------------------
        # Anlık hataya orantılı
        p_term = self.kp * error
        
        # ---------------------------------------------------------------------
        # I TERİMİ (Integral)
        # ---------------------------------------------------------------------
        # Hata birikimini hesapla (Riemann sum)
        if self.ki != 0:
            self._integral += error * dt
            
            # Anti-windup: Integral'i sınırla
            self._integral = self._clamp(self._integral, 
                                         -self.integral_max, 
                                         self.integral_max)
        
        i_term = self.ki * self._integral
        
        # ---------------------------------------------------------------------
        # D TERİMİ (Derivative)
        # ---------------------------------------------------------------------
        # Hata değişim hızı
        if self.kd != 0 and not self._first_run:
            if measurement is not None and self._prev_measurement is not None:
                # Derivative on measurement (kick önleme)
                # Ölçüm değişimine göre hesapla (setpoint değişimini yoksay)
                d_error = -(measurement - self._prev_measurement) / dt
            else:
                # Derivative on error
                d_error = (error - self._prev_error) / dt
            
            d_term = self.kd * d_error
        else:
            d_term = 0.0
        
        # Önceki değerleri kaydet
        self._prev_error = error
        if measurement is not None:
            self._prev_measurement = measurement
        self._first_run = False
        
        # ---------------------------------------------------------------------
        # TOPLAM ÇIKIŞ
        # ---------------------------------------------------------------------
        output = p_term + i_term + d_term
        
        # Output sınırlama
        output = self._clamp(output, self.output_min, self.output_max)
        
        return output
    
    # =========================================================================
    # CONFIGURATION
    # =========================================================================
    
    def set_gains(self, kp: float = None, ki: float = None, kd: float = None):
        """
        PID kazançlarını güncelle.
        
        Args:
            kp: Yeni Kp değeri (None ise değişmez)
            ki: Yeni Ki değeri (None ise değişmez)
            kd: Yeni Kd değeri (None ise değişmez)
        """
        if kp is not None:
            self.kp = kp
        if ki is not None:
            self.ki = ki
        if kd is not None:
            self.kd = kd
        
        self._logger.debug(f"{self.name} kazançları güncellendi: "
                          f"Kp={self.kp}, Ki={self.ki}, Kd={self.kd}")
    
    def set_output_limits(self, output_min: float, output_max: float):
        """
        Çıkış limitlerini güncelle.
        
        Args:
            output_min: Minimum çıkış
            output_max: Maksimum çıkış
        """
        if output_min >= output_max:
            self._logger.warning("output_min >= output_max!")
            return
        
        self.output_min = output_min
        self.output_max = output_max
    
    def set_integral_max(self, integral_max: float):
        """
        Integral limitini güncelle (anti-windup).
        
        Args:
            integral_max: Maksimum integral değeri
        """
        self.integral_max = abs(integral_max)
    
    def set_sample_time(self, sample_time: float):
        """
        Örnekleme süresini ayarla.
        
        Args:
            sample_time: Örnekleme süresi (saniye)
        """
        if sample_time <= 0:
            self._logger.warning("sample_time <= 0!")
            return
        
        self.sample_time = sample_time
    
    # =========================================================================
    # RESET
    # =========================================================================
    
    def reset(self):
        """
        PID durumunu sıfırla.
        
        Integral biriktiriciyi ve önceki hata değerlerini temizler.
        Yeni bir kontrol sekansı başlamadan önce çağrılmalı.
        """
        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_measurement = None
        self._last_time = None
        self._first_run = True
        
        self._logger.debug(f"{self.name} sıfırlandı")
    
    def reset_integral(self):
        """
        Sadece integral terimini sıfırla.
        
        Wind-up durumunda kullanılabilir.
        """
        self._integral = 0.0
        self._logger.debug(f"{self.name} integral sıfırlandı")
    
    # =========================================================================
    # UTILITY
    # =========================================================================
    
    @staticmethod
    def _clamp(value: float, min_val: float, max_val: float) -> float:
        """
        Değeri belirli aralıkta sınırla.
        
        Args:
            value: Sınırlanacak değer
            min_val: Minimum değer
            max_val: Maksimum değer
        
        Returns:
            float: Sınırlanmış değer
        """
        return max(min_val, min(max_val, value))
    
    def get_terms(self) -> dict:
        """
        PID terimlerinin son değerlerini al (debug için).
        
        Returns:
            dict: {'p': p_term, 'i': i_term, 'd': d_term}
        """
        return {
            'p': self.kp * self._prev_error,
            'i': self.ki * self._integral,
            'd': 0.0,  # D terimi kaydedilmiyor, anlık hesaplanıyor
            'integral': self._integral,
            'error': self._prev_error
        }
    
    def __repr__(self) -> str:
        """String representation."""
        return (f"PIDController(name={self.name}, kp={self.kp}, ki={self.ki}, "
                f"kd={self.kd}, limits=[{self.output_min}, {self.output_max}])")


# =============================================================================
# UTILITY FUNCTIONS
# =============================================================================

def tune_pid_ziegler_nichols(ku: float, tu: float, controller_type: str = 'PID') -> dict:
    """
    Ziegler-Nichols yöntemi ile PID kazançlarını hesapla.
    
    Args:
        ku: Ultimate gain (salınım başladığı Kp değeri)
        tu: Ultimate period (salınım periyodu, saniye)
        controller_type: 'P', 'PI', veya 'PID'
    
    Returns:
        dict: {'kp': kp, 'ki': ki, 'kd': kd}
    
    Note:
        Ziegler-Nichols yöntemi:
        1. Ki ve Kd'yi 0 yap
        2. Kp'yi salınım başlayana kadar artır (= Ku)
        3. Salınım periyodunu ölç (= Tu)
        4. Aşağıdaki formülleri kullan
    """
    if controller_type == 'P':
        return {'kp': 0.5 * ku, 'ki': 0.0, 'kd': 0.0}
    
    elif controller_type == 'PI':
        return {'kp': 0.45 * ku, 'ki': 0.54 * ku / tu, 'kd': 0.0}
    
    elif controller_type == 'PID':
        return {'kp': 0.6 * ku, 'ki': 1.2 * ku / tu, 'kd': 0.075 * ku * tu}
    
    else:
        raise ValueError(f"Unknown controller type: {controller_type}")


# =============================================================================
# TEST
# =============================================================================

if __name__ == '__main__':
    """
    Test script - basit simülasyon ile PID testi.
    """
    logging.basicConfig(level=logging.DEBUG)
    
    print("PID Controller Test")
    print("=" * 50)
    
    # PID oluştur
    pid = PIDController(
        kp=0.5,
        ki=0.1,
        kd=0.05,
        output_min=-1.0,
        output_max=1.0,
        name="TestPID"
    )
    
    print(f"📐 {pid}")
    
    # Basit simülasyon: Setpoint = 0, başlangıç = 10
    setpoint = 0.0
    position = 10.0
    
    print("\n🎮 Simülasyon başlıyor...")
    print(f"{'Step':>4} | {'Position':>10} | {'Error':>10} | {'Output':>10}")
    print("-" * 50)
    
    for i in range(50):
        # Hata hesapla
        error = setpoint - position
        
        # PID çıkışı
        output = pid.compute(error)
        
        # Sistemi güncelle (basit 1. derece sistem)
        position += output * 0.5
        
        # Her 5 adımda bir yazdır
        if i % 5 == 0:
            print(f"{i:>4} | {position:>10.4f} | {error:>10.4f} | {output:>10.4f}")
        
        time.sleep(0.01)
    
    print("-" * 50)
    print(f"\n📊 Son durum:")
    print(f"   Position: {position:.4f}")
    print(f"   Error: {setpoint - position:.4f}")
    
    terms = pid.get_terms()
    print(f"   PID Terms: {terms}")
    
    # Reset test
    pid.reset()
    print(f"\n🔄 PID sıfırlandı")
    print(f"   Terms after reset: {pid.get_terms()}")
    
    # Ziegler-Nichols test
    print("\n📐 Ziegler-Nichols hesaplama:")
    zn = tune_pid_ziegler_nichols(ku=1.0, tu=0.5, controller_type='PID')
    print(f"   Ku=1.0, Tu=0.5 → {zn}")
    
    print("\n✅ Test tamamlandı")
