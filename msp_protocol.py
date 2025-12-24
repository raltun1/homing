#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
===============================================================================
MSP_PROTOCOL.PY - MSP V2 Protocol Implementation
===============================================================================
MultiWii Serial Protocol Version 2 (MSP V2) implementasyonu.
INAV flight controller ile seri port üzerinden haberleşme sağlar.

MSP V2 Frame Yapısı:
  $X<direction><flag><function><size><payload><crc>
  
  - $  (0x24) : Start byte
  - X  (0x58) : MSP V2 identifier
  - <  (0x3C) : Request (to FC)
  - >  (0x3E) : Response (from FC)
  - !  (0x21) : Error
  - flag      : 0x00 (normal)
  - function  : 2 bytes (little endian)
  - size      : 2 bytes (little endian)
  - payload   : variable
  - crc       : CRC8 DVB-S2

Kullanılan MSP Fonksiyonları:
  - MSP_SET_RAW_RC (200) : RC kanallarını override et
  - MSP_ALTITUDE (109)   : Yükseklik bilgisi al
  - MSP_STATUS (101)     : FC durumu al
  - MSP_FC_VARIANT (2)   : FC bilgisi al

Referanslar:
  - INAV GitHub: github.com/iNavFlight/inav
  - MSP Protocol: github.com/iNavFlight/inav/wiki/MSP-V2
===============================================================================
"""

import struct
import time
import logging
import threading
from typing import Optional, Tuple, Dict, List

# Serial port kütüphanesi
try:
    import serial
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False
    logging.warning("pyserial kurulu değil! Simülasyon modunda çalışılacak.")

# =============================================================================
# MSP CONSTANTS
# =============================================================================

# MSP V2 Frame markers
MSP_V2_START = 0x24      # '$'
MSP_V2_IDENT = 0x58      # 'X'
MSP_V2_REQUEST = 0x3C    # '<'
MSP_V2_RESPONSE = 0x3E   # '>'
MSP_V2_ERROR = 0x21      # '!'

# MSP Function IDs
MSP_FC_VARIANT = 2       # FC tanımlayıcı (INAV, BTFL, etc.)
MSP_FC_VERSION = 3       # FC versiyon
MSP_STATUS = 101         # FC durumu (armed, mode, etc.)
MSP_ALTITUDE = 109       # Yükseklik (cm)
MSP_SET_RAW_RC = 200     # RC kanallarını override et
MSP_RC = 105             # Mevcut RC değerlerini oku

# RC Channel limitleri
RC_MIN = 1000
RC_MAX = 2000
RC_MID = 1500

# Timeout değerleri
DEFAULT_TIMEOUT = 1.0    # saniye
READ_TIMEOUT = 0.1       # saniye


class MSPProtocol:
    """
    MSP V2 protokol sınıfı.
    
    Flight controller ile seri port üzerinden MSP V2 protokolü ile
    haberleşme sağlar.
    
    Attributes:
        port (str): Seri port adı (örn: '/dev/serial0')
        baudrate (int): Baudrate (varsayılan: 115200)
        simulation (bool): Simülasyon modu
    
    Example:
        >>> msp = MSPProtocol('/dev/serial0', 115200)
        >>> msp.connect()
        >>> msp.send_rc_override(1500, 1500, 1500, 1500)
    """
    
    def __init__(self, port: str = '/dev/serial0', baudrate: int = 115200,
                 simulation: bool = False):
        """
        MSP protokol nesnesini oluştur.
        
        Args:
            port: Seri port adı
            baudrate: Baudrate
            simulation: True ise gerçek port kullanılmaz
        """
        self.port = port
        self.baudrate = baudrate
        self.simulation = simulation
        
        # Serial port instance
        self._serial: Optional[serial.Serial] = None
        
        # Thread safety
        self._lock = threading.Lock()
        
        # Son gönderilen RC değerleri
        self._last_rc_channels = [RC_MID] * 18
        
        # İstatistikler
        self._tx_count = 0
        self._rx_count = 0
        self._error_count = 0
        
        # Logger
        self._logger = logging.getLogger(__name__)
    
    # =========================================================================
    # CONNECTION METHODS
    # =========================================================================
    
    def connect(self) -> bool:
        """
        Seri porta bağlan.
        
        Returns:
            bool: Başarılı ise True
        """
        if self.simulation:
            self._logger.info("Simülasyon modu - seri port kullanılmıyor")
            return True
        
        if not SERIAL_AVAILABLE:
            self._logger.error("pyserial kurulu değil!")
            return False
        
        try:
            self._serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=READ_TIMEOUT,
                write_timeout=DEFAULT_TIMEOUT
            )
            
            # Tamponları temizle
            self._serial.reset_input_buffer()
            self._serial.reset_output_buffer()
            
            # Bağlantıyı test et
            time.sleep(0.1)
            
            self._logger.info(f"Seri port bağlandı: {self.port} @ {self.baudrate}")
            return True
            
        except serial.SerialException as e:
            self._logger.error(f"Seri port hatası: {e}")
            return False
    
    def disconnect(self):
        """
        Seri port bağlantısını kapat.
        """
        if self._serial and self._serial.is_open:
            self._serial.close()
            self._logger.info("Seri port kapatıldı")
    
    def is_connected(self) -> bool:
        """
        Bağlantı durumunu kontrol et.
        """
        if self.simulation:
            return True
        return self._serial is not None and self._serial.is_open
    
    # =========================================================================
    # CRC CALCULATION
    # =========================================================================
    
    @staticmethod
    def _calculate_crc8_dvb_s2(data: bytes) -> int:
        """
        CRC8 DVB-S2 checksum hesapla.
        
        MSP V2 protokolü DVB-S2 standardı kullanır.
        Polynomial: 0xD5
        
        Args:
            data: CRC hesaplanacak veri
        
        Returns:
            int: 8-bit CRC değeri
        """
        crc = 0x00
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x80:
                    crc = ((crc << 1) ^ 0xD5) & 0xFF
                else:
                    crc = (crc << 1) & 0xFF
        return crc
    
    # =========================================================================
    # FRAME BUILDING
    # =========================================================================
    
    def _build_msp_v2_frame(self, function: int, payload: bytes = b'') -> bytes:
        """
        MSP V2 frame oluştur.
        
        Frame yapısı:
        [0] $     : 0x24
        [1] X     : 0x58 (MSP V2)
        [2] <     : 0x3C (request)
        [3] flag  : 0x00
        [4-5] func: function ID (little endian)
        [6-7] size: payload size (little endian)
        [8..] pay : payload
        [-1] crc  : CRC8 DVB-S2
        
        Args:
            function: MSP function ID
            payload: Payload verisi
        
        Returns:
            bytes: Oluşturulan frame
        """
        # Header
        header = bytes([MSP_V2_START, MSP_V2_IDENT, MSP_V2_REQUEST])
        
        # Flag (always 0 for normal messages)
        flag = bytes([0x00])
        
        # Function ID (2 bytes, little endian)
        func_bytes = struct.pack('<H', function)
        
        # Payload size (2 bytes, little endian)
        size_bytes = struct.pack('<H', len(payload))
        
        # CRC hesapla (flag + function + size + payload)
        crc_data = flag + func_bytes + size_bytes + payload
        crc = self._calculate_crc8_dvb_s2(crc_data)
        
        # Frame oluştur
        frame = header + crc_data + bytes([crc])
        
        return frame
    
    # =========================================================================
    # FRAME PARSING
    # =========================================================================
    
    def _parse_msp_v2_response(self, timeout: float = DEFAULT_TIMEOUT) -> Optional[Tuple[int, bytes]]:
        """
        MSP V2 cevabını oku ve parse et.
        
        Args:
            timeout: Okuma timeout süresi
        
        Returns:
            Tuple[function_id, payload] veya None (hata durumunda)
        """
        if self.simulation:
            return None
        
        if not self._serial:
            return None
        
        start_time = time.time()
        
        try:
            # Frame başlangıcını bul ($X>)
            while time.time() - start_time < timeout:
                # İlk byte oku
                byte = self._serial.read(1)
                if not byte:
                    continue
                
                if byte[0] != MSP_V2_START:
                    continue
                
                # İkinci byte
                byte = self._serial.read(1)
                if not byte or byte[0] != MSP_V2_IDENT:
                    continue
                
                # Üçüncü byte (direction)
                byte = self._serial.read(1)
                if not byte:
                    continue
                
                direction = byte[0]
                
                if direction == MSP_V2_ERROR:
                    self._logger.warning("MSP Error response alındı")
                    self._error_count += 1
                    return None
                
                if direction != MSP_V2_RESPONSE:
                    continue
                
                # Flag (1 byte)
                flag = self._serial.read(1)
                if not flag:
                    return None
                
                # Function ID (2 bytes)
                func_bytes = self._serial.read(2)
                if len(func_bytes) != 2:
                    return None
                function = struct.unpack('<H', func_bytes)[0]
                
                # Payload size (2 bytes)
                size_bytes = self._serial.read(2)
                if len(size_bytes) != 2:
                    return None
                payload_size = struct.unpack('<H', size_bytes)[0]
                
                # Payload
                payload = b''
                if payload_size > 0:
                    payload = self._serial.read(payload_size)
                    if len(payload) != payload_size:
                        return None
                
                # CRC
                crc_byte = self._serial.read(1)
                if not crc_byte:
                    return None
                received_crc = crc_byte[0]
                
                # CRC doğrula
                crc_data = flag + func_bytes + size_bytes + payload
                calculated_crc = self._calculate_crc8_dvb_s2(crc_data)
                
                if received_crc != calculated_crc:
                    self._logger.warning(f"CRC hatası! Beklenen: {calculated_crc}, "
                                        f"Alınan: {received_crc}")
                    self._error_count += 1
                    return None
                
                self._rx_count += 1
                return (function, payload)
            
            return None
            
        except serial.SerialException as e:
            self._logger.error(f"Serial okuma hatası: {e}")
            self._error_count += 1
            return None
    
    # =========================================================================
    # SEND / RECEIVE
    # =========================================================================
    
    def _send_frame(self, frame: bytes) -> bool:
        """
        Frame'i seri porta gönder.
        
        Args:
            frame: Gönderilecek frame
        
        Returns:
            bool: Başarılı ise True
        """
        if self.simulation:
            self._tx_count += 1
            return True
        
        if not self._serial:
            return False
        
        with self._lock:
            try:
                self._serial.write(frame)
                self._serial.flush()
                self._tx_count += 1
                return True
            except serial.SerialException as e:
                self._logger.error(f"Serial yazma hatası: {e}")
                self._error_count += 1
                return False
    
    def _send_and_receive(self, function: int, payload: bytes = b'',
                         timeout: float = DEFAULT_TIMEOUT) -> Optional[bytes]:
        """
        MSP komutu gönder ve cevap al.
        
        Args:
            function: MSP function ID
            payload: Gönderilecek payload
            timeout: Cevap bekleme süresi
        
        Returns:
            bytes: Cevap payload veya None
        """
        frame = self._build_msp_v2_frame(function, payload)
        
        with self._lock:
            if not self._send_frame(frame):
                return None
            
            result = self._parse_msp_v2_response(timeout)
            if result:
                recv_func, recv_payload = result
                if recv_func == function:
                    return recv_payload
        
        return None
    
    # =========================================================================
    # RC OVERRIDE
    # =========================================================================
    
    def send_rc_override(self, roll: int = RC_MID, pitch: int = RC_MID,
                        throttle: int = RC_MID, yaw: int = RC_MID,
                        aux1: int = RC_MID, aux2: int = RC_MID,
                        aux3: int = RC_MID, aux4: int = RC_MID) -> bool:
        """
        RC kanallarını MSP üzerinden override et.
        
        MSP_SET_RAW_RC (200) komutu kullanır.
        INAV'da MSP RC Override modu aktif olmalı!
        
        Kanal sıralaması (AETR):
          0: Roll (Aileron)
          1: Pitch (Elevator)
          2: Throttle
          3: Yaw (Rudder)
          4-7: AUX1-4
        
        Args:
            roll: Roll değeri (1000-2000)
            pitch: Pitch değeri (1000-2000)
            throttle: Throttle değeri (1000-2000)
            yaw: Yaw değeri (1000-2000)
            aux1-4: Yardımcı kanallar
        
        Returns:
            bool: Başarılı ise True
        
        Note:
            ÖNEMLİ: MSP gönderme hızı minimum 5Hz olmalı!
            5Hz altında INAV otomatik olarak RC receiver'a döner.
        """
        # Değerleri sınırla
        def clamp(val):
            return max(RC_MIN, min(RC_MAX, int(val)))
        
        # Kanal değerlerini hazırla (18 kanal için)
        # INAV 18 kanal bekler ama biz ilk 8'ini kullanıyoruz
        channels = [
            clamp(roll),
            clamp(pitch),
            clamp(throttle),
            clamp(yaw),
            clamp(aux1),
            clamp(aux2),
            clamp(aux3),
            clamp(aux4),
        ]
        
        # Geri kalan kanalları 1500 (nötr) yap
        while len(channels) < 18:
            channels.append(RC_MID)
        
        # Payload oluştur (18 kanal × 2 byte = 36 byte)
        # Little endian unsigned short format
        payload = struct.pack('<' + 'H' * 18, *channels)
        
        # Frame oluştur ve gönder
        frame = self._build_msp_v2_frame(MSP_SET_RAW_RC, payload)
        success = self._send_frame(frame)
        
        if success:
            self._last_rc_channels[:8] = channels[:8]
        
        return success
    
    def send_rc_channels(self, channels: List[int]) -> bool:
        """
        Tüm RC kanallarını tek seferde gönder.
        
        Args:
            channels: Kanal değerleri listesi (max 18)
        
        Returns:
            bool: Başarılı ise True
        """
        # 18 kanala tamamla
        ch = list(channels)
        while len(ch) < 18:
            ch.append(RC_MID)
        
        # Sınırla
        ch = [max(RC_MIN, min(RC_MAX, int(c))) for c in ch[:18]]
        
        payload = struct.pack('<' + 'H' * 18, *ch)
        frame = self._build_msp_v2_frame(MSP_SET_RAW_RC, payload)
        
        return self._send_frame(frame)
    
    # =========================================================================
    # TELEMETRY REQUESTS
    # =========================================================================
    
    def request_altitude(self) -> Optional[float]:
        """
        FC'den yükseklik bilgisi iste.
        
        MSP_ALTITUDE (109) komutu kullanır.
        
        Returns:
            float: Yükseklik (metre) veya None
        """
        if self.simulation:
            return 10.0  # Simülasyon için sabit değer
        
        response = self._send_and_receive(MSP_ALTITUDE)
        
        if response and len(response) >= 4:
            # İlk 4 byte: altitude in cm (signed int32)
            altitude_cm = struct.unpack('<i', response[:4])[0]
            return altitude_cm / 100.0  # cm → m
        
        return None
    
    def request_status(self) -> Tuple[Optional[bool], Optional[str]]:
        """
        FC'den durum bilgisi iste.
        
        MSP_STATUS (101) komutu kullanır.
        
        Returns:
            Tuple[armed, mode_name] veya (None, None)
        """
        if self.simulation:
            return (False, "DISARMED")
        
        response = self._send_and_receive(MSP_STATUS)
        
        if response and len(response) >= 11:
            # Byte 6: Flight mode flags
            # Byte 10: arming flags (bit 0 = armed)
            arming_flags = response[10] if len(response) > 10 else 0
            armed = bool(arming_flags & 0x01)
            
            # Flight mode (basitleştirilmiş)
            mode = "ARMED" if armed else "DISARMED"
            
            return (armed, mode)
        
        return (None, None)
    
    def request_fc_info(self) -> Optional[Dict]:
        """
        FC tanımlama bilgilerini iste.
        
        MSP_FC_VARIANT (2) ve MSP_FC_VERSION (3) kullanır.
        
        Returns:
            Dict: {'name': str, 'version': str} veya None
        """
        if self.simulation:
            return {'name': 'SIMULATION', 'version': '0.0.0'}
        
        info = {}
        
        # FC Variant (INAV, BTFL, etc.)
        response = self._send_and_receive(MSP_FC_VARIANT)
        if response:
            # 4 byte ASCII identifier
            info['name'] = response[:4].decode('ascii', errors='ignore').strip()
        
        # FC Version
        response = self._send_and_receive(MSP_FC_VERSION)
        if response and len(response) >= 3:
            major = response[0]
            minor = response[1]
            patch = response[2]
            info['version'] = f"{major}.{minor}.{patch}"
        
        return info if info else None
    
    def request_rc_channels(self) -> Optional[List[int]]:
        """
        Mevcut RC kanal değerlerini oku.
        
        MSP_RC (105) komutu kullanır.
        
        Returns:
            List[int]: Kanal değerleri veya None
        """
        if self.simulation:
            return self._last_rc_channels[:8]
        
        response = self._send_and_receive(MSP_RC)
        
        if response:
            # Her kanal 2 byte (little endian)
            num_channels = len(response) // 2
            channels = struct.unpack('<' + 'H' * num_channels, response)
            return list(channels)
        
        return None
    
    # =========================================================================
    # UTILITY METHODS
    # =========================================================================
    
    def get_statistics(self) -> Dict:
        """
        İletişim istatistiklerini döndür.
        """
        return {
            'tx_count': self._tx_count,
            'rx_count': self._rx_count,
            'error_count': self._error_count,
            'connected': self.is_connected()
        }
    
    def __del__(self):
        """
        Destructor - seri portu kapat.
        """
        self.disconnect()


# =============================================================================
# TEST / DEMO
# =============================================================================

if __name__ == '__main__':
    """
    Test script - doğrudan çalıştırıldığında.
    """
    logging.basicConfig(level=logging.DEBUG)
    
    print("MSP Protocol Test")
    print("=" * 50)
    
    # Simülasyon modunda test
    msp = MSPProtocol(simulation=True)
    
    # Bağlan
    if msp.connect():
        print("✅ Bağlantı başarılı")
        
        # FC bilgisi
        info = msp.request_fc_info()
        print(f"📡 FC: {info}")
        
        # Yükseklik
        alt = msp.request_altitude()
        print(f"📍 Altitude: {alt}m")
        
        # Durum
        armed, mode = msp.request_status()
        print(f"🔋 Armed: {armed}, Mode: {mode}")
        
        # RC Override test
        print("\n🎮 RC Override testi...")
        for i in range(10):
            roll = 1500 + (i * 10)
            success = msp.send_rc_override(roll=roll)
            print(f"   Roll={roll}, Success={success}")
            time.sleep(0.05)
        
        # İstatistikler
        stats = msp.get_statistics()
        print(f"\n📊 Stats: {stats}")
        
        msp.disconnect()
        print("\n✅ Test tamamlandı")
    else:
        print("❌ Bağlantı başarısız")
