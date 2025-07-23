"""UART-to-I2C Bridge Protocol Module"""

import serial
from enum import IntEnum
import threading
import queue
import time

class Command(IntEnum):
    I2C_WRITE = 0x01
    I2C_READ = 0x02
    GPIO_READ = 0x10
    GPIO_WRITE = 0x11
    PING = 0xFF

class Status(IntEnum):
    OK = 0x00
    ERROR = 0x01
    CRC_ERROR = 0x02

class UartToI2C:
    """Класс для работы с UART-to-I2C мостом.
    
    Публичные методы:
        - connect(port, baudrate=9600) -> bool
        - disconnect()
        - i2c_write(addr, cmd, data=None) -> bool
        - i2c_read(addr, cmd, length=1) -> bytes|None
        - gpio_read(pin) -> int|None
        - gpio_write(pin, state) -> bool
        - ping() -> bool
    """

    def __init__(self):
        """Инициализация моста (без подключения)"""
        self._ser = None
        self._use_crc = True
        self._rx_queue = queue.Queue()
        self._running = False
        self._packet_id = 0
        #self._byte_delay = 0.001  # 1ms задержка между байтами

    def connect(self, port, baudrate=9600, byte_delay=0.002):
        """Подключиться к устройству.
        
        Args:
            port (str): Порт (например, "COM3" или "/dev/ttyUSB0")
            baudrate (int): Скорость UART (по умолчанию 9600)
            
        Returns:
            bool: Успешность подключения
        """
        try:
            self._ser = serial.Serial(port, baudrate, timeout=1)
            self._byte_delay = byte_delay
            self._running = True
            threading.Thread(target=self._read_thread, daemon=True).start()
            return True
        except Exception as e:
            print(f"Connection error: {e}")
            return False

    def disconnect(self):
        """Отключиться от устройства"""
        self._running = False
        if self._ser and self._ser.is_open:
            self._ser.close()

    def i2c_write(self, addr, cmd, data=None):
        """Запись данных по I2C.
        
        Args:
            addr (int): 7-битный адрес устройства (0x00-0x7F)
            cmd (int): 16-битная команда
            data (bytes, optional): Данные для записи
            
        Returns:
            bool: True если команда отправлена успешно
        """
        cmd_bytes = bytes([(cmd >> 8) & 0xFF, cmd & 0xFF])
        write_len = len(data) if data else 0
        payload = bytes([addr, write_len]) + cmd_bytes + (data if data else bytes())
        return self._send_packet(Command.I2C_WRITE, payload)

    def i2c_read(self, addr, cmd, length=1):
        """Чтение данных по I2C.
        
        Args:
            addr (int): 7-битный адрес устройства
            cmd (int): 16-битная команда
            length (int): Количество байт для чтения
            
        Returns:
            bytes|None: Прочитанные данные или None при ошибке
        """
        cmd_bytes = bytes([(cmd >> 8) & 0xFF, cmd & 0xFF])
        payload = bytes([addr, length]) + cmd_bytes
        if not self._send_packet(Command.I2C_READ, payload):
            return None
        
        response = self._wait_response(timeout=1.0)
        if response and response[0] == Status.OK and len(response) > 1:
            return response[1:]  # Возвращаем данные (без статуса)
        return None

    def gpio_read(self, pin):
        """Чтение состояния GPIO.
        
        Args:
            pin (int): Номер пина (0-2)
            
        Returns:
            int|None: 0/1 (LOW/HIGH) или None при ошибке
        """
        if not self._send_packet(Command.GPIO_READ, bytes([pin])):
            return None
        
        response = self._wait_response(timeout=1.0)
        if response and response[0] == Status.OK and len(response) >= 2:
            return response[1]  # Состояние пина
        return None

    def gpio_write(self, pin, state):
        """Запись состояния GPIO.
        
        Args:
            pin (int): Номер пина (1-2)
            state (int): 0 (LOW) или 1 (HIGH)
            
        Returns:
            bool: True если команда выполнена успешно
        """
        payload = bytes([pin, state])
        if not self._send_packet(Command.GPIO_WRITE, payload):
            return False
        
        response = self._wait_response(timeout=1.0)
        return response is not None and response[0] == Status.OK

    def ping(self):
        """Проверка связи с устройством.
        
        Returns:
            bool: True если устройство ответило
        """
        if not self._send_packet(Command.PING):
            return False
        
        response = self._wait_response(timeout=1.0)
        return response is not None and response[0] == Status.OK

    # Приватные методы
    def _calculate_crc(self, current_crc, new_byte):
        crc = current_crc ^ new_byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) if (crc & 0x80) else (crc << 1)
        return crc & 0xFF

    def _send_packet(self, cmd, data=None):
        if not self._ser or not self._ser.is_open:
            return False
        
        self._packet_id = (self._packet_id + 1) % 256
        payload = bytes([self._packet_id, cmd]) + (data if data else bytes())
        packet = bytes([0x41, 0x41, len(payload)]) + payload
        
        if self._use_crc:
            crc = 0
            crc = self._calculate_crc(crc, len(payload))
            for byte in payload:
                crc = self._calculate_crc(crc, byte)
            packet += bytes([crc])
        
        try:
            # Отправка с задержкой между байтами
            for byte in packet:
                self._ser.write(bytes([byte]))
                time.sleep(self._byte_delay)  # Задержка между байтами
            
            self._ser.flush()
            return True
        except Exception as e:
            print(f"Send error: {e}")
            return False

    def _read_thread(self):
        buffer = bytes()
        while self._running:
            try:
                if self._ser.in_waiting > 0:
                    buffer += self._ser.read(self._ser.in_waiting)
                    
                    while len(buffer) >= 5:
                        start = buffer.find(b'\x41\x41')
                        if start == -1:
                            buffer = bytes()
                            break
                        
                        buffer = buffer[start:]
                        if len(buffer) < 5:
                            break
                        
                        length = buffer[2]
                        if len(buffer) < 3 + length + (1 if self._use_crc else 0):
                            break
                        
                        packet = buffer[:3 + length + (1 if self._use_crc else 0)]
                        buffer = buffer[3 + length + (1 if self._use_crc else 0):]
                        
                        if self._use_crc:
                            crc = 0
                            crc = self._calculate_crc(crc, length)
                            for byte in packet[3:3+length]:
                                crc = self._calculate_crc(crc, byte)
                            if packet[-1] != crc:
                                self._rx_queue.put(("ERROR", "CRC error"))
                                continue
                        
                        payload = packet[3:3+length]
                        self._rx_queue.put(("DATA", payload[2:]))  # Игнорируем packet_id и cmd
            except Exception as e:
                self._rx_queue.put(("ERROR", str(e)))
                time.sleep(0.1)

    def _wait_response(self, timeout=1.0):
        start_time = time.time()
        while time.time() - start_time < timeout:
            if not self._rx_queue.empty():
                event_type, data = self._rx_queue.get()
                if event_type == "DATA":
                    return data
        return None