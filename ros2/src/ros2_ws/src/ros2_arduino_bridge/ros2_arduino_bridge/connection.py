"""
Двустороннее общение по Serial Master - Slave
"""
from enum import Enum
from struct import Struct
from time import sleep
from typing import Final

from serial import Serial
import struct

class Primitives(Enum):
    """Примитивные типы"""

    i8 = Struct("b")
    """int8_t"""
    u8 = Struct("B")
    """uint8_t"""
    i16 = Struct("h")
    """int16_t"""
    u16 = Struct("H")
    """uint16_t"""
    i32 = Struct("<l")
    """int32_t"""
    u32 = Struct("L")
    """uint32_t"""
    i64 = Struct("q")
    """int64_t"""
    I64 = Struct("Q")
    """unt64_t"""
    f32 = Struct("f")
    """float"""
    f64 = Struct("d")  # ! Не поддерживается на Arduino
    """double"""

    def pack(self, value: bool | int | float) -> bytes:
        return self.value.pack(value)

    def unpack(self, buffer: bytes) -> bool | int | float:
        return self.value.unpack(buffer)[0]

from dataclasses import dataclass

@dataclass(frozen=True)
class Data:
    dist: int

    data_packer = Struct("H")

    @classmethod
    def make(cls, buffer: bytes):
        return cls(*cls.data_packer.unpack(buffer))

class Command:
    """
    Команда по порту

    Имеет свой код (Должен совпадать на slave устройстве)
    Сигнатуру аргументов (Должна совпадать на устройстве)
    """

    def __init__(self, code: int, signature: tuple[Primitives, ...]) -> None:
        self.header: Final[bytes] = Primitives.u8.pack(code)
        self.signature = signature

    def pack(self, *args) -> bytes:
        """
        Скомпилировать команду в набор байт
        :param args: аргументы команды. (Их столько же, и такого же типа, что и сигнатура команды)
        :return:
        """
        return self.header + b"".join(primitive.pack(arg) for primitive, arg in zip(self.signature, args))


class ArduinoConnection:
    """Пример подключения к Arduino с dataминимальным набором команд"""

    def __init__(self, serial: Serial) -> None:
        self._serial = serial
        # Команды этого устройства
        self._set_motors = Command(0x10, (Primitives.i8, Primitives.i8))
        self._turn = Command(0x12, (Primitives.i8, Primitives.i8))
        self._get_data = Command(0x11, (Primitives.u8,))
        self._go_dist = Command(0x13, (Primitives.i32, Primitives.i32))

    # Обёртки над командами ниже, чтобы сразу компилировать и отправлять их в порт
    def setSpeeds(self, left: int, right: int) -> None:
        self._serial.write(self._set_motors.pack(left, right))

    def turn_robot(self, angle: int, speed: int) -> bool:
        self._serial.write(self._turn.pack(angle, speed))
        response = self._serial.read()
        return Primitives.u8.unpack(response)

    def go_dist(self, dist: int, speed: int) -> bool:
        self._serial.write(self._go_dist.pack(dist, speed))
        print(self._go_dist.pack(dist, speed))
        response = self._serial.read()
        print(response)
        return Primitives.u8.unpack(response)

    def get_data(self):
        self._serial.write(self._get_data.pack(1))
        data_bytes = self._serial.read(Data.data_packer.size)
        return Data.make(data_bytes)

    def close(self):
        self._serial.close()

if __name__ == '__main__':
    port_name = "/dev/ttyACM0"
    arduino = ArduinoConnection(Serial(port_name, 115200))

    sleep(2)
    #print(arduino.turn_robot(90, 4))
    print(arduino.go_dist(200, 2))
    # print(arduino.get_data())

    # arduino.setSpeeds(0, 0)
    # sleep(1)

    # arduino.setSpeeds(4, 0)
    # sleep(1)

    # arduino.setSpeeds(-4, 0)
    # sleep(1)

    # arduino.setSpeeds(0, 4)
    # sleep(1)

    # arduino.setSpeeds(0, -4)
    # sleep(1)

    # arduino.setSpeeds(0, 1)
    # sleep(1)

    arduino.close()
