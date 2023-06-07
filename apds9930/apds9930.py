# APDS9930 library
# adapted to CircuitPython from
# https://github.com/micropython-Chinese-Community/mpy-lib/blob/master/sensor/APDS9930/APDS9930.py

import busio
import time

# APDS9930 I2C address register
_APDS9930_I2C_ADDRESS = 0x39

# APDS-9930 register addresses
_APDS9930_ENABLE      = 0x00
_APDS9930_ATIME       = 0x01
_APDS9930_PTIME       = 0x02
_APDS9930_WTIME       = 0x03
_APDS9930_AILTL       = 0x04
_APDS9930_AILTH       = 0x05
_APDS9930_AIHTL       = 0x06
_APDS9930_AIHTH       = 0x07
_APDS9930_PILTL       = 0x08
_APDS9930_PILTH       = 0x09
_APDS9930_PIHTL       = 0x0A
_APDS9930_PIHTH       = 0x0B
_APDS9930_PERS        = 0x0C
_APDS9930_CONFIG      = 0x0D
_APDS9930_PPULSE      = 0x0E
_APDS9930_CONTROL     = 0x0F
_APDS9930_ID          = 0x12
_APDS9930_STATUS      = 0x13
_APDS9930_Ch0DATAL    = 0x14
_APDS9930_Ch0DATAH    = 0x15
_APDS9930_Ch1DATAL    = 0x16
_APDS9930_Ch1DATAH    = 0x17
_APDS9930_PDATAL      = 0x18
_APDS9930_PDATAH      = 0x19
_APDS9930_POFFSET     = 0x1E

# ALS coefficients
DF = 52
GA = 0.49
B  = 1.862
C  = 0.746
D  = 1.291

# AGAIN
_APDS9930_AGAIN = (1, 8, 16, 120)

# PGAIN
_APDS9930_PGAIN = (1, 2, 4, 8)

class APDS9930:
    def __init__(
    self,
    i2c,
    address: int = _APDS9930_I2C_ADDRESS
    ):

        self.i2c = i2c
        self._APDS9930_ADDRESS = address

        self.ATIME(256 - 8)
        self._setReg(_APDS9930_ENABLE, 0)
        self._setReg(_APDS9930_ATIME, 0xFF)
        self._setReg(_APDS9930_PTIME, 0xFF)
        self._setReg(_APDS9930_WTIME, 0xFF)
        self._setReg(_APDS9930_PERS, 0x22)
        self._setReg(_APDS9930_CONFIG, 0)
        self._setReg(_APDS9930_PPULSE, 8)
        self._setReg(_APDS9930_CONTROL, 0x2C)

        self.enable_power(True)
        self.enable_als(True)
        self.enable_proximity(True)

    def _setReg(self, reg, dat):
        self.i2c.try_lock()
        self.i2c.writeto(self._APDS9930_ADDRESS, bytes([reg | 0xA0, dat]))
        self.i2c.unlock()

    def _getReg(self, reg):
        self.i2c.try_lock()
        self.i2c.writeto_then_readfrom(self._APDS9930_ADDRESS, bytearray([reg | 0xA0]), bytearray(1))
        data = bytearray(1)
        self.i2c.readfrom_into(self._APDS9930_ADDRESS, data)
        self.i2c.unlock()
        return data[0]

    def _get2Reg(self, reg):
        self.i2c.try_lock()
        self.i2c.writeto_then_readfrom(self._APDS9930_ADDRESS, bytearray([reg | 0xA0]), bytearray(2))
        data = bytearray(2)
        self.i2c.readfrom_into(self._APDS9930_ADDRESS, data)
        self.i2c.unlock()
        return data[0] + data[1] * 256

    def ATIME(self, v = None):
        if v == None:
            return self._getReg(_APDS9930_ATIME)
        else:
            self._setReg(_APDS9930_ATIME, v)

    def AGAIN(self, gain = None):
        t = self._getReg(_APDS9930_CONTROL)
        if gain == None:
            return  _APDS9930_AGAIN[t & 0x03]
        else:
            t &= 0xFC
            t |= _APDS9930_AGAIN.index(gain)
            self._setReg(_APDS9930_CONTROL, t)

    def PGAIN(self, gain: int = None):
        t = self._getReg(_APDS9930_CONTROL)
        if gain == None:
            return  _APDS9930_PGAIN[(t & 0x0F)>>2]
        else:
            t &= 0xF3
            t |= _APDS9930_PGAIN.index(gain)<<2
            self._setReg(_APDS9930_CONTROL, t)

    def enable_power(self, on: bool = True):
        t = self._getReg(_APDS9930_ENABLE)
        t &= 0xFE
        if on:
            t |= 1
        self._setReg(_APDS9930_ENABLE, t)
        time.sleep(0.003)

    def enable_als(self, on: bool = True):
        t = self._getReg(_APDS9930_ENABLE)
        t &= 0xFD
        if on:
            t |= 2
        self._setReg(_APDS9930_ENABLE, t)

    def enable_proximity(self, on: bool = True):
        t = self._getReg(_APDS9930_ENABLE)
        t &= 0xFB
        if on:
            t |= 4
        self._setReg(_APDS9930_ENABLE, t)

    def enable_wait(self, on=True):
        t = self._getReg(_APDS9930_ENABLE)
        t &= 0xF7
        if on:
            t |= 8
        self._setReg(_APDS9930_ENABLE, t)

    @property
    def als(self) -> float:
        CH0DATA = 256 * self._getReg(_APDS9930_Ch0DATAH) + self._getReg(_APDS9930_Ch0DATAL)
        CH1DATA = 256 * self._getReg(_APDS9930_Ch1DATAH) + self._getReg(_APDS9930_Ch1DATAL)
        ALSIT = 2.72 * (256 - self.ATIME())
        IAC1 = CH0DATA - B * CH1DATA
        IAC2 = C * CH0DATA - D * CH1DATA
        IAC = max(IAC1, IAC2, 0)
        LPC = GA * DF / (ALSIT * self.AGAIN())
        return IAC * LPC

    @property
    def proximity(self) -> float:
#         PDATA = 256 * self._getReg(_APDS9930_PDATAH) + self._getReg(_APDS9930_PDATAL)
        PDATA = self._get2Reg(_APDS9930_PDATAL)
        return PDATA / self.PGAIN()
