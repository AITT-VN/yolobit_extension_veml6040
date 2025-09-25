import machine
from yolobit import *
from micropython import const

_VEML6040_I2C_ADDRESS = const(0x10)

# REGISTER CONF
_VEML6040_IT_40MS   = const(0x00)
_VEML6040_IT_80MS   = const(0x10)
_VEML6040_IT_160MS  = const(0x20)
_VEML6040_IT_320MS  = const(0x30)
_VEML6040_IT_640MS  = const(0x40)
_VEML6040_IT_1280MS = const(0x50)

_VEML6040_AF_AUTO   = const(0x00)
_VEML6040_SD_ENABLE = const(0x00)

# COMMAND CODES
_COMMAND_CODE_CONF  = const(0x00)
_COMMAND_CODE_RED   = const(0x08)
_COMMAND_CODE_GREEN = const(0x09)
_COMMAND_CODE_BLUE  = const(0x0A)
_COMMAND_CODE_WHITE = const(0x0B)

# G SENSITIVITY
_VEML6040_GSENS = {
    _VEML6040_IT_40MS:   0.25168,
    _VEML6040_IT_80MS:   0.12584,
    _VEML6040_IT_160MS:  0.06292,
    _VEML6040_IT_320MS:  0.03146,
    _VEML6040_IT_640MS:  0.01573,
    _VEML6040_IT_1280MS: 0.007865,
}


def _sensitivity_from_config(config):
    return _VEML6040_GSENS.get(config & 0x70, 0.06292)  # default 160ms


def rgb2hsv(r, g, b):
    """RGB (0–65535) → HSV dict. Trả về None nếu dữ liệu không hợp lệ."""
    try:
        if None in (r, g, b):
            return {'hue': None, 'sat': None, 'val': None}
        r_f, g_f, b_f = r / 65535.0, g / 65535.0, b / 65535.0
    except Exception:
        return {'hue': None, 'sat': None, 'val': None}

    high, low = max(r_f, g_f, b_f), min(r_f, g_f, b_f)
    v, d = high, high - low
    s = 0.0 if high == 0 else d / high

    if d == 0:
        h = 0.0
    elif high == r_f:
        h = (g_f - b_f) / d + (6 if g_f < b_f else 0)
    elif high == g_f:
        h = (b_f - r_f) / d + 2
    else:
        h = (r_f - g_f) / d + 4
    h = (h / 6.0) * 360.0

    return {'hue': h % 360, 'sat': s, 'val': v}


class VEML6040Sensor:
    def __init__(self, address=_VEML6040_I2C_ADDRESS):
        self._i2c = machine.SoftI2C(scl=pin19.pin, sda=pin20.pin, freq=100000)
        self._addr = address
        self._config = 0

        if address not in self._i2c.scan():
            raise Exception('Color sensor VEML6040 not found')

        self.config(_VEML6040_IT_160MS + _VEML6040_AF_AUTO + _VEML6040_SD_ENABLE)

    def config(self, config):
        try:
            self._i2c.writeto(self._addr, bytes([_COMMAND_CODE_CONF, config, 0]))
            self._config = config
        except OSError:
            pass

    def read(self, commandCode):
        try:
            data = self._i2c.readfrom_mem(self._addr, commandCode, 2)
            return data[0] + (data[1] << 8)
        except OSError:
            return None

    def get_red(self):   return self.read(_COMMAND_CODE_RED)
    def get_green(self): return self.read(_COMMAND_CODE_GREEN)
    def get_blue(self):  return self.read(_COMMAND_CODE_BLUE)
    def get_white(self): return self.read(_COMMAND_CODE_WHITE)

    def get_lux(self):
        green = self.get_green()
        if green is None:
            return None
        return int(green * _sensitivity_from_config(self._config))

    def get_cct(self, offset=0):
        r, g, b = self.get_red(), self.get_green(), self.get_blue()
        if None in (r, g, b) or g == 0:
            return None
        try:
            ccti = (float(r) - float(b)) / float(g) + offset
            return 4278.6 * pow(ccti, -1.2455)
        except Exception:
            return None

    def Classify_Hue(self):
        hues = {"red": 0, "yellow": 60, "green": 120, "cyan": 180, "blue": 240, "magenta": 300}
        d = self.read_hsv()
        if not d or d['hue'] is None or d['val'] is None:
            return None
        if d['val'] > 0:
            return min(hues.items(), key=lambda x: min(360 - abs(d['hue'] - x[1]), abs(d['hue'] - x[1])))[0]
        return None

    def read_rgb(self):
        r, g, b, w = self.get_red(), self.get_green(), self.get_blue(), self.get_white()
        if None in (r, g, b):
            return {"red": None, "green": None, "blue": None, "white": None, "als": None, "cct": None}

        try:
            X = (-0.023249 * r) + (0.291014 * g) + (-0.364880 * b)
            Y = (-0.042799 * r) + (0.272148 * g) + (-0.279591 * b)
            Z = (-0.155901 * r) + (0.251534 * g) + (-0.076240 * b)
            total = X + Y + Z
            if total == 0:
                return {"red": None, "green": None, "blue": None, "white": None, "als": None, "cct": None}
            x, y = X / total, Y / total
            n = (x - 0.3320) / (0.1858 - y)
            CCT = 449.0 * n ** 3 + 3525.0 * n ** 2 + 6823.3 * n + 5520.33
        except Exception:
            CCT = None

        ALS = int(g * _sensitivity_from_config(self._config)) if g else None
        return {"red": r, "green": g, "blue": b, "white": w, "als": ALS, "cct": CCT}

    def read_hsv(self):
        d = self.read_rgb()
        if not d or None in (d['red'], d['green'], d['blue']):
            return {'hue': None, 'sat': None, 'val': None}
        return rgb2hsv(d['red'], d['green'], d['blue'])


veml6040_sensor = VEML6040Sensor()
