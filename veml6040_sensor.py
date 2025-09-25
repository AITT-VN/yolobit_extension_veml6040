import machine, time
from yolobit import *
from micropython import const
from math import exp, isnan
from utility import say
# from setting import *

_VEML6040_I2C_ADDRESS    = const(0x10)

# REGISTER CONF (00H) SETTINGS
_VEML6040_IT_40MS        = const(0x00)
_VEML6040_IT_80MS        = const(0x10)
_VEML6040_IT_160MS       = const(0x20)
_VEML6040_IT_320MS       = const(0x30)
_VEML6040_IT_640MS       = const(0x40)
_VEML6040_IT_1280MS      = const(0x50)

_VEML6040_TRIG_DISABLE   = const(0x00)
_VEML6040_TRIG_ENABLE    = const(0x04)

_VEML6040_AF_AUTO        = const(0x00)
_VEML6040_AF_FORCE       = const(0x02)

_VEML6040_SD_ENABLE      = const(0x00)
_VEML6040_SD_DISABLE     = const(0x01)

# COMMAND CODES
_COMMAND_CODE_CONF       = const(0x00)
_COMMAND_CODE_RED        = const(0x08)
_COMMAND_CODE_GREEN      = const(0x09)
_COMMAND_CODE_BLUE       = const(0x0A)
_COMMAND_CODE_WHITE      = const(0x0B)

# G SENSITIVITY (sử dụng để tính ALS chính xác hơn)
_VEML6040_GSENS_40MS     = 0.25168
_VEML6040_GSENS_80MS     = 0.12584
_VEML6040_GSENS_160MS    = 0.06292
_VEML6040_GSENS_320MS    = 0.03146
_VEML6040_GSENS_640MS    = 0.01573
_VEML6040_GSENS_1280MS   = 0.007865

def _sensitivity_from_config(config):
    """Trả về hệ số sensitivity tương ứng với integration time trong config."""
    it = config & 0x70
    if it == _VEML6040_IT_40MS:
        return _VEML6040_GSENS_40MS
    elif it == _VEML6040_IT_80MS:
        return _VEML6040_GSENS_80MS
    elif it == _VEML6040_IT_160MS:
        return _VEML6040_GSENS_160MS
    elif it == _VEML6040_IT_320MS:
        return _VEML6040_GSENS_320MS
    elif it == _VEML6040_IT_640MS:
        return _VEML6040_GSENS_640MS
    elif it == _VEML6040_IT_1280MS:
        return _VEML6040_GSENS_1280MS
    else:
        return _VEML6040_GSENS_160MS  # fallback

def rgb2hsv(r, g, b):
    """
    Chuyển RGB (16-bit expected, 0..65535) -> dict {'hue','sat','val'}.
    Nếu bất kỳ r/g/b là None hoặc không hợp lệ, trả về {'hue':None,'sat':None,'val':None}.
    """
    # bảo vệ khi receive None hoặc giá trị không phải số
    try:
        if r is None or g is None or b is None:
            return {'hue': None, 'sat': None, 'val': None}
        # ép về float
        r_f = float(r) / 65535.0
        g_f = float(g) / 65535.0
        b_f = float(b) / 65535.0
    except Exception:
        return {'hue': None, 'sat': None, 'val': None}

    high = max(r_f, g_f, b_f)
    low = min(r_f, g_f, b_f)
    v = high
    d = high - low
    s = 0.0 if high == 0 else (d / high)

    if d == 0:
        h = 0.0
    else:
        # chọn channel max để tính hue (giữ logic cũ)
        if high == r_f:
            h_ = (g_f - b_f) / d + (6 if g_f < b_f else 0)
        elif high == g_f:
            h_ = (b_f - r_f) / d + 2
        else:
            h_ = (r_f - g_f) / d + 4
        h = (h_ / 6.0) * 360.0

    # chuẩn hoá hue vào [0,360)
    if h is None:
        return {'hue': None, 'sat': s, 'val': v}
    while h < 0:
        h += 360.0
    while h >= 360.0:
        h -= 360.0

    return {'hue': h, 'sat': s, 'val': v}

class VEML6040Sensor:

    def __init__(self, address=_VEML6040_I2C_ADDRESS):
        # Lưu ý: giữ nguyên cách khởi tạo I2C từ yolobit (pin19.pin, pin20.pin)
        self._i2c = machine.SoftI2C(scl=pin19.pin, sda=pin20.pin, freq=100000)
        self._addr = address
        self._config = 0

        if self._i2c.scan().count(address) == 0:
            raise Exception('Color sensor VEML6040 not found')

        # cấu hình mặc định
        self.config(_VEML6040_IT_160MS + _VEML6040_AF_AUTO + _VEML6040_SD_ENABLE)

    def config(self, config):
        self._i2c.writeto(self._addr, bytes([_COMMAND_CODE_CONF, config, 0]))
        self._config = config

    def read(self, commandCode):
        # đọc 2 byte little-endian
        data = self._i2c.readfrom_mem(self._addr, commandCode, 2)
        return data[0] + (data[1] << 8)

    def get_red(self):
        return self.read(_COMMAND_CODE_RED)

    def get_green(self):
        return self.read(_COMMAND_CODE_GREEN)

    def get_blue(self):
        return self.read(_COMMAND_CODE_BLUE)

    def get_white(self):
        return self.read(_COMMAND_CODE_WHITE)

    def get_lux(self):
        sensorValue = self.read(_COMMAND_CODE_GREEN)
        it = (self._config & 0x70)
        # so sánh đúng bit bằng cách dùng ()
        if it == _VEML6040_IT_40MS:
            ambientLightInLux = sensorValue * _VEML6040_GSENS_40MS
        elif it == _VEML6040_IT_80MS:
            ambientLightInLux = sensorValue * _VEML6040_GSENS_80MS
        elif it == _VEML6040_IT_160MS:
            ambientLightInLux = sensorValue * _VEML6040_GSENS_160MS
        elif it == _VEML6040_IT_320MS:
            ambientLightInLux = sensorValue * _VEML6040_GSENS_320MS
        elif it == _VEML6040_IT_640MS:
            ambientLightInLux = sensorValue * _VEML6040_GSENS_640MS
        elif it == _VEML6040_IT_1280MS:
            ambientLightInLux = sensorValue * _VEML6040_GSENS_1280MS
        else:
            ambientLightInLux = -1
        try:
            return int(ambientLightInLux)
        except Exception:
            return -1

    def get_cct(self, offset = 0):
        red = self.get_red()
        green = self.get_green()
        blue = self.get_blue()

        # bảo vệ chia cho 0
        try:
            ccti = (float(red) - float(blue)) / float(green)
        except Exception:
            return None
        ccti = ccti + offset
        try:
            cct = 4278.6 * pow(ccti, -1.2455)
        except Exception:
            cct = None
        return cct

    def Classify_Hue(self):
        """
        Trả về tên màu gần đúng (red, yellow, green, cyan, blue, magenta) hoặc None.
        An toàn với dữ liệu None.
        """
        hues = {"red": 0, "yellow": 60, "green": 120, "cyan": 180, "blue": 240, "magenta": 300}
        min_brightness = 0

        d = self.read_hsv()

        # kiểm tra an toàn
        hue = d.get('hue')
        val = d.get('val')
        if hue is None or val is None:
            return None
        if val > min_brightness:
            # chọn màu có khoảng cách nhỏ nhất (cân bằng vòng tròn)
            key, valm = min(hues.items(), key=lambda x: min(360 - abs(hue - x[1]), abs(hue - x[1])))
            return key
        else:
            return None

    def read_rgb(self):
        red = self.get_red()
        green = self.get_green()
        blue = self.get_blue()
        white = self.get_white()

        # Nếu đọc bị lỗi trả về None (an toàn)
        try:
            colour_X = (-0.023249 * red) + (0.291014 * green) + (-0.364880 * blue)
            colour_Y = (-0.042799 * red) + (0.272148 * green) + (-0.279591 * blue)
            colour_Z = (-0.155901 * red) + (0.251534 * green) + (-0.076240 * blue)
        except Exception:
            return {"red": None, "green": None, "blue": None, "white": None, "als": None, "cct": None}

        colour_total = colour_X + colour_Y + colour_Z
        if colour_total == 0:
            return {"red": None, "green": None, "blue": None, "white": None, "als": None, "cct": None}
        else:
            colour_x = colour_X / colour_total
            colour_y = colour_Y / colour_total

        # tính CCT (bảo vệ giá trị bất thường)
        try:
            colour_n = (colour_x - 0.3320) / (0.1858 - colour_y)
            colour_CCT = 449.0 * colour_n ** 3 + 3525.0 * colour_n ** 2 + 6823.3 * colour_n + 5520.33
        except Exception:
            colour_CCT = None

        # Tính ALS bằng hệ số sensitivity tương ứng với integration time
        try:
            sens = _sensitivity_from_config(self._config)
            colour_ALS = int(green * sens)
        except Exception:
            colour_ALS = None

        return {"red": red, "green": green, "blue": blue, "white": white, "als": colour_ALS, "cct": colour_CCT}

    def read_hsv(self):
        """
        Trả về dict {'hue','sat','val'}.
        Luôn an toàn (không ném exception) — trả giá trị None nếu không đọc được.
        """
        d = self.read_rgb()
        # nếu bất kỳ channel nào None -> trả về dict hợp lệ có giá trị None
        if not d:
            return {'hue': None, 'sat': None, 'val': None}
        if d.get('red') is None or d.get('green') is None or d.get('blue') is None:
            return {'hue': None, 'sat': None, 'val': None}
        return rgb2hsv(d['red'], d['green'], d['blue'])

veml6040_sensor = VEML6040Sensor()

