from ssd1306 import SSD1306_I2C
from writer import Writer
import math
import NotoSansCondensed15 as font


class TFADisplay(SSD1306_I2C):
    def __init__(self, *args, **kwargs):
        SSD1306_I2C.__init__(self, *args, **kwargs)

        self._wri = Writer(self, font)
        self.fill(0)
        self.text('Waiting for data', 0, 0, 1)
        self.show()

    def text(self, string, x, y, col):
        # In case a previous test has altered this
        self._wri.set_textpos(self, y, x)
        self._wri.printstring(string, not col)

    def update(self, data_dict, datetime_tuple, wifi_rssi=None, cc1101_rssi=-300):
        self.fill(0)
        self.text(f'{data_dict.get("temperature",math.nan):.1f}C {data_dict.get("humidity",math.nan):.0f}% {data_dict.get("rain",math.nan):.1f}mm', 0, 0, 1)
        if wifi_rssi:
            self.text(
                f'{data_dict.get("windspeed",math.nan):0.1f}-{data_dict.get("windgust",math.nan):0.1f}m/s {wifi_rssi}dBm', 0, font.height(), 1)
        else:
            self.text(
                f'{data_dict.get("windspeed",math.nan):0.1f}-{data_dict.get("windgust",math.nan):0.1f}m/s noWiFi', 0, font.height(), 1)
        self.text(
            f'{datetime_tuple[0]:04d}-{datetime_tuple[1]:02d}-{datetime_tuple[2]:02d} {cc1101_rssi}dBm', 0, 2*font.height(), 1)

        self.text(
            f'{datetime_tuple[4]:02d}:{datetime_tuple[5]:02d}:{datetime_tuple[6]:02d}', 0, 3*font.height(), 1)
        self.show()
