from ssd1306 import SSD1306_I2C
from writer import Writer
import math
import time
import NotoSansCondensed15 as font_small
import NotoSansCondensed18 as font
import NotoSansCondensed28 as font_large


class TFADisplay(SSD1306_I2C):
    def __init__(self, *args, **kwargs):
        SSD1306_I2C.__init__(self, *args, **kwargs)

        self._page = 0
        self._data = {}
        self._wri = Writer(self, font)
        self._wri_small = Writer(self, font_small)
        self._wri_large = Writer(self, font_large)

        self.fill(0)
        self.text('Waiting for data', 0, 0, 1)
        self.show()

    def text(self, string, x, y, col):
        # In case a previous test has altered this
        self._wri.set_textpos(self, y, x)
        self._wri.printstring(string, not col)

    def text_small(self, string, x, y, col):
        # In case a previous test has altered this
        self._wri_small.set_textpos(self, y, x)
        self._wri_small.printstring(string, not col)

    def text_large(self, string, x, y, col):
        # In case a previous test has altered this
        self._wri_large.set_textpos(self, y, x)
        self._wri_large.printstring(string, not col)

    def update_values(self, **kwargs):
        for key, val in kwargs.items():
            self._data[key] = val

    def draw_page(self, page):
        self.fill(0)
        datetime_tuple = time.localtime()
        if page == 0:
            self.text(
                f'{self._data.get("temperature",math.nan):.1f}C {self._data.get("rain",math.nan):.1f}mm', 0, 0, 1)
            self.text(f'{self._data.get("humidity",math.nan):.0f}% {self._data.get("windspeed",math.nan):0.1f}-{self._data.get("windgust",math.nan):0.1f}m/s', 0, font.height(), 1)

            self.text_small(
                f'{datetime_tuple[0]:04d}-{datetime_tuple[1]:02d}-{datetime_tuple[2]:02d} {datetime_tuple[3]:02d}:{datetime_tuple[4]:02d}:{datetime_tuple[5]:02d}', 0, 2*font.height(), 1)
            self.text_small(
                f'{self._data.get("wifi_rssi",math.nan)}dBm {self._data.get("cc1101_rssi",math.nan)}dBm', 0, 64-font_small.height(), 1)
        else:
            self.text_large(
                f'{self._data.get("temperature",math.nan):.1f}C {self._data.get("humidity",math.nan):.0f}%', 0, 0, 1)
            self.text_small(
                f'{datetime_tuple[0]:04d}-{datetime_tuple[1]:02d}-{datetime_tuple[2]:02d} {datetime_tuple[3]:02d}:{datetime_tuple[4]:02d}:{datetime_tuple[5]:02d}', 0, font_large.height(), 1)
            self.text_small(f'{self._data.get("battery_low",math.nan)}',
                            0, font_large.height() + font_small.height(), 1)

        self.show()

    def next_page(self, *args):
        self._page = (self._page + 1) % 2
        self.redraw()

    def redraw(self):
        self.draw_page(self._page)
