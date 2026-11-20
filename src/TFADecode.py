"""
https://github.com/RFD-FHEM/RFFHEM/blob/master/FHEM/lib/SD_ProtocolData.pm#L3218
https://forum.fhem.de/index.php/topic,119335.msg1221926.html#msg1221926
https://github.com/RFD-FHEM/RFFHEM/blob/1762fb28d352e7de9c282ef4f52fd9c0803ee19c/FHEM/14_SD_WS.pm#L1307
"""
from crc import crc_poly

try:
    from machine import RTC
except ImportError:
    def RTC():
        return None


def bits2int(lst) -> int:
    """
    Each value in lst is interpreted as boolean value of an MSB integer number
    """
    val = 0
    length = len(lst)
    for i in range(length):
        if lst[length-1 - i]:
            val += lst[length-1-i] << i
    return val


def bitlen(val: int) -> int:
    """
    Return how many bits are required to store the unsigned integer `val`
    """
    if val > 0:
        len = 0
        while val > 0:
            val = val >> 1
            len += 1
        return len


def last_non_ascii_zero_index_in_bytes(data_bytes: bytes) -> int:
    for i in range(len(data_bytes)-1, -1, -1):
        # print(i, data_bytes[i])
        if data_bytes[i] != 0x30:  # ASCII zero is 0x30
            return i
    else:
        return -1


def test_last_non_ascii_zero_index_in_bytes():
    assert last_non_ascii_zero_index_in_bytes(b'005') == 2
    assert last_non_ascii_zero_index_in_bytes(b'101001') == 5
    assert last_non_ascii_zero_index_in_bytes(b'00') == -1


def twostate_demodulate(val: bytes, state1: bytes = b'100', state2: bytes = b'11100') -> list[bool]:
    """
    `val` is a bytes object which is a concatenation of bytes objects `state1` and `state2`.
    Iteration over `val`, a list of boolean is filled where True is appended when content of
    `state1` occures, and False is appended when content of `state2` occurs.
    """
    data = []
    i = 0
    while True:
        try:
            if val[i:i+len(state1)] == state1:
                data.append(True)
                i += len(state1)
            elif val[i:i+len(state2)] == state2:
                data.append(False)
                i += len(state2)
            else:
                break
        except IndexError:
            break
    idx = last_non_ascii_zero_index_in_bytes(state1)
    if (i+idx) < len(val) and (i+len(state1)) >= len(val):
        if val[i:min(len(val), i+len(state1))] == state1[:len(val)-i]:
            data.append(True)
    idx = last_non_ascii_zero_index_in_bytes(state2)
    if (i+idx) < len(val) and (i+len(state2)) >= len(val):
        if val[i:min(len(val), i+len(state2))] == state2[:len(val)-i]:
            data.append(False)

    # print(data)
    # print(val)
    return data

def twostate_modulate(lst_bool, state_true =b'100', state_false = b'11100') -> bytes:
    lst = []
    for val in lst_bool:
        if val:
            lst.append(state_true)
        else:
            lst.append(state_false)
    return b''.join(lst)


def test_twostate_demodulate():
    assert twostate_demodulate(b'10011100') == [True, False]
    assert twostate_demodulate(b'100111000') == [True, False]
    assert twostate_demodulate(b'10011000') == [True]
    assert twostate_demodulate(b'1000') == [True]
    assert twostate_demodulate(b'100111000100') == [True, False]
    assert twostate_demodulate(b'1001') == [True, True]
    assert twostate_demodulate(b'10010') == [True, True]
    assert twostate_demodulate(b'100100') == [True, True]
    assert twostate_demodulate(b'1001000') == [True, True]
    assert twostate_demodulate(b'100111') == [True, False]
    assert twostate_demodulate(b'1001110') == [True, False]
    assert twostate_demodulate(b'10011100') == [True, False]
    assert twostate_demodulate(b'100111000') == [True, False]
    assert twostate_demodulate(b'11100111001000000000') == [False, False, True]
    assert twostate_demodulate(b'11100111000000000000') == [False, False]


class TFADecode:
    def __init__(self):
        self._rtc = RTC()
        self._bits = [True]*80
        self._bytes = bytearray(10)
        self._preamble = None
        self._whid = None
        self._ident = None
        self._flag = None
        self._max_decode = 0
        self.crcok = False
        # storage for weather data
        self.data_dict = {'battery_low': None,
                          'temperature': None,
                          'humidity': None,
                          'windspeed': None,
                          'windgust': None,
                          'rain': None,
                          }

        self._reset()

    @property
    def data_json(self):
        import json
        return json.dumps(self.data_dict)

    @property
    def data_raw(self):
        return bits2int(self._bits)

    def is_weather_data(self):
        """weather data when flag is False and whid is 0,1,0,1"""
        return not self._flag and self._whid == [0, True, 0, True]

    def is_time_data(self):
        """time data when flag is true and whid is 0,1,1,0"""
        return self._flag and self._whid == [0, True, True, 0]

    def update(self, val):
        """
        Updates attributes based on `val`.
        Returns True, if data was plausible (by length and CRC), False otherwise.

        Parameters
        ----------
        val : bytes, int, list[bool]
            Either, bytes containsing 1 and 0 characters representing bits before the
            twostate demodulation, integer representing the demodulated value, list of
            bool representing the correspoding bitfield after demodulation.

        Returns
        -------
        bool
        """
        self._max_decode = 0
        self.crcok = False

        if isinstance(val, int):
            for i in range(bitlen(val), len(self._bits)):
                # fill up rest of bits
                self._bits[i] = 0
                self._max_decode = i
            for i in range(bitlen(val)-1, -1, -1):
                self._bits[i] = val % 2
                val = val >> 1
        else:  # got array of bits
            if len(val) < 78:
                self._max_decode = -1
            else:
                if len(val) > 80:
                    orig_val = val
                    val = twostate_demodulate(val)
                    reconstructed_val = twostate_modulate(val)
                    tmp_idx = min(len(orig_val), len(reconstructed_val))
                    if orig_val[:tmp_idx] != reconstructed_val[:tmp_idx]:
                        print('Reconstruction error')
                        print(orig_val)
                        print(reconstructed_val)
                    self._max_decode = len(val)

                for i in range(min(80, len(val))):
                    self._bits[i] = bool(val[i])
                    self._max_decode = i
                else:
                    # fill up rest of bits
                    for j in range(i, len(self._bits)):
                        self._bits[j] = False

        if self._max_decode < 78:
            self._reset()
        else:
            for i in range(10):
                acc = 0
                for j in range(8):
                    acc += self._bits[i*8+(7-j)] << j
                self._bytes[i] = acc

            # crc8 with poly 0x31 and first byte stripped according to
            # https://github.com/RFD-FHEM/RFFHEM/blob/1762fb28d352e7de9c282ef4f52fd9c0803ee19c/FHEM/14_SD_WS.pm#L1379
            self.crcok = crc_poly(self._bytes[1::], 8, 0x31) == 0

            if self.crcok:
                self._decode()
            else:
                self._reset()

        return self.crcok

    def _reset(self):
        """
        Invalidate all attributes holding weather or time data by setting
        them to None.
        """
        self.dfc_tuple = None
        self._whid = None
        self._flag = None
        self.data_dict['battery_low'] = None
        self.data_dict['temperature'] = None
        self.data_dict['humidity'] = None
        self.data_dict['windspeed'] = None
        self.data_dict['windgust'] = None
        self.data_dict['rain'] = None

    def _decode(self):
        bits = self._bits  # local name for speed in micropython
        self._preamble = bits[0:7]
        self._whid = bits[7:11]
        self._ident = bits[11:19]
        self._flag = bits[19]
        # print(f'Metadata:flag {self.flag}, whid: {self.whid}, ident: {bits2int(self.ident)}')

        self.data_dict['battery_low'] = bool(bits[20])

        if self.is_time_data():
            # got time-data
            year = 2000+bits2int(bits[47:51])*10+bits2int(bits[51:55])
            month = bits2int(bits[58:59])*10+bits2int(bits[59:63])
            day = bits2int(bits[65:67])*10+bits2int(bits[67:71])
            hour = bits2int(bits[25:27])*10+bits2int(bits[27:31])
            minute = bits2int(bits[32:35])*10+bits2int(bits[35:39])
            second = bits2int(bits[40:43])*10+bits2int(bits[43:47])
            self.dfc_tuple = (year, month, day, 0, hour, minute, second, 0)
            if self._rtc:
                self._rtc.datetime(self.dfc_tuple)
            # print(self.dfc_tuple)
        elif self.is_weather_data():
            # got weather-data
            self.data_dict['temperature'] = bits2int(bits[21:31])*0.1 - 40
            self.data_dict['humidity'] = bits2int(bits[31:39])
            self.data_dict['windspeed'] = bits2int(bits[39:47])/3
            self.data_dict['windgust'] = bits2int(bits[47:55])/3
            self.data_dict['rain'] = bits2int(bits[55:71])/30
            # print(self.data_dict)


__all__ = [twostate_demodulate, TFADecode]


def main():
    vals_raw = [b'10010010010010010010011100100111001001001001001110011100111001110011100111001110011100100100111001001001110010011100111001110010011100100111001110010010011100111001110011100111001110011100111001110011100111001110011100111001110011100111001110011100111001001001110010010011100111001110011100100111001110011100100111001110010011100111001000000000000000000000000000000000000000000010000111111010',
                b'10010010010010010010011100100111001001001001001110011100111001110011100111001110011100100100111001001001110010011100111001110010011100100111001110010010011100111001110011100111001110011100111001110011100111001110011100111001110010011100111001110011100100100111001001001110011100111001110010011100111001110011100111001110010010010010000000000000000000000000000000000000000000000010001011101111',
                b'10010010010010010010011100100110000000010010011100111001110011100111001100011000110010010011100100100111001001110000011000001110010011100111001110011100110001100011001110011100111001110010011100110001100011001110011100111001001100000000000000000000000000000000000000000000000000000000000000000000000000000111001110000000010000000000000000000000000000000000000000000000000000000000110011111111',
                0b1111111010111100000000111010011010110000000001100000100000000010011100000101001,
                b'100100100100100100100111001001110010010010010011100111001110011100111001110011100111001001001001110010011100111001001001110010011100100100111001110011100111001110011100111001110011100100100111001110011100111001110010011100111001110011100111001110011100111001110010011100111001001001001110011100111001110011100100111001001110011100100000000000',
                b'100100100100100100100111001001110010010010010011100111001110011100111001110011100111001001001001001001001110010011100111001110010010010010010011100111001110011100111001001110011100100111001110011100111001001001110010011100111001110011100100100111001001110010010011100111001110011100100111001001001110011100111001001',
                b'1001001001001001001001110010011100100100100100111001110011100111001110011100111001110010010010010010011100100100111001110010011100111001110010010011100111001110011100111001001001001001110011100111001001110010011100111001110011100111001110010010011100100111001001001110011100111001110010011100111001110010011100100100111',
                b'1001001001001001001001110010011100100100100100111001110011100111001110011100111001110010010011100100100100100111001001110010011100111001001001001001110011100111001110011100111001110010011100111001110011100111001110010011100111001110011100111001001001110010011100100100111001110010011100111001110010011100111001001001001',
                b'1001001001001001001001110010011100100100100100111001110011100111001110011100111001110010010011100100100100100111001001110010011100111001001001001001110011100111001110011100111001110010011100111001110011100111001110',
                0xFEAB049EAA080C0C5400,
                0xFEAB049EAA080C0C00,
                0xFEAB049EAA080C0C5300,
                1199481086648889884278784,
                ]
    # 0xFEAB049EAA080C0C5300 is successfull CRC decode from here:
    # https://forum.fhem.de/index.php?topic=119335.0;all
    # other values have been recorded by myself using Universal Radio Hacker
    tfa = TFADecode()

    for val_raw in vals_raw:
        # if isinstance(val_raw,bytes) and len(val_raw)>80:
        #     status, data = twostate_demodulate(val_raw)
        # else:
        #     data = val_raw

        print("var_raw", val_raw)
        tfa.update(val_raw)        
        print(tfa.crcok, tfa._max_decode)
        if tfa.is_time_data():
            print(tfa.dfc_tuple)
        else:
            print(tfa.data_dict)


if __name__ == '__main__':
    main()
