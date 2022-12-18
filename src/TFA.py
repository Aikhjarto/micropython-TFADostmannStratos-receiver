from crc8 import crc8
import time
from machine import RTC

def bits2int(lst):
    val = 0
    length = len(lst)
    for i in range(length):
        if lst[length-1- i]:
            val +=lst[length-1-i] << i
    return val


def bitlen(val):
    if val > 0:
        len = 0
        while val > 0:
            val = val >> 1
            len += 1
        return len
    

def twostate_demodulate(val, state1=b'100', state2=b'11100'):
    data = []
    i = 0
    while True:
        try:
            if val[i:i+len(state1)] == state1:
                data.append(True)
                i +=len(state1)
            elif val[i:i+len(state2)] == state2:
                data.append(False)
                i +=len(state2)
            else:
                break
        except IndexError:
            break
    if (i+len(state1))>=len(val):
        if val[i:] == state1[:len(val)-i]:
            data.append(True)
    if (i+len(state2))>=len(val):
        if val[i:] == state2[:len(val)-i]:
            data.append(False)
    
    #print(data)
    #print(val)
    return False, data


class TFADecode:
    def __init__(self, display = None):
        self._bits = [True]*80
        self._bytes = bytearray(8)
        self._crc = crc8()
        
        self._rtc = RTC()
        
        self._display = display
        
        # storage for weather data
        self.data_dict = {'battery_low': False,
                          'temperature': None,
                          'humidity': None,
                          'windspeed': None,
                          'windgust': None,
                          'rain': None,
                          'localtime': None
                          }
        
    @property
    def data_json(self):
        import json
        return json.dumps(self.data_dict)

    @property
    def data_raw(self):
        return bits2int(self._bits)
    
    def is_weather_data(self):
        return not self.flag and self.whid==[0,True,0,True]
    
    def is_time_data(self):
        return self.flag and self.whid==[0,True,True,0]
    
    def update(self, val):
        if isinstance(val,int):
            for i in range(bitlen(val),len(self._bits)):
                # fill up rest of bits
                self._bits[i]=False  
            for i in range(bitlen(val)-1,-1,-1):
                self._bits[i] = val % 2
                val = val >> 1            
        else:            
            for i in range(len(val)):
                self._bits[i]=bool(val[i])
            else:
                # fill up rest of bits
                for j in range(i,len(self._bits)):
                    self._bits[j]=False
                
        for i in range(8):
            acc = 0
            for j in range(8):
                acc += self._bits[i*8+j] << j
            self._bytes[i]= acc
            
        self.preamble = self._bits[0:7]
        self.whid = self._bits[7:11]
        self.ident = self._bits[11:19]
        self.flag = self._bits[19]
        print(f'Metadata:flag {self.flag}, whid: {self.whid}, ident: {bits2int(self.ident)}')
            
        self._crc.update(self._bytes)
        
        self._decode()
        
        return self.crcok()
        
    def crcok(self):
        return not(self._crc.digest())
        
    def _decode(self):
        bits = self._bits # local name for speed in micropython
        self.data_dict['battery_low'] = bool(bits[20])
        
        # time data when flag is true and whid is 0,1,1,0
        # weather data whenn flag is False and whid is 0,1,0,1
        if self.is_time_data():
            # got time-data
            year = 2000+bits2int(bits[47:51])*10+bits2int(bits[51:55])
            month = bits2int(bits[58:59])*10+bits2int(bits[59:63])
            day = bits2int(bits[65:67])*10+bits2int(bits[67:71])
            hour = bits2int(bits[25:27])*10+bits2int(bits[27:31])
            minute = bits2int(bits[32:35])*10+bits2int(bits[35:39])
            second = bits2int(bits[40:43])*10+bits2int(bits[43:47])
            self.dfc_tuple = (year, month, day, 0, hour, minute, second, 0)
            self._rtc.datetime(self.dfc_tuple)
            print(self.dfc_tuple)
        elif self.is_weather_data():
            # got weather-data
            self.data_dict['temperature'] = bits2int(bits[21:31])*0.1 - 40
            self.data_dict['humidity'] = bits2int(bits[31:39])
            self.data_dict['windspeed'] = bits2int(bits[39:47])/3
            self.data_dict['windgust'] = bits2int(bits[47:55])/3
            self.data_dict['rain'] = bits2int(bits[55:71])/30
            print(self.data_dict)
        else:
            raise RuntimeError(f'neither time nor weather data in {[int(b) for b in bits]}')
            

__all__ = [twostate_demodulate, TFADecode]

if __name__=='__main__':
    val = 0b1111111010111100000000111010011010110000000001100000100000000010011100000101001
    
    val_raw =b'100100100100100100100111001001110010010010010011100111001110011100111001110011100111001001001001110010011100111001001001110010011100100100111001110011100111001110011100111001110011100100100111001110011100111001110010011100111001110011100111001110011100111001110010011100111001001001001110011100111001110011100100111001001110011100100000000000'
    
    twostate_demodulate(val_raw)
    
    tfa=TFA()
    tfa.update(val)
