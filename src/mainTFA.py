from boot import *  # get settings
import time
from machine import Pin, SoftSPI, SoftI2C
import machine
from pycc1101 import TICC1101
import micropython
import TFA
import json
import gc
import _thread


pPower = Pin(26,Pin.OUT)
pPower.on()

# SPI for CC1101
pCS = Pin(5, Pin.OUT)
pCS.on()
pSCK = Pin(19, Pin.OUT)
pMOSI = Pin(18, Pin.OUT)
pMISO = Pin(23, Pin.IN)
spi = SoftSPI(baudrate=int(1e6), polarity=0, phase=0, bits=8, firstbit = SoftSPI.MSB, sck = pSCK, miso = pMISO, mosi = pMOSI)


# CC1101
pGDO0 = Pin(34, Pin.IN)
pGDO2 = Pin(33, Pin.IN)
CC1101 = TICC1101(spi, pCS=pCS, pGDO0=pGDO0, pGDO2=pGDO2, debug = False)
CC1101.reset()
CC1101.selfTest()
CC1101.reset()
CC1101.setDefaultValues()

# CC1101 settings
CC1101.setCarrierFrequency(868)
CC1101.setCarrierFrequencyHz(868400000) # my oscillator is a bit to low
CC1101.setChannelBandwidth(0,1)
CC1101.setModulation('ASK')
CC1101.setBaud(2052) # my oscillator is a bit to low
CC1101.setRXAttenuation(0)
CC1101.setOptimumASKGainControl()


# CC1101 packet mode
CC1101.setPacketMode("PKT_LEN_FIXED")
CC1101.setPktLen(45)
CC1101.enCRC(False)
CC1101.enFEC(False)
CC1101.setSyncMode(2) 
CC1101._writeSingleByte(CC1101.SYNC1, 0b10010010)  # High Byte
CC1101._writeSingleByte(CC1101.SYNC0, 0b01001001)  # Low Byte


# prepare receive buffer to be read to copy data from RXFIFO
RXbuff = bytearray(CC1101._readSingleByte(CC1101.PKTLEN)+60)
RXbuff_mv = memoryview(RXbuff)
# sync word is part of package data
RXbuff[0]=0b10010010 # SYNC1
RXbuff[1]=0b01001001 # SYNC0
RXbuff_idx = 2
data_available=0 # number of valid bytes in buffer

# spi write buffer for burst RX mode
SPI_buff=bytearray(65) # RX FIFO size + 1
SPI_buff[0]=0xFF # read burst read of RXFIFO
spi_buff_mv = memoryview(SPI_buff)


def ISR_data_available(irq):
    global data_available, spi_buff_mv, RXbuff_mv, RXbuff_idx
    data_available = 0  # state that buffer is currently written to
    self=CC1101
    while True:
        # Get length byte in packet safely according to errata https://www.ti.com/lit/er/swrz020e/swrz020e.pdf
        # RXBytes must be read at rate at least double the baudrate until the same value is read twice
        tmp1=-1; tmp2=-2; cnt=0
        while tmp1!=tmp2:
            tmp2 = tmp1
            tmp1 = self._readSingleByte(self.RXBYTES)
            if cnt>10:  # don't try forever
                print('communication error')
                return
        bytes_available=tmp1            
        
        if (bytes_available & 0b10000000):
            # RXOverflow, reset buffers and revert to idle
            print('RX FIFO in CC1101 overflow')
            CC1101._strobe(CC1101.SFRX)
            RXbuff_idx = 2
            return
        elif bytes_available > 0:
            # bytes available and place in receive buffer is enough
            if (len(spi_buff_mv) < bytes_available+1) or (len(RXbuff_mv) < RXbuff_idx+bytes_available):
                # buffers full, maybe something was left from a previously aborted transmission
                print('receive buffer on MCU is full')
                self._strobe(self.SIDLE)
                RXbuff_idx = 2
                return
            else:
                tmp = RXbuff_mv[RXbuff_idx-1]
                self._pCS.off()
                self._spi.write_readinto(spi_buff_mv[0:bytes_available+1], RXbuff_mv[RXbuff_idx-1:RXbuff_idx+bytes_available])
                self._pCS.on()
                RXbuff_mv[RXbuff_idx-1]=tmp
                RXbuff_idx += bytes_available
        else:
            if pGDO2.value() == 0:
                # end of data
                #print('end of data')
                micropython.schedule(process, ())
                data_available=RXbuff_idx
                RXbuff_idx=2
            return

micropython.alloc_emergency_exception_buf(100)  # allocate buffer for stack trace in IRQ
CC1101.setGDO0Cfg(0x01) # asserts when data is available (fifo is filled above threshold, or end of packet), de-asserts when fifo is empty
CC1101.setGDO2Cfg(0x06) # asserts when sync-word received, de-asserts when end of package or error
pGDO0.irq(ISR_data_available, Pin.IRQ_RISING, 2)

if 'WIFI_ESSID' in globals():
    # start WiFi
    import network
    sta_if = network.WLAN(network.STA_IF)
    sta_if.active(False) # deactivate in case it was already active
    sta_if.active(True)
    sta_if.connect(WIFI_ESSID, WIFI_PASSWORD)
else:
    sta_if = None

if 'MQTT_CLIENTID' in globals():
    from umqttsimple import MQTTClient
    client = MQTTClient(MQTT_CLIENTID, MQTT_SERVER)
else:
    client = None
    
from TFADisplay import TFADisplay
i2c = SoftI2C(Pin(16),Pin(17))
tfa_display = TFADisplay(128,64,i2c)
tfa=TFA.TFADecode()

loop = True

def display_update_cb(args):
    tfa_display.update(*args)
    
def process(*args):
    global data_available
    
    # briefly disable irq to safely copy data
    irq_state = machine.disable_irq()
    data_from_isr = RXbuff[:data_available]
    machine.enable_irq(irq_state)
    
    if loop:
        CC1101._setRXState() # re-arm receiver
        
    data_bytes_bin = ''.join(['{:08b}'.format(b) for b in data_from_isr]).encode()
    decode_state, data_bin = TFA.twostate_demodulate(data_bytes_bin)
    print(decode_state, len(data_bin))
    
    # check wifi
    wifi_rssi = None
    if sta_if:
        if sta_if.isconnected():
            wifi_rssi = sta_if.status('rssi')
        else:
            # reset WiFi
            sta_if.active(False)
            sta_if.active(True)
                
    cc1101_rssi = CC1101.getRSSIdBm()
    
    if len(data_bin) >= 79:
        tfa.update(data_bin)
        if wifi_rssi and client:
            # schedule display update (should run in background in case mqtt connection throws an exception)
            micropython.schedule(display_update_cb, (tfa.data_dict, tfa._rtc.datetime(), wifi_rssi, cc1101_rssi))
            
            json_data = json.dumps({'wifi_rssi': wifi_rssi,
                                                   'cc1101_rssi': cc1101_rssi,
                                                   'raw_int': str(tfa.data_raw),
                                                   'isweather': tfa.is_weather_data(),
                                                   'istime': tfa.is_time_data(),
                                                   })
            print(json_data)
            
            client.connect()
            client.publish('tfa/tele', json_data)
            if tfa.is_weather_data():
                client.publish('tfa/weather', tfa.data_json)
            elif tfa.is_time_data():
                client.publish('tfa/time', str(tfa.dfc_tuple))
            client.disconnect()
        
    else:
        # TODO: adjust baudrate
        if wifi_rssi and client:
            client.connect()
            json_data = json.dumps({'wifi_rssi': sta_if.status('rssi'),
                                                   'cc1101_rssi': cc1101_rssi,
                                                   'raw': f'{data_bytes_bin}',
                                                   'maxdecode': len(data_bin),
                                                   }) 
            print(json_data)
            
            client.publish('tfa/tele', json_data)
            client.disconnect()

    data_available = 0

    gc.collect() # free memory after processing


def watchdog(*args):
    # watchdog to reset in case of buffer overflow
    global RXbuff_idx
    while True:
        if loop: 
            state = CC1101._getMRStateMachineState()
            print(f'{state}', end=' ')
            if state == 17: # RX FIFO overflow
                CC1101._strobe(CC1101.SFRX)
                RXbuff_idx=2
            if state != 13:  # not in RX state
                CC1101._setRXState()
        time.sleep(30)
        
watchdog_thread = _thread.start_new_thread(watchdog, ())
