from TFADisplay import TFADisplay
from boot import *  # get settings
import time
from machine import Pin, SoftSPI, SoftI2C
import machine
from pycc1101 import TICC1101
import micropython
import TFADecode
import json
import gc
import _thread


# CC1101 Pins
pPower = Pin(26, Pin.OUT)
pGDO0 = Pin(34, Pin.IN)
pGDO2 = Pin(33, Pin.IN)


# SPI Pins
pCS = Pin(5, Pin.OUT)
pSCK = Pin(19, Pin.OUT)
pMOSI = Pin(18, Pin.OUT)
pMISO = Pin(23, Pin.IN)


# I2C
pSDA = Pin(16)
pSCL = Pin(17)


pButton = Pin(25, Pin.IN, Pin.PULL_UP)


# CC1101
pPower.on()
spi = SoftSPI(baudrate=int(1e6), polarity=0, phase=0, bits=8,
              firstbit=SoftSPI.MSB, sck=pSCK, miso=pMISO, mosi=pMOSI)
CC1101 = TICC1101(spi, pCS=pCS, pGDO0=pGDO0, pGDO2=pGDO2, debug=False)
CC1101.reset()
CC1101.selfTest()
CC1101.reset()
CC1101.setDefaultValues()


# CC1101 settings
CC1101.setCarrierFrequency(868)
CC1101.setCarrierFrequencyHz(868410000)  # my oscillator is a bit to low
CC1101.setChannelBandwidth(0, 1)
CC1101.setModulation('ASK')
CC1101.setBaud(2052)  # my oscillator is a bit to low
CC1101.setRXAttenuation(0)
CC1101.setOptimumASKGainControl()
CC1101.setPacketMode("PKT_LEN_FIXED")
CC1101.setPktLen(45)
CC1101.enCRC(False)
CC1101.enFEC(False)
CC1101.setSyncMode(2)
CC1101._writeSingleByte(CC1101.SYNC1, 0b10010010)  # High Byte
CC1101._writeSingleByte(CC1101.SYNC0, 0b01001001)  # Low Byte
# asserts when data is available (fifo is filled above threshold, or end of packet), de-asserts when fifo is empty
CC1101.setGDO0Cfg(0x01)
# asserts when sync-word received, de-asserts when end of package or error
CC1101.setGDO2Cfg(0x06)

CC1101_settings_dict = {"RSSI": -300,
                        "Baudrate": CC1101.getBaud(),
                        "CarrierFrequency": CC1101.getCarrierFrequency(),
                        "ChannelBandwidth": CC1101.getChannelBandwidth()}


# data decoder and display
tfa = TFADecode.TFADecode()
i2c = SoftI2C(pSDA, pSCL)
tfa_display = TFADisplay(128, 64, i2c)

button_press_time = time.ticks_ms()


def button_handler(pin):
    global button_press_time
    now = time.ticks_ms()
    if time.ticks_diff(now, button_press_time) > 400:
        button_press_time = now
        micropython.schedule(tfa_display.next_page, ())


pButton.irq(trigger=machine.Pin.IRQ_FALLING, handler=button_handler)


# loop-control
loop = True  # if True, re-arm CC1101 after if received packet

# state variables
last_successful_deocde_time_str=""
last_time_package_received_time_str=""

# prepare receive buffer to be read to copy data from RXFIFO
RXbuff = bytearray(CC1101._readSingleByte(CC1101.PKTLEN)+60)
RXbuff_mv = memoryview(RXbuff)
# sync word is part of package data
RXbuff[0] = 0b10010010  # SYNC1
RXbuff[1] = 0b01001001  # SYNC0
RXbuff_idx = 2


# spi write buffer for burst RX mode
SPI_buff = bytearray(65)  # RX FIFO size + 1
SPI_buff[0] = 0xFF  # read burst read of RXFIFO
spi_buff_mv = memoryview(SPI_buff)

STATE = {"WiFi":{"SSId": WIFI_ESSID,},
         'CC1101': CC1101_settings_dict
        }

def readout(*args):
    """
    Readout of rx fifo of CC1101.
    """
    global spi_buff_mv, RXbuff_mv, RXbuff_idx

    while True:
        # Get length byte in packet safely according to errata https://www.ti.com/lit/er/swrz020e/swrz020e.pdf
        # RXBytes must be read at rate at least double the baudrate until the same value is read twice
        tmp1 = -1
        tmp2 = -2
        cnt = 0
        while tmp1 != tmp2:
            tmp2 = tmp1
            tmp1 = CC1101._readSingleByte(CC1101.RXBYTES)
            if cnt > 10:  # don't try forever
                raise RuntimeError('communication error')
                return
            cnt += 1
        bytes_available = tmp1

        if (bytes_available & 0b10000000):
            # RXOverflow, reset buffers and revert to idle
            print('RX FIFO in CC1101 overflow')
            CC1101._strobe(CC1101.SFRX)
            RXbuff_idx = 2
            return
        elif bytes_available > 0:
            if (len(spi_buff_mv) < bytes_available+1) or (len(RXbuff_mv) < RXbuff_idx+bytes_available):
                # buffers full, maybe something was left from a previously aborted transmission
                print('receive buffer on MCU is full')
                CC1101._strobe(CC1101.SIDLE)
                RXbuff_idx = 2
                return
            else:
                # bytes available and in receive buffer is enough space
                tmp = RXbuff_mv[RXbuff_idx-1]
                CC1101._pCS.off()
                CC1101._spi.write_readinto(
                    spi_buff_mv[0:bytes_available+1], RXbuff_mv[RXbuff_idx-1:RXbuff_idx+bytes_available])
                CC1101._pCS.on()
                RXbuff_mv[RXbuff_idx-1] = tmp
                RXbuff_idx += bytes_available
        else:
            if pGDO2.value() == 0:
                # end of data
                #print('end of data')
                micropython.schedule(process_TFA_data, (RXbuff[:RXbuff_idx]))
                RXbuff_idx = 2

                if loop:
                    CC1101._setRXState()  # re-arm receiver

            return


# attach ISR to GDO0, i.e. data available or and of packet
def ISR_data_available(irq):
    micropython.schedule(readout, ())


pGDO0.irq(ISR_data_available, Pin.IRQ_RISING, 2)


# configure wifi
if 'WIFI_ESSID' in globals():
    # start WiFi
    import network
    sta_if = network.WLAN(network.STA_IF)
    sta_if.active(False)  # deactivate in case it was already active
    sta_if.active(True)
    sta_if.connect(WIFI_ESSID, WIFI_PASSWORD)
else:
    sta_if = None


# configure MQTT client
if 'MQTT_CLIENTID' in globals():
    from umqttsimple import MQTTClient
    client = MQTTClient(MQTT_CLIENTID, MQTT_SERVER)
else:
    client = None


def display_update_cb(args):
    tfa_display.redraw()


def check_wifi(sta_if):
    """
    If connected, returns the RSSI of the wifi connection. 
    If not connected, schedules reconnect in background and returns None.
    """
    wifi_rssi = None
    if sta_if:
        if sta_if.isconnected():
            wifi_rssi = sta_if.status('rssi')
            STATE["WiFi"]["RSSI"]=wifi_rssi
            wifi_channel = None
            try:
                # it is not on every hardware possible to read-out the channel
                wifi_channel = sta_if.config('channel')
            except OSError:
                pass
            STATE["WiFi"]["Channel"]=wifi_channel
        else:
            # reset WiFi
            STATE["WiFi"]={"SSId": WIFI_ESSID,}
            sta_if.active(False)
            sta_if.active(True)
            sta_if.connect(WIFI_ESSID, WIFI_PASSWORD)
    return wifi_rssi

def get_local_time_str():
    """
    Return local time as string"""
    return '{0}-{1:02d}-{2:02d}T{3:02d}:{4:02d}:{5:02d}'.format(*time.localtime())

def process_TFA_data(data_from_isr):
    """
    Callback for end-of-packet
    """

    global last_time_package_received_time_str
    global last_successful_deocde_time_str

    localtime_str = get_local_time_str()
    STATE["last_time_package_received_time"] = localtime_str

    data_bytes_bin = ''.join(['{:08b}'.format(b)
                             for b in data_from_isr]).encode()
    decode_state, data_bin = TFADecode.twostate_demodulate(data_bytes_bin)
    print(f'Decoded {len(data_bin)} bits')

    # check wifi
    wifi_rssi = check_wifi(sta_if)

    # check CC1101 link quality
    cc1101_rssi = CC1101.getRSSIdBm()
    CC1101_settings_dict['RSSI'] = cc1101_rssi

    # prepare mqtt session
    if wifi_rssi and client:
        client.connect()

    if len(data_bin) >= 79:
        STATE["last_successful_decode_time"] = localtime_str

        # decode data
        tfa.update(data_bin)

        RESULT_json = json.dumps({'raw_int': str(tfa.data_raw),
                                  'isweather': tfa.is_weather_data(),
                                  'istime': tfa.is_time_data(),
                                  })

        # schedule display update (can run in background while MQTT messages are processed)
        tfa_display.update_values(
            wifi_rssi=wifi_rssi, cc1101_rssi=cc1101_rssi, **tfa.data_dict)
        micropython.schedule(display_update_cb, ())

        if wifi_rssi and client:
            if tfa.is_weather_data():
                client.publish(f'tele/{MQTT_TOPIC}/SENSOR', json.dumps({'Time': localtime_str,
                                                                        'TFA': tfa.data_dict}))
            elif tfa.is_time_data():
                client.publish(f'tele/{MQTT_TOPIC}/DFC', str(tfa.dfc_tuple))

    else:
        # TODO: if reoccuring, adjust baudrate or carrier frequency in case it drifted too much
        RESULT_json = json.dumps(
            {'raw': f'{data_bytes_bin}',
                                  'maxdecode': len(data_bin),
                                  })

    print(f'stat/{MQTT_TOPIC}/RESULT', RESULT_json)

    if wifi_rssi and client:
        client.connect()
        STATE["Time"] = localtime_str
        STATE_json = json.dumps(STATE)
        print(f'tele/{MQTT_TOPIC}/STATE', STATE_json)
        client.publish(f'tele/{MQTT_TOPIC}/STATE', STATE_json)

        client.publish(f'stat/{MQTT_TOPIC}/RESULT', RESULT_json)

        client.disconnect()

    gc.collect()  # free memory after processing


def publish_state(timer):
    # check wifi
    wifi_rssi = check_wifi(sta_if)

    STATE["Time"] = get_local_time_str()

    # prepare mqtt session
    if wifi_rssi and client:
        client.connect()
        
        STATE_json = json.dumps(STATE)
        print(f'tele/{MQTT_TOPIC}/STATE', STATE_json)
        client.publish(f'tele/{MQTT_TOPIC}/STATE', STATE_json)

        client.disconnect()

state_timer = machine.Timer(1)
state_timer.init(mode=machine.Timer.PERIODIC, period=60000, callback=publish_state)


def watchdog(*args):
    """
    watchdog/heartbeat-thread to clear buffers in case of buffer overflow or set to RX mode
    """
    global RXbuff_idx
    while True:
        if loop:
            state = CC1101._getMRStateMachineState()
            CC1101_settings_dict['StateMachineState'] = state

            if state == 17:  # RX FIFO overflow
                CC1101._strobe(CC1101.SFRX)
                RXbuff_idx = 2

            if state != 13:  # not in RX state
                publish_state(None)
                print(f'CC1101 state: {state}->13')
                CC1101._setRXState()

        time.sleep(5)


watchdog_thread = _thread.start_new_thread(watchdog, ())
