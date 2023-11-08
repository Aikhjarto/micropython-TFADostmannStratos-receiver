# This file is executed on every boot (including wake-boot from deepsleep)
#import esp
#esp.osdebug(None)
import webrepl
webrepl.start()

from micropython import const

WIFI_ESSID = const('ESSID')
WIFI_PASSWORD = const('WIFI_PASSWORD')

HOSTNAME = const('TFA')

MQTT_CLIENTID = const('TFA')
MQTT_SERVER = const('mqttbroker.private.lan')
MQTT_TOPIC = const('tfa')

SYSLOG_SERVER='syslog.private.lan'