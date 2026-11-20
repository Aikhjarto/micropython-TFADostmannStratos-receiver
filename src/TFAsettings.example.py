from micropython import const

# Copy to the device and fill in; do not commit real values.
WIFI_ESSID = const('your-ssid')
WIFI_PASSWORD = const('your-password')

HOSTNAME = const('TFA')

MQTT_CLIENTID = const('TFA')
MQTT_SERVER = const('mqttbroker.example.lan')
MQTT_TOPIC = const('tfa')

SYSLOG_SERVER = 'syslog.example.lan'
