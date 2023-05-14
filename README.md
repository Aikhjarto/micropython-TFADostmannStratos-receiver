Receiver implementation a TFA Dostmann Stratos 35.1077.54.S2 weather station with 30.3151 thermo/hygro sensor), a 30.3152 rainsensor, and a 30.3153 anemometer for micropython using a CC1101 868 MHz receiver and an SSD1306 128x64px OLED display.


Received data is published to an MQTT server using the built-in WiFi if available, to the following topics
```
tele/{MQTT_TOPIC}/SENSOR
tele/{MQTT_TOPIC}/DFC
tele/{MQTT_TOPIC}/STATE
stat/{MQTT_TOPIC}/RESULT
```
as JSON object.

* `SENSOR` and `DFC` are published each time weather and time data are received respectivley.
* `STATE` contains internal like RSSI, and last successfull sensor reads and is publishd every 60 seconds or whenever some important state change happened.
* `RESULT` contains interals on decoding the bytestream received from  the TFA station. 

## Upload to Device
install required tools
```
pip install --user -r requirements.txt
```

Precompile libraries
```sh
mkdir -p lib
3rdParty/micropython-font-to-py/font_to_py.py -x /usr/share/fonts/truetype/NotoSans-Condensed.ttf   15 lib/NotoSansCondensed15.py
3rdParty/micropython-font-to-py/font_to_py.py -x /usr/share/fonts/truetype/NotoSans-Condensed.ttf   18 lib/NotoSansCondensed18.py
3rdParty/micropython-font-to-py/font_to_py.py -x /usr/share/fonts/truetype/NotoSans-Condensed.ttf   28 lib/NotoSansCondensed18.py

find lib -name "*.py" -exec mpy-cross {} \;

mpy-cross 3rdParty/pycc1101/pycc1101/pycc1101.py -o lib/pycc1101.mpy
mpy-cross 3rdParty/micropython-font-to-py/writer/writer.py -o lib/writer.mpy
mpy-cross 3rdParty/micropython-lib/micropython/umqtt.simple/umqtt/simple.py -o lib/umqttsimple.mpy
mpy-cross 3rdParty/micropython-lib/micropython/drivers/display/ssd1306/ssd1306.py -o lib/ssd1306.mpy
mpy-cross 3rdParty/crc8/crc8.py -o lib/crc8.mpy
mpy-cross 3rdParty/crc/crc.py -o lib/crc.mpy
```

Upload everything to microcontroller
```sh
find lib -name "*.mpy" -exec ampy -p /dev/ttyUSB0 put {} {} \;
find src -name "*TFA*.py" -print0 | xargs -0 -L1 sh -c 'ampy -p /dev/ttyUSB0 put "$0" "$(basename "$0")"'
```
