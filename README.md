# GSM/GPS Tracker for TTGO SIM7000G Board #

This is a GPS tracker that uploads its position to an MQTT broker by using a mobile phone modem to connect to the internet.  It is designed and tested on the TTGO SIM7000G board with an OLED added.

# Tracker #

## Hardware ##

This should work with any GSM/LTE modem supported by TinyGSM, **provided it includes GPS capability,** however it has only been tested with a TTGO ESP32 board and SIM7000G modem.

This particular board has a battery holder for an 18560 LiPo.  Remember that such batteries are generally not designed for use in low temperatures, so use plenty of foam insulation.  Since the device only needs to work at low altitudes then a short period of being cold shouldn't matter.  That said, **I have yet to flight-test the device/firmware** so use at your own risk.

Status messages are displayed on a small OLED display connected to the ESP32 board by 4 wires.  Any such board 128x64 pixels should work fine.  The firmware should operate happily with that not connected however that too is untested.  Connect the OLED to Vcc (3V3), GND, SDA and SCL (pins 21 and 22).

For testing, you can use a USB-C lead for power.  Use a good one as GSM can use a lot of current for short periods.


## Software ##

Use the Arduino ID to load and build the source code, and program the device.  Test with the serial port set to 115,200 baud.

You will need to install some ESP32 board definitions if you haven't already got those, and choose "ESP32 Dev Board" or whatever.

There are several dependencies which you can install from the Arduino IDE:

```
TinyGSM
PubSubClient
Adafruit GFX
Adafruit SSD1306
```

Configuration is within the source file.  You need to set:

# Operation #

You can watch progress on the OLED or with a terminal program connected to the USB serial port.

On power-up, the program sets up the hardware including the modem which can take several seconds.

The program then enters a loop, first waiting for a GPS position, then connecting to the mobile network, connecting to the internet, connecting to the MQTT broker, then publishing telemetry in UKHAS format to the broker.

After that it waits for 10 seconds and starts the loop again.  With constant GPS and mobile connections, there should be a new telemetry sentence published to the broker every 15 seconds or so.

At present, the mobile and internet and MQTT connections are re-established each loop.  This is simple and should make it more robust where the mobile signal in particular is patchy, including during initial ascent, final descent, and landing if the signal is poor.  There is some extra bandwidth usage doing it this way, however in my tests the usage was around 1KB/message, which with a ThingsMobile IOT SIM on PAYG has a cost of €0.0001 per message, or about €0.024 per hour.  This is much much much less than using SMS on PAYG.