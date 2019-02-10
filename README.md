# AirQualityMultiSensorESP

Beeing a synergy-project betwen "BME680_UniversalSensor" and "Raum(-luft)sensor (Temp, Hum, Co2, VOC, LUX)"
Issued by @hdgucken and @herrmannj @FHEM.de

Using BOSCH Sensortec static libalgosec.a library for ESP8266  BSEC V1.4.7.1.
Which need some additional compiler Arduino settings, according BSEC prerequisites.

There are a lot of configurable additional sensors:
DHT22
BH1750
MH-Z19
etc...
Multiple protocols: UDP or LaCrosse via RFM69CW

Intentioned to be compiled by Arduino IDE or Visual Micro, using MS Visual Studio VS2017 community edition.

Hardware:
Wemos D1 oder NodeMCU
BME680-Breakout-Board (OSH) oder besser PeMues-Kombi: BME680+BH1750 Breakout
OLED Display: "0,96 Zoll I2C IIC 12864 128X64 Pixel OLED LCD Display Modul SSD1306 Chip 4 Pin"
TFT_1:  "ST7735 128x160" (Sainsmart) "1.8 SPI TFT LCD Display Module 128x160 Chip ST7735/ILI9341"
TFT_2:  WEMOS TFT 2.4 Touch Shield (hier) 320 × 240 Pixel, TFT-Treiber IC: ILI9341
TFT_3:  ePaper-Hat 2,7 inch, 274x176 Pixel, "Tri-Color 2.7 inch E-Ink Display Module"
Platine von PapaRomeo
MH-Z19 (Ali) + iAQcore (AMS) + BME680 (Reichelt)[/url]

Arduino Libraries:
ESP8266(Wemos) TFT_eSPI
BSEC-Installation, siehe z.B. hier: HowTo und hier BSEC-Arduino-library
BME_680_Driver: BoschSensortec
EEPROM-Lib
ESP8266core (WiFi)
MH-Z19-Lib

Reference Project: https://forum.fhem.de/index.php?topic=97161.msg903521#msg903521

Prelimininay stuff:
===================

This project is based on hdguckens cc_sensor, HCS@fhem has made some changes to upgrade it to the UniversalSensor, many thanks to HCS !
The main component is an ESP8266 nodemcu, but i dont use the wifi section, wireless transmission is done by an RFM69CW module.
Since V3.0 you can use ESP8266 based NodeMCU/Wemos D1 mini OR STM32F103Cx based BluPill/Maple mini boards !
Hardware setup you can find in the sketch.
The RFM69 transmit at 868.30 MHz with LaCrosse Weatherstation Protocoll. The signal is decoded by a LaCrosseGateway module (by HCS),
wich transmitt the data to a FHEM server. There is created a LaCrosse like weather sensor with all the sensor data as readings.
This sensor gives you the following data:
temerature (degree Celsius), humidity (% rH), air pressure (hPa/mBar), air quality, light intensity (Lux) and battery voltage (V).
You can use an optional 128x64 OLED Display with SH1106/SSD1306 Controller, to display the data. It is autodetected, simply connect it or not.
The Lightsensor (BH1750) is optional too, it not need to be connected.
The RFM69CW not need to be connected too, in this case, no wireless transmission is possible. The data where transmitted
by the serial port only (115200 Baud).
