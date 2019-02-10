/*
    * Visual Micro is in vMicro>General>Tutorial Mode
    * ===============================================

    Name:       AirQualityMultiSensorESP_VS.ino
    Created:	09.02.2019 11:46:49
    Author:     juergs@fhem
    Copyright:  BoschSensortec: see "BoschCopyrightCredentials.txt"

    ***********************************************************************************/

// Define User Types below here or use a .h file
/*==============================================*/
/*--- select your hardware options */
#define VERSION                 1.0f
#define HAS_RFM69               false   //is auto detected, if not found, sensor works without RFM69, data only on serial port
#define HAS_OLED                true    //is auto detected, if not found, sensor works without OLED (SH1106 or SSD1306)
#define HAS_TFT                 true    //is auto detected, if not found, sensor works without OLED (SH1106 or SSD1306)
#define HAS_LIGHTSENSOR         false   //is auto detected, if not found, sensor works without BH1750
#define VCC_MEASURE             true    //if not needed, you can disable it here
#define HAS_DHT22               true    //if not needed, you can disable it here
#define HAS_IAQ_CORE            false
#define HAS_MHZ19               false  
#define HAS_LDR                 false
#define SOFT_SPI                false   //if you need SOFT-SPI, set true
#define DEBUG                   false   //activate debug mode
#define USE_EEPROM              true

//--- LEDs
#define LED_ESP_BUILTIN         D4
#define LED_ESP_NODEMCU         D0
#define LEDpin                  LED_ESP_BUILTIN     //auto or set pin of your choice

/*===============================================*/
/* header files                                  */
/*===============================================*/
#include <Arduino.h>

//--- hardware 
#include <EEPROM.h>
#include <Wire.h>
#include "bsec_integration.h"

#if USE_EEPROM
    #include "HandleEeprom.h"
#endif

#if HAS_RFM69
    #include "RFMxx.h"
    #include "SensorBase.h"
    //--- bridge to LaCrosseGateway using RFM69CW
    //#include "UniversalSensor.h"
#endif

#if HAS_OLED
    #include <Adafruit_GFX.h>
    #include "Adafruit_SSD1306.h"
#endif

#if HAS_LIGHTSENSOR
    #include "AS_BH1750.h"
#endif

#if HAS_MHZ19
    #include <MHZ.h>
#endif 

#if HAS_IAQ_CORE
    #include "iAQcore.h" // iAQ-Core driver
#endif  //--- HAS_IAQ_CORE

#if HAS_DHT22
    #include <DHT.h>
    #include <DHT_U.h>
#endif

//--- custom images
#include "images.h"
#include "RobotoML_Plain.h"

//--- esp8266 specific stuff
#ifdef ESP8266
    #include <BearSSLHelpers.h>
    #include <CertStoreBearSSL.h>
    //--- web stuff 
    #include <ESP8266WebServer.h>
    #include <ESP8266WiFi.h>
    #include <ESP8266WiFiAP.h>
    #include <ESP8266WiFiGeneric.h>
    #include <ESP8266WiFiMulti.h>
    #include <ESP8266WiFiScan.h>
    #include <ESP8266WiFiSTA.h>
    #include <ESP8266WiFiType.h>
    //--- udp 
    #include <Ticker.h>
    #include <WiFiClient.h>
    #include <WiFiClientSecure.h>
    #include <WiFiClientSecureAxTLS.h>
    #include <WiFiClientSecureBearSSL.h>
    #include <WiFiManager.h>
    #include <WiFiServer.h>
    #include <WiFiServerSecure.h>
    #include <WiFiServerSecureAxTLS.h>
    #include <WiFiServerSecureBearSSL.h>
    #include <WiFiUdp.h>
#endif

/*=================*/
/*   Variables     */
/*=================*/

//-- multicast declarations
WiFiUDP         Udp;
IPAddress       ipMulti(239, 255, 255, 250);    // site-local
unsigned int    portMulti = 2085;               // port
char            incomingPacket[255];            // UDP in-buffer
char            message[512];                   //--- udp message to send

//--- timer settings
unsigned long   prevLedMillis = millis();       // counter main loop for signaling led
unsigned long   prevDhtMillis = millis();       // counter main loop for dht
unsigned long   prevIaqMillis = millis();       // counter main loop for iaq
unsigned long   prevMhz19Millis = millis();     // counter main loop for MH-Z19
unsigned long   prevLdrMillis = millis();       // counter main loop for LDR

//--- signal LEDs
unsigned long   sigLedOn = 50;                  // ms for led on
unsigned long   sigLedOff = 450;                // ms for led off
unsigned long   intervalDht = 10000;            // default
unsigned long   intervalIaq = 10000;            // default
unsigned long   intervalMhz19 = 10000;          // default
unsigned long   intervalLdr = 1000;             // default
unsigned long   intervalLed = 0;                // 

uint8_t         NODEID;
float           TEMPOFFSET;
float           ALTITUDE;
HandleEeprom    eeprom;

char            serviceMsg[512];
Ticker          serviceMessageTimer;            // cyclic service message (alive)

#if HAS_MHZ19
    char mhz19Msg[32];
    
    //---  PWM reading, unused
    #define CO2_IN 10
    
    //--- uart pins
    #define MH_Z19_RX D7  // D7
    #define MH_Z19_TX D8  // D6

    MHZ co2(MH_Z19_RX, MH_Z19_TX, CO2_IN, MHZ19B);
#endif

#if HAS_DHT22
    #define DHTPIN D5           // NodeMCU D5
    #define DHTTYPE DHT22       // !!! https://github.com/adafruit/DHT-sensor-library/issues/94                                          
    
    char    str_temp[6];
    char    str_hum[6];
    char    dhtMsg[512];
    
    DHT     dht(DHTPIN, DHTTYPE);    
#endif

#if HAS_IAQ_CORE
    char    iaqMsg[512];
    iAQcore iaqcore;
#endif

#if HAS_RFM69
    unsigned long DATA_RATE = 17241ul;      //default data rate (for transmit on RFM69)
    unsigned long INITIAL_FREQ = 868300;    //default frequency in kHz (5 kHz steps, 860480 ... 879515)
    bool RFM69 = false;                     //if RFM69 is not detected
    uint8_t loop_count_lim = 5;             //time to wait between data transmissions (20: 20 * 3sec = 60sec)
    uint8_t loop_counter;
    /*--- RFM69, PIN-config SPI (GPIO XX or pin Dx): */
    #ifdef ESP8266
        #define RFM_SS             D8           //15    SS pin -> RFM69 (NSS)  //for both Soft- and HW-SPI
        #define RFM_MISO           D6           //12  MISO pin <- RFM69 (MOSI) //only used by soft spi
        #define RFM_MOSI           D7           //13  MOSI pin -> RFM69 (MISO) //only used by soft spi
        #define RFM_SCK            D5           //14  SCK  pin -> RFM69 (SCK)  //only used by soft spi
    #elif defined(__STM32F1__)
        #define RFM_SS             PA4         // SS   pin -> RFM69 (NSS)  //for both Soft- and HW-SPI
        #define RFM_SCK            PA5         // SCK  pin -> RFM69 (SCK)  //only used by soft spi
        #define RFM_MISO           PA6         // MISO pin <- RFM69 (MOSI) //only used by soft spi
        #define RFM_MOSI           PA7         // MOSI pin -> RFM69 (MISO) //only used by soft spi
    #endif

    RFMxx  rfm(RFM_MOSI, RFM_MISO, RFM_SCK, RFM_SS, SOFT_SPI);
#endif
   
#if HAS_OLED
    bool OLED                    = false;      //if display not detected
    Adafruit_SSD1306             display;
#endif

#if HAS_LIGHTSENSOR
    bool BH1750 = false;      //if BH1750 not detected
    AS_BH1750                    bh1750;
#endif

#ifdef ESP8266
    #if VCC_MEASURE
        ADC_MODE(ADC_VCC);    //--- use esp internal vcc measuring
    #endif
#endif

//=====================================================                              
//--- Define Function Prototypes that use User Types below here or use a .h file
//--- Define Functions below here or use other .ino or cpp files
//=====================================================
//=====================================================

/* blink led */
static void blink(byte pin, byte n = 3, int del = 50)
{
    for (byte i = 0; i < n; ++i)
    {
        digitalWrite(pin, LOW);
        delay(del);
        digitalWrite(pin, HIGH);
        delay(del);
    }
}
//---------------------------------------------------
void dim_display(byte brightness)
{
    #if HAS_OLED   
        display.ssd1306_command(SSD1306_SETCONTRAST);
        display.ssd1306_command(brightness);   
    #endif
}
/*----------------------------------------------------------------------------------------------*/
/*!
* @brief           Write operation in either Wire or SPI
* param[in]        dev_addr        Wire or SPI device address
* param[in]        reg_addr        register address
* param[in]        reg_data_ptr    pointer to the data to be written
* param[in]        data_len        number of bytes to be written
* @return          result of the bus communication function
*/
int8_t bus_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data_ptr, uint16_t data_len)
{
    Wire.beginTransmission(dev_addr);
    Wire.write(reg_addr);    /* Set register address to start writing to */                            
    for (int index = 0; index < data_len; index++)
    {
        Wire.write(reg_data_ptr[index]);  /* Write the data */
    }
    #ifdef __STM32F1__
        Wire.endTransmission();
        return 0;
    #else
        return (int8_t)Wire.endTransmission();
    #endif
}
/*----------------------------------------------------------------------------------------------*/
/*!
* @brief           Read operation in either Wire or SPI
* param[in]        dev_addr        Wire or SPI device address
* param[in]        reg_addr        register address
* param[out]       reg_data_ptr    pointer to the memory to be used to store the read data
* param[in]        data_len        number of bytes to be read
* @return          result of the bus communication function
*/
int8_t bus_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data_ptr, uint16_t data_len)
{
    int8_t comResult = 0;
    Wire.beginTransmission(dev_addr);
    Wire.write(reg_addr);                              /* Set register address to start reading from */
    comResult = Wire.endTransmission();

    delayMicroseconds(150);                           /* Precautionary response delay */
    Wire.requestFrom(dev_addr, (uint8_t)data_len);    /* Request data */

    int index = 0;
    while (Wire.available())  /* The slave device may send less than requested (burst read) */
    {
        reg_data_ptr[index] = Wire.read();
        index++;
    }
    #ifdef __STM32F1__
        return 0;
    #else
        return comResult;
    #endif
}
/*----------------------------------------------------------------------------------------------*/
/*!
* @brief           System specific implementation of sleep function
* @param[in]       t_ms    time in milliseconds
* @return          none
*/
void sleep(uint32_t t_ms)
{
    delay(t_ms);
}
/*----------------------------------------------------------------------------------------------*/
/*!
* @brief           Capture the system time in microseconds
* @return          system_current_time    current system timestamp in microseconds
*/
int64_t get_timestamp_us()
{
    return (int64_t)millis() * 1000;
}
/*----------------------------------------------------------------------------------------------*/
/*!
* @brief           Load previous library state from non-volatile memory
* @param[in,out]   state_buffer    buffer to hold the loaded state string
* @param[in]       n_buffer        size of the allocated state buffer
* @return          number of bytes copied to state_buffer
*/
uint32_t state_load(uint8_t *state_buffer, uint32_t n_buffer)
{    
    // Load a previous library state from non-volatile memory, if available.
    // Return zero if loading was unsuccessful or no state was available,
    // otherwise return length of loaded state string. 
    return 0;
}
/*----------------------------------------------------------------------------------------------*/
/*!
* @brief           Save library state to non-volatile memory
* @param[in]       state_buffer    buffer holding the state to be stored
* @param[in]       length          length of the state string to be stored
* @return          none
*/
void state_save(const uint8_t *state_buffer, uint32_t length)
{
    // ...
    // Save the string some form of non-volatile memory, if possible.
    // ...
}
/*----------------------------------------------------------------------------------------------*/
/*!
* @brief           Load library config from non-volatile memory
* @param[in,out]   config_buffer    buffer to hold the loaded state string
* @param[in]       n_buffer        size of the allocated state buffer
* @return          number of bytes copied to config_buffer
*/
uint32_t config_load(uint8_t *config_buffer, uint32_t n_buffer)
{
    // ...
    // Load a library config from non-volatile memory, if available.
    //
    // Return zero if loading was unsuccessful or no config was available,
    // otherwise return length of loaded config string.
    // ...
    return 0;
}
/*----------------------------------------------------------------------------------------------*/
/*!
* @brief           Handling of the ready outputs
* @param[in]       timestamp       time in nanoseconds
* @param[in]       iaq             IAQ signal
* @param[in]       iaq_accuracy    accuracy of IAQ signal
* @param[in]       temperature     temperature signal
* @param[in]       humidity        humidity signal
* @param[in]       pressure        pressure signal
* @param[in]       raw_temperature raw temperature signal
* @param[in]       raw_humidity    raw humidity signal
* @param[in]       gas             raw gas sensor signal
* @param[in]       bsec_status     value returned by the bsec_do_steps() call
* @return          none
*/
void output_ready(int64_t timestamp, float iaq, uint8_t iaq_accuracy, float temperature, float humidity, float pressure, float raw_temperature, float raw_humidity, float gas, bsec_library_return_t bsec_status, float static_iaq, float co2_equivalent, float breath_voc_equivalent)
{
    #if DEBUG
        unsigned long cycle_start = millis();
    #endif
        /* get battery voltage */
    #if VCC_MEASURE
        #ifdef ESP8266
            float vcc = ESP.getVcc() / 930.9; //correction for 220k/100k voltage divider to A0 on NodeMCU and D1 mini...
        #elif defined(__STM32F1__)
            //int vcc = analogRead(A0);
            float vcc = 3.3; //TODO: for STM32
        #else
            float vcc = 0xFF;
        #endif
    #else
        float vcc = 0xFF;
    #endif

    /* get light level */
    #if HAS_LIGHTSENSOR
        float lux;
        if (BH1750)
        {
            lux = bh1750.readLightLevel();
    
        }
        else
        {
            lux = 0xFFFFFF;
        }

        #if HAS_OLED && HAS_LIGHTSENSOR       
            /* display dimm, if lightlevel < 10lux */
            if (lux < 10)
                dim_display(0);
            else
                dim_display(0x80);
        #endif

    #else
        float lux = 0xFFFFFF;
    #endif

    pressure /= pow(((float) 1.0 - ((float)ALTITUDE / 44330.0)), (float) 5.255);
    pressure /= 100;

    #if HAS_OLED    
        display.setCursor(0, 0);
        display.clearDisplay();
    #endif

    /* Output measurings */
    Serial.print("[");
    Serial.print(timestamp / 1e6);
    Serial.print("] P: ");
    Serial.print(pressure, 1);
    Serial.print("| T: ");
    Serial.print(temperature);
    Serial.print("| rH: ");
    Serial.print(humidity);
    Serial.print("| IAQ: ");
    Serial.print(iaq);
    Serial.print(" (");
    Serial.print(iaq_accuracy);
    Serial.print(")");
    /***** new since BSEC V1.4.7.1: *****/
    Serial.print("| Static IAQ: ");
    Serial.print(static_iaq);
    Serial.print("| CO2e: ");
    Serial.print(co2_equivalent);
    Serial.print("| bVOC: ");
    Serial.println(breath_voc_equivalent);
    /************************************/
    Serial.print("| Gas: ");
    Serial.print(gas);
#if VCC_MEASURE
    Serial.print("| UBat: ");
    Serial.print(vcc, 1);
    Serial.print("V");
#endif
#if HAS_LIGHTSENSOR
    if (BH1750)
    {
        Serial.print("| Light: ");
        Serial.print(lux);
        Serial.print("lx");
    }
#endif
    Serial.println("");

    /* on OLED Display */
    #if HAS_OLED
        display.setTextSize(2);
        display.print(temperature, 1);
        display.setTextSize(1);
        display.setCursor(48, 7);
        display.write(247);
        display.print("C ");

        display.setTextSize(2);
        display.setCursor(74, 0);
        display.print(humidity, 0);
        display.setTextSize(1);
        display.setCursor(98, 7);
        display.println("%");
        display.println();

        display.print("Luftdruck: ");
        display.print(pressure, 0);
        display.println("hPa");

        display.print("Luftguete: ");
        if (iaq_accuracy > 0)
        {
            display.print(iaq, 0);
            display.print(" (");
            display.print(iaq_accuracy);
            display.print(")");
        }
        else
        {
            display.print("berechne..");
        }
        display.println();

        #if HAS_LIGHTSENSOR
            if (BH1750)
            {
                display.print("Licht    : ");
                display.print(lux, 0);
                display.println("lx");
            }
        #endif

        #if VCC_MEASURE
            display.print("Batterie : ");
            display.print(vcc, 1);
            display.println("V");
        #endif

            display.print("Gas      : ");
            display.print(gas / 1000 + 0.005, 2);
            display.println("kOhm");
            display.display();
    #endif

    #if HAS_RFM69
        if (RFM69)
        {
            loop_counter += 1;
            if (loop_counter >= loop_count_lim) //send every (loop_count_lim * 3) seconds
            {
                loop_counter = 0;

                UniversalSensor::Frame data;
                data.ID = NODEID;
                data.Flags = 0;
                data.Temperature = temperature;
                data.Humidity = humidity;
                data.Pressure = pressure;
                data.Gas1 = iaq;
                data.Gas2 = gas;
                data.Lux = lux;
                data.Voltage = vcc;
                data.Version = VERSION * 10;
                data.Rain = 0xFFFF;
                data.WindDirection = 0xFFFF;
                data.WindGust = 0xFFFF;
                data.WindSpeed = 0xFFFF;
                data.Debug = 0xFFFFFF;

                byte bytes[UniversalSensor::FRAME_LENGTH];
                UniversalSensor::EncodeFrame(&data, bytes);
                rfm.SendArray(bytes, UniversalSensor::FRAME_LENGTH);
                rfm.PowerDown();

                #if DEBUG
                    Serial.print(sizeof(bytes));
                    Serial.print(" bytes");
                    Serial.println(" sent.");
                    Serial.println();
                #endif

                blink(LEDpin, 1, 25);
            }
        }
    #endif

    #if DEBUG
        unsigned long cycle = millis() - cycle_start;
        Serial.print("Zykluszeit (ms): "); Serial.println(cycle);
    #endif
}
/*----------------------------------------------------------------------------------------------*/
//--- ticker message
void serviceAlive() 
{
    sprintf(serviceMsg, "F:SERVICE;UP:%ld;", millis());
};
/*----------------------------------------------------------------------------------------------*/
#if HAS_DHT22
    void getDhtReadings() 
    {
        float h = dht.readHumidity();
        float t = dht.readTemperature();
        if (isnan(t) || isnan(h)) 
        {
            prevDhtMillis = millis() - intervalDht + 2000;  // retry in 2 sec
        }
        else 
        {
            //TODO
            //display.setTempHum(t, h);
            dtostrf(h, 3, 1, str_hum);
            dtostrf(t, 3, 1, str_temp);
            sprintf(dhtMsg, "F:TH_A;T:%s;H:%s;", str_temp, str_hum);
        };
    };
#endif // HAS_DHT22
/*----------------------------------------------------------------------------------------------*/
#if HAS_IAQ_CORE
    void getIaqReadings() 
    {
        uint16_t eco2;
        uint16_t stat;
        uint32_t resist;
        uint16_t etvoc;

        iaqcore.read(&eco2, &stat, &resist, &etvoc);

        if (stat & IAQCORE_STAT_I2CERR) {
            DEBUG_PRINT("iAQcore: I2C error");
        }
        else if (stat & IAQCORE_STAT_ERROR) {
            DEBUG_PRINT("iAQcore: chip broken");
        }
        else if (stat & IAQCORE_STAT_BUSY) {
            DEBUG_PRINT("iAQcore: chip busy");
            // prevIaqMillis = millis() - intervalIaq + 500;  // retry in 500msec
        }
        else {
            display.setCo2VOC(eco2, etvoc);
            sprintf(iaqMsg, "F:IAQ;C:%u;V:%u;R:%u;", eco2, etvoc, resist);
        };
    };
#endif // HAS_IAQ_CORE

/*----------------------------------------------------------------------------------------------*/
#if HAS_MHZ19
    void getMhz19Readings() 
    {
        if (co2.isPreHeating()) return;
        int ppm_uart = co2.readCO2UART();
        if (ppm_uart > 0) 
        {
            display.setCo2(ppm_uart);
            sprintf(mhz19Msg, "F:CO2;C:%u;", ppm_uart);
        };
    };
#endif
/*----------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------*/
// The setup() function runs once each time the micro-controller starts
void setup()
{
    bsec_version_t  version;        // get bsec version 
    bsec_get_version(&version);
    pinMode(LEDpin, OUTPUT);
    digitalWrite(LEDpin, HIGH);     //LED is low activ !

    #if DEBUG
        eeprom.SetDebugMode(true);
        #if HAS_RFM69
            rfm.SetDebugMode(true);    
        #endif
    #endif 

    Serial.begin(115200);
    sleep(6000);        //some time to open the terminalprogram ;o)

    Serial.println("");
    Serial.print("BME680 wireless sensor V");
    Serial.println(VERSION, 1);
    Serial.print("Compiled: ");
    Serial.println(__TIMESTAMP__);
    Serial.println("Protocol: UniversalSensor");
#ifdef ESP8266    
    Serial.printf("BSEC version: %d.%d.%d.%d\n", version.major, version.minor, version.major_bugfix, version.minor_bugfix);
    Serial.print("ESP Core Version: ");
    Serial.println(ESP.getCoreVersion());
    Serial.print("ESP SDK Version: ");
    Serial.println(ESP.getSdkVersion());
#elif defined(__STM32F1__)
    Serial.print("BSEC Version: ");
    Serial.print(version.major);
    Serial.print(".");
    Serial.print(version.minor);
    Serial.print(".");
    Serial.print(version.major_bugfix);
    Serial.print(".");
    Serial.println(version.minor_bugfix);
#endif
    Serial.println();

    //--- init standard I2C 
    //Wire.begin();

    //--- enable I2C for ESP8266 NodeMCU boards [VDD to 3V3, GND to GND, SDA to D2, SCL to D1]
    Wire.begin(SDA, SCL);
    Wire.setClockStretchLimit(1000); // Default is 230us, see line78 of https://github.com/esp8266/Arduino/blob/master/cores/esp8266/core_esp8266_si2c.c

    //--- nicht für STM32
#ifdef ESP8266
    Wire.setClock(400000);
#endif 

#if HAS_OLED
    /* init OLED */
    if (display.begin(SSD1306_SWITCHCAPVCC));
    {
        OLED = true;
        display.setTextSize(2);
        display.setTextColor(WHITE, BLACK);
        display.setCursor(5, 1);
        display.ssd1306_command(SSD1306_SETCONTRAST);
        display.ssd1306_command(0x80);
        display.clearDisplay();

        /* startscreen */
        display.println("  BME680");
        display.display();
        display.setTextSize(1);
        display.println();
        display.print("Wireless Sensor V");
        display.println(VERSION, 1);
        display.println("UniversalSensor Prot.");
        display.print("BSEC Ver.: ");
        display.print(version.major);
        display.print(".");
        display.print(version.minor);
        display.print(".");
        display.print(version.major_bugfix);
        display.print(".");
        display.println(version.minor_bugfix);
        display.display();
        sleep(5000);
    }
#endif

#ifdef ESP8266
    EEPROM.begin(6); //we need only 6 bytes
#endif

    /*** restore defaults ***/
    eeprom.RestoreDefaults();

    /*** read settings from EEPROM ***/
    NODEID     = eeprom.ReadNodeID();
    ALTITUDE   = eeprom.ReadAltitude();
    TEMPOFFSET = eeprom.ReadTempOffset();

    /*** if no or not all values stored in EEPROM, edit settings ***/
    if (NODEID == 0xFF || ALTITUDE == 65560.5 || (TEMPOFFSET > -1.2 && TEMPOFFSET < -1.0))
    {
        #if HAS_OLED
            display.setCursor(0, 0);
            display.clearDisplay();
            display.println("   *** SETUP ***");
            display.println();
            display.display();
        #endif

        /*** edit nodeID ***/
        if (NODEID == 0xFF)
            NODEID = eeprom.EditNodeID(NODEID);

        /*** edit altitude ***/
        if (ALTITUDE == 65560.5)
            ALTITUDE = eeprom.EditAltitude(ALTITUDE);

        /*** edit temperature offset ***/
        if (TEMPOFFSET > -1.2 && TEMPOFFSET < -1.0)
            TEMPOFFSET = eeprom.EditTempOffset(TEMPOFFSET);

        Serial.println("Save settings ...");
        eeprom.SaveSettings(NODEID, ALTITUDE, TEMPOFFSET);
    }
    else
    {
        Serial.println("Settings from EEPROM:");

    #if HAS_OLED
        display.setCursor(0, 0);
        display.clearDisplay();
        display.println("   *** KONFIG ***   ");
        display.println();
        display.display();
    #endif
    }

    Serial.print("Node-ID           : ");
    Serial.print(NODEID, DEC);
    Serial.print(" (0x");
    if (NODEID < 16)
        Serial.print("0");
    Serial.print(NODEID, HEX);
    Serial.println(")");

    Serial.print("Altitude          : ");
    Serial.print(ALTITUDE, 1);
    Serial.println("m");

    Serial.print("Temperature offset: ");
    Serial.print(TEMPOFFSET, 1);
    Serial.println(" degrees celsius");
    Serial.println();

    #if HAS_OLED
        if (OLED)
        {
            display.print("Node-ID  : ");
            display.print(NODEID, DEC);
            display.print(" (0x");
            if (NODEID < 16)
                display.print("0");
            display.print(NODEID, HEX);
            display.println(")");
            display.print("Hoehe    : ");
            display.print(ALTITUDE, 1);
            display.println("m");
            display.print("TempOffs.: ");
            display.print(TEMPOFFSET, 1);
            display.write(247);
            display.println("C");
            display.display();
            sleep(5000);
            display.setCursor(0, 0);
            display.clearDisplay();
        }
    #endif

    sleep(5000); //short time to read settings

    #if DEBUG
            Serial.println("activated components:");
        #if HAS_RFM69
            Serial.println("RFM69 - 433/868/915MHz transmitter");
        #endif
        #if HAS_LIGHTSENSOR
            Serial.println("BH1750 - lightsensor");
        #endif
        #if HAS_OLED
            Serial.println("OLED - 128x64 OLED display (SH1106 or SSD1306)");
        #endif
            Serial.print("measuring of battery voltage ");
        #if VCC_MEASURE
            Serial.println("activ");
        #else
            Serial.println("deactivated");
        #endif
    #endif

    /* --- RFM69CW init --- */
    #if HAS_RFM69
        if (rfm.Begin())
        {
            RFM69 = true; //RFM69 is present
            loop_counter = loop_count_lim; //to send data at startup
            #if HAS_OLED
                display.println("RFM69 gefunden.");
                display.display();
            #endif

            Serial.print("RFM69  init ... ");

            rfm.InitializeLaCrosse();
    
            #if DEBUG
                Serial.println();
                Serial.println("Init LaCrosse done");
            #endif

            rfm.SetFrequency(INITIAL_FREQ);
            float init_freq = float(INITIAL_FREQ);

            #if HAS_OLED
                display.print("Frequenz: ");
                display.print(init_freq / 1000, 3);
                display.println(" MHz");
            #endif

            #if DEBUG
                Serial.print("Set frequency to ");
                Serial.print(init_freq / 1000, 3);
                Serial.println(" MHz");
            #endif

            rfm.SetDataRate(DATA_RATE);

            #if HAS_OLED
                display.print("Baudrate: ");
                display.print(DATA_RATE);
                display.println(" Baud");
            #endif
            #if DEBUG
                Serial.print("Set datarate to ");
                Serial.print(DATA_RATE);
                Serial.println(" bps");
            #endif

            rfm.PowerDown(); // sleep to save power
            Serial.println("done");
            #if HAS_OLED
                display.println("RFM69 bereit.");
                display.display();
            #endif
        }
        else
        {
            #if HAS_OLED
                display.println("kein RFM69 gefunden!");
                display.display();
            #endif
            Serial.println("RFM69 not found, no wireless transmission !");
        }
    #endif

    /* --- BME680 init --- */
    Serial.print("BME680 init ... ");

    #if HAS_OLED
        display.print("BME680 init...");
        display.display();
    #endif


    //--- Wifi-stuff
    //--- fetches ssid and pass from eeprom and tries to connect
    //--- opens an AP (ESP+ChipID), enable WiFi setup at 192.168.4.1
    //--- reboots if credentials are set
    WiFiManager wifiManager;
    //--- uncomment and run it once, if you want to erase all the stored information
    //--- wifiManager.resetSettings();
    wifiManager.setTimeout(1); // TODO !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    wifiManager.autoConnect();

    // restart watchdog as a safety measure. 
    // its unknown how much time we spend in connect - but the wdt triggers after 3.5 sec
    ESP.wdtFeed();
    ESP.wdtEnable(0);

    Udp.begin(portMulti);
    DEBUG_PRINT("connected. device ip: " + WiFi.localIP());

    sprintf(serviceMsg, "F:SERVICE;START:%s;", ESP.getResetReason().c_str());
    serviceMessageTimer.attach(60, serviceAlive);

    #if HAS_IAQ_CORE
        iaqcore.begin();
        prevIaqMillis = millis() - intervalIaq + 500;  // first Iaq reading 500 msec after startup
    #endif 

    #if HAS_DHT22
        dht.begin();
        prevDhtMillis = millis() - intervalDht + 1000;  // first Dht reading 1 sec after startup
    #endif 

    #if HAS_LDR
        prevLdrMillis = millis() - intervalLdr + 200;  // first LDR reading 200 msec after startup
    #endif 

    /* Call to the function which initializes the BSEC library
    * Switch on low-power mode and provide no temperature offset
    * for UltraLowPower Mode change following line to:
    * ret = bsec_iot_init(BSEC_SAMPLE_RATE_ULP, ... */
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    return_values_init ret = bsec_iot_init(BSEC_SAMPLE_RATE_LP, TEMPOFFSET, bus_write, bus_read, sleep, state_load, config_load);
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    if (ret.bme680_status)
    {
        /*--- could not intialize BME680 */
        Serial.println("Error while initializing, BME680 connected ?");
        #if HAS_OLED               
            display.println();
            display.println("BME680 init. Fehler !");
            display.println("nicht angeschlossen ?");
            display.display();               
        #endif
        return; //--- jump to main loop
    }
    else if (ret.bsec_status)
    {
        /* Could not intialize BSEC library */
        Serial.println("Error while initializing BSEC library !");
        #if HAS_OLED            
            display.println();
            display.println("BSEC init. Fehler !");
            display.display();            
        #endif
        return; //jump to main loop
    }

    Serial.println("done");
    #if HAS_OLED        
        display.println("bereit");
        display.display();        
    #endif

    /* --- BH1750 init --- */
    #if HAS_LIGHTSENSOR
        Serial.print("BH1750 init ... ");
        #if HAS_OLED           
            display.print("BH1750 init...");
            display.display();           
        #endif
        /* for normal sensor resolution (1 lx resolution, 0-65535 lx, 120ms, no PowerDown) use: bh1750.begin(RESOLUTION_NORMAL, false); */
        if (bh1750.begin())
        {
            BH1750 = true; //sensor found on 0x23 or 0x5C
            Serial.println("done");
            #if HAS_OLED                
                display.println("bereit");
                display.display();                
            #endif
        }
        else
        {
            #if HAS_OLED                
                display.println("fehlt.");
                display.display();                
            #endif
            Serial.println("not present");
            BH1750 = false;
        }
    #endif

    blink(LEDpin, 3, 250); //--- setup success

    Serial.println("Ready, start measuring ...");
    Serial.println("");

    #if HAS_OLED       
        display.print("Starte Messungen ...");
        display.display();
        sleep(5000); //time to read display messages
        display.clearDisplay();
        display.display();       
    #endif

    /* main loop in output_ready, not in loop*/
    /* Call to endless loop function which reads and processes data based on sensor settings */
    /* State is saved every 10.000 samples, which means every 10.000 * 3 secs = 500 minutes  */    
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    bsec_iot_loop(sleep, get_timestamp_us, output_ready, state_save, 10000);
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

}
/*----------------------------------------------------------------------------------------------*/
// Add the main program code into the continuous loop() function
void loop()
{
    //--- if BME680 has an error during setup, signal through blinking LED
    blink(LEDpin, 3, 250); //BME680 setup not successful
    sleep(1000);
}
/*----------------------------------------------------------------------------------------------*/
void prepareMessage(char* payload) {
    sprintf(message, "T:IAQC;FW:1.0;ID:%06X;IP:%s;R:%ld;%s", ESP.getChipId(), WiFi.localIP().toString().c_str(), WiFi.RSSI(), payload);
    Serial.println(message);
};
/*----------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------*/
/* <eot> */
/*----------------------------------------------------------------------------------------------*/