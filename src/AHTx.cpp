/***************************************************************************************************/
/*
   This is an Arduino library for Aosong ASAIR AHTx, AHT15 Digital Humidity & Temperature Sensor

   written by : enjoyneering79
   sourse code: https://github.com/enjoyneering/


   This chip uses I2C bus to communicate, specials pins are required to interface
   Board:                                    SDA                    SCL                    Level
   Uno, Mini, Pro, ATmega168, ATmega328..... A4                     A5                     5v
   Mega2560................................. 20                     21                     5v
   Due, SAM3X8E............................. 20                     21                     3.3v
   Leonardo, Micro, ATmega32U4.............. 2                      3                      5v
   Digistump, Trinket, ATtiny85............. 0/physical pin no.5    2/physical pin no.7    5v
   Blue Pill, STM32F103xxxx boards.......... PB7                    PB6                    3.3v/5v
   ESP8266 ESP-01........................... GPIO0/D5               GPIO2/D3               3.3v/5v
   NodeMCU 1.0, WeMos D1 Mini............... GPIO4/D2               GPIO5/D1               3.3v/5v
   ESP32.................................... GPIO21/D21             GPIO22/D22             3.3v

   Frameworks & Libraries:
   ATtiny  Core          - https://github.com/SpenceKonde/ATTinyCore
   ESP32   Core          - https://github.com/espressif/arduino-esp32
   ESP8266 Core          - https://github.com/esp8266/Arduino
   STM32   Core          - https://github.com/stm32duino/Arduino_Core_STM32
                         - https://github.com/rogerclarkmelbourne/Arduino_STM32

   GNU GPL license, all text above must be included in any redistribution,
   see link for details  - https://www.gnu.org/licenses/licenses.html
*/
/***************************************************************************************************/

#include "AHTx.h"
#include <nrf_delay.h>


/**************************************************************************/
/*
    Constructor
*/
/**************************************************************************/
AHTx::AHTx(uint8_t address, ASAIR_I2C_SENSOR sensorName)
{
    _address    = address;
    _sensorName = sensorName;
}

/**************************************************************************/
/*
    begin()

    Initialize I2C & configure the sensor, call this function before
    doing anything else

    NOTE:
    - Wire.endTransmission() returned value:
      - 0 success
      - 1 data too long to fit in transmit data16
      - 2 received NACK on transmit of address
      - 3 received NACK on transmit of data
      - 4 other error
*/
/**************************************************************************/
bool AHTx::begin(void)
{

    nrf_delay_ms(AHTX_POWER_ON_DELAY);    //wait for sensor to initialize

    setNormalMode();                //one measurement+sleep mode

    return enableFactoryCalCoeff(); //load factory calibration coeff
}


/**************************************************************************/
/*
    readRawData()

    Read raw measurment data from sensor over I2C
*/
/**************************************************************************/
uint8_t AHTx::readRawData()
{
    /* send measurment command */
    Wire.beginTransmission(_address);
    I2Cdev::writeBit(_address, AHTX_START_MEASURMENT_CMD);
#if (ARDUINO) >= 100
    Wire.write(AHTX_START_MEASURMENT_CMD);                                     //send measurment command
  Wire.write(AHTX_DATA_MEASURMENT_CMD);                                      //send measurment parameter
  Wire.write(AHTX_DATA_NOP);                                                 //send measurment parameter
#else
    Wire.send(AHTX_START_MEASURMENT_CMD);
    Wire.send(AHTX_DATA_MEASURMENT_CMD);
    Wire.send(AHTX_DATA_NOP);
#endif
    I2Cdev::write()
    APP_ERROR_CHECK(getCalibrationBit() != 0x01);
    if (getBusyBit(AHTX_USE_READ_DATA) != 0x00) nrf_delay_ms(AHTX_MEASURMENT_DELAY); //measurement delay

    /* read 6-bytes from sensor */
#if defined(_VARIANT_ARDUINO_STM32_)
    Wire.requestFrom(_address, 6);
#else
    Wire.requestFrom(_address, 6, true);                                        //true - send stop after transmission & release I2C bus
#endif
    if (Wire.available() != 6)
    {
        _rawDataBuffer[0] = AHTX_ERROR;                                          //for condition when AHTX_USE_READ_DATA is used
        return AHTX_ERROR;                                                       //check rxBuffer & error handler, collision on the i2c bus
    }

    /* read 6 bytes from "wire.h" rxBuffer */
#if (ARDUINO) >= 100
    for (uint8_t i = 0; i < 6 ; i++)
  {
    _rawDataBuffer[i] = Wire.read();
  }
#else
    for (uint8_t i = 0; i < 6 ; i++)
    {
        _rawDataBuffer[i] = Wire.receive();
    }
#endif

    return true;
}


/**************************************************************************/
/*
    readTemperature()

    Read temperature, °C 

    NOTE:
    - temperature range      -40°C..+80°C
    - temperature resolution 0.01°C
    - temperature accuracy   ±0.3°C
*/
/**************************************************************************/
float AHTx::readTemperature(bool readI2C)
{
    if (readI2C == AHTX_FORCE_READ_DATA)
    {
        if (readRawData() == AHTX_ERROR) return AHTX_ERROR;   //force to read data to _rawDataBuffer & error handler
    }

    if (_rawDataBuffer[0] == AHTX_ERROR) return AHTX_ERROR; //error handler, collision on I2C bus

    uint32_t temperature = ((uint32_t)(_rawDataBuffer[3] & 0x0F) << 16) | ((uint16_t)_rawDataBuffer[4] << 8) | _rawDataBuffer[5]; //20-bit raw temperature data

    return (float)temperature * 0.000191 - 50;
}


/**************************************************************************/
/*
    readHumidity()

    Read relative humidity, %

    NOTE:
    - prolonged exposure for 60 hours at humidity > 80% can lead to a
      temporary drift of the signal +3%. Sensor slowly returns to the
      calibrated state at normal operating conditions.
    - relative humidity range      0%..100%
    - relative humidity resolution 0.024%
    - relative humidity accuracy   ±2%
*/
/**************************************************************************/
float AHTx::readHumidity(bool readI2C)
{
    if (readI2C == AHTX_FORCE_READ_DATA)
    {
        if (readRawData() == AHTX_ERROR) return AHTX_ERROR;   //force to read data to _rawDataBuffer & error handler
    }

    if (_rawDataBuffer[0] == AHTX_ERROR) return AHTX_ERROR; //error handler, collision on I2C bus

    uint32_t rawData = (((uint32_t)_rawDataBuffer[1] << 16) | ((uint16_t)_rawDataBuffer[2] << 8) | (_rawDataBuffer[3])) >> 4; //20-bit raw humidity data

    float humidity = (float)rawData * 0.000095;

    if (humidity < 0)   return 0;
    if (humidity > 100) return 100;
    return humidity;
}


/**************************************************************************/
/*
    softReset()  
 
    Restart sensor, without power off

    NOTE:
    - takes ~20ms
    - all registers restores to default
*/
/**************************************************************************/
bool AHTx::softReset(void)
{
    Wire.beginTransmission(_address);

#if (ARDUINO) >= 100
    Wire.write(AHTX_SOFT_RESET_CMD);
#else
    Wire.send(AHTX_SOFT_RESET_CMD);
#endif

    if (Wire.endTransmission(true) != 0) return false; //safety check, make sure sensor reset

    delay(AHTX_SOFT_RESET_DELAY);

    setNormalMode();                                   //reinitialize sensor registers after reset

    return enableFactoryCalCoeff();                    //reinitialize sensor registers after reset
}


/**************************************************************************/
/*
    setNormalMode()  
 
    Set normal measurment mode

    NOTE:
    - one measurement & power down??? no info in datasheet!!!
*/
/**************************************************************************/
bool AHTx::setNormalMode(void)
{
    Wire.beginTransmission(_address);

#if (ARDUINO) >= 100
    Wire.write(AHTX_NORMAL_CMD);
  Wire.write(AHTX_DATA_NOP);
  Wire.write(AHTX_DATA_NOP);
#else
    Wire.send(AHTX_NORMAL_CMD);
    Wire.send(AHTX_DATA_NOP);
    Wire.send(AHTX_DATA_NOP);
#endif

    if (Wire.endTransmission(true) != 0) return false; //safety check, make sure transmission complete

    delay(AHTX_CMD_DELAY);

    return true;
}


/**************************************************************************/
/*
    setCycleMode()  
 
    Set cycle measurment mode

    NOTE:
    - continuous measurement
*/
/**************************************************************************/
bool AHTx::setCycleMode(void)
{
    Wire.beginTransmission(_address);

#if (ARDUINO) >= 100
    if   (_sensorName != AHT20_SENSOR) Wire.write(AHTX_INIT_CMD); //set command mode
  else                               Wire.write(AHT20_INIT_CMD); 
  Wire.write(AHTX_INIT_CYCLE_MODE | AHTX_INIT_CAL_ENABLE);     //0,[0,1],0,[1],0,0,0
  Wire.write(AHTX_DATA_NOP);
#else
    if   (_sensorName != AHT20_SENSOR) Wire.send(AHTX_INIT_CMD);
    else                               Wire.send(AHT20_INIT_CMD);
    Wire.send(AHTX_INIT_CYCLE_MODE | AHTX_INIT_CAL_ENABLE);
    Wire.send(AHTX_DATA_NOP);
#endif

    if (Wire.endTransmission(true) != 0) return false;             //safety check, make sure transmission complete
    return true;
}


/**************************************************************************/
/*
    readStatusByte()

    Read status byte from sensor over I2C
*/
/**************************************************************************/
uint8_t AHTx::readStatusByte()
{
#if defined(_VARIANT_ARDUINO_STM32_)
    Wire.requestFrom(_address, 1);
#else
    Wire.requestFrom(_address, 1, true);           //true - send stop after transmission & release I2C bus
#endif
    if (Wire.available() != 1) return AHTX_ERROR; //check rxBuffer & error handler, collision on I2C bus

    /* read byte from "wire.h" rxBuffer */
#if (ARDUINO) >= 100
    return Wire.read();
#else
    return Wire.receive();
#endif
}


/**************************************************************************/
/*
    getCalibrationBit()

    Read Calibration bit from status byte

    NOTE:
    - 0, factory calibration coeff disabled
    - 1, factory calibration coeff loaded
*/
/**************************************************************************/
uint8_t AHTx::getCalibrationBit(bool readI2C)
{
    if (readI2C == AHTX_FORCE_READ_DATA) _rawDataBuffer[0] = readStatusByte(); //force to read status byte

    if (_rawDataBuffer[0] != AHTX_ERROR) return bitRead(_rawDataBuffer[0], 3); //get 3-rd bit
    return AHTX_ERROR;
}


/**************************************************************************/
/*
    enableFactoryCalCoeff()
 
    Load factory calibration coefficients
*/
/**************************************************************************/
bool AHTx::enableFactoryCalCoeff()
{
    /* load factory calibration coeff */
    Wire.beginTransmission(_address);

#if (ARDUINO) >= 100
    if   (_sensorName != AHT20_SENSOR) Wire.write(AHTX_INIT_CMD); //set command mode
  else                               Wire.write(AHT20_INIT_CMD);
  Wire.write(AHTX_INIT_CAL_ENABLE);                             //0,0,0,0,[1],0,0,0
  Wire.write(AHTX_DATA_NOP);                                    //0,0,0,0,0,0,0,0
#else
    if   (_sensorName != AHT20_SENSOR) Wire.send(AHTX_INIT_CMD);
    else                               Wire.send(AHT20_INIT_CMD);
    Wire.send(AHTX_INIT_CAL_ENABLE);
    Wire.send(AHTX_DATA_NOP);
#endif

    if (Wire.endTransmission(true) != 0) return false;             //safety check, make sure transmission complete

    delay(AHTX_CMD_DELAY);

    /*check calibration enable */
    if (getCalibrationBit() == 0x01) return true;
    return false;
}


/**************************************************************************/
/*
    getBusyBit()

    Read busy bit from status byte

    NOTE:
    - 0, sensor idle & sleeping
    - 1, sensor busy & in measurement state
*/
/**************************************************************************/
uint8_t AHTx::getBusyBit(bool readI2C)
{
    if (readI2C == AHTX_FORCE_READ_DATA) _rawDataBuffer[0] = readStatusByte(); //force to read status byte

    if (_rawDataBuffer[0] != AHTX_ERROR) return bitRead(_rawDataBuffer[0], 7); //get 7-th bit
    return AHTX_ERROR;
}
