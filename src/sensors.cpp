/**
 *  @filename   :   sensors.cpp
 *  @brief      :   ESP32 Weather Station
 *
 *  @author     :   Kevin Kessler
 *
 * Copyright (C) 2020 Kevin Kessler
 *
 *    This program is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <ADS1115.h>
#include "weather.h"

Adafruit_BME280 bme;
ADS1115 adc(ADS1115_ADDRESS_ADDR_GND);

extern uint16_t windCount;
extern uint16_t rainCount;

void pollAlertReadyPin() {
  for (uint32_t i = 0; i<100000; i++)
    if (!digitalRead(ADC_ALERT_PIN)) return;
   Serial.println("Failed to wait for AlertReadyPin, it's stuck high!");
}

uint8_t convertMVtoDir(float mv) {
    return 1;
}

void collectData(sensor_data_t *data) {
  // Required for ADS1115 which does not initialize i2c  
  Wire.begin();

  // Turn on Power divider
  digitalWrite(POWER_PIN_GND,HIGH);

  adc.initialize();
  adc.setMode(ADS1115_MODE_SINGLESHOT);
  adc.setRate(ADS1115_RATE_128);
  adc.setGain(ADS1115_PGA_4P096);
  adc.setConversionReadyPinMode();

  adc.setMultiplexer(ADS1115_MUX_P0_NG);
  adc.triggerConversion();

  // Work on the BMP while the ADC conversion is happening
  uint8_t status;
  if (!(status=bme.begin(0x76,&Wire))) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    Serial.println(status);
    data->pressure = -1;
    data->temperature = 999.0;
    data->humidity=999.0;
  } else {
    bme.setSampling(Adafruit_BME280::MODE_FORCED, // Go to sleep after one reading
                    Adafruit_BME280::SAMPLING_X1, // temperature
                    Adafruit_BME280::SAMPLING_X1, // pressure
                    Adafruit_BME280::SAMPLING_X1, // humidity
                    Adafruit_BME280::FILTER_OFF );
    data->temperature = bme.readTemperature();
    data->pressure = bme.readPressure();  
    data->humidity = bme.readHumidity();
  }

  // Wait if the conversion is still not finished
  pollAlertReadyPin();
  data->battery_millivolts = adc.getMilliVolts(false) * 2.0;

  // Disable Power Divider
  digitalWrite(POWER_PIN_GND,LOW);

  // Power on wind vane
  digitalWrite(WIND_VANE_PIN_VCC,1);
  adc.setMultiplexer(ADS1115_MUX_P1_NG);
  adc.triggerConversion();

  // Wait if the conversion is still not finished
  pollAlertReadyPin();
  data->direction = convertMVtoDir(adc.getMilliVolts(false));

  data->rain_count=rainCount;
  rainCount=0;
  data->anemometer_count=windCount;
  windCount=0;
  data->anemometer_gust=3;
  

  //Power off vane
  digitalWrite(WIND_VANE_PIN_VCC,0);
}