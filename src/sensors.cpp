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
uint8_t s35770Addr=0x32;

extern uint16_t windCount;
extern uint16_t rainCount;
extern esp_sleep_wakeup_cause_t wakeup_reason;

void pollAlertReadyPin() {
  for (uint32_t i = 0; i<250000; i++)
    if (!digitalRead(ADC_ALERT_PIN)) return;
   Serial.println("Failed to wait for AlertReadyPin, it's stuck high!");
}

uint16_t convertMVtoDir(float mv) {
  //ADC Error
  if (mv == 0.0) return 0xFFFF;

  // Have to fudge these numbers a bit to account for the fact that the higher the resistance, the
  // Measured voltage is higher than expected. I don't know why
  if (mv < 241.2) return 113;
  if (mv < 285.0) return 68;
  if (mv < 382.2) return 90; 
  if (mv < 536.2) return 158;
  if (mv < 733.0) return 135;
  if (mv < 906.9) return 203;
  if (mv < 1175.3) return 180;
  if (mv < 1471.1) return 23;
  if (mv < 1798.3) return 45;
  if (mv < 2083.0) return 248;
  if (mv < 2262.1) return 225;
  if (mv < 2521.7) return 338;
  if (mv < 2731.2) return 0;
  if (mv < 2900.1) return 293;
  if (mv < 3096.9) return 315;
  if (mv < 3322.5) return 270;
  
  // Some Error Happened
  return 0xFFFF;
}

void resetS35770(){
    Wire.beginTransmission(s35770Addr);
    Wire.write(0b10000001);
    Wire.write(0x00);
    Wire.write(0x00);
    Wire.write(0b00000010);
    Wire.endTransmission();
}

uint32_t readS35770Count() {
    Wire.requestFrom(s35770Addr, (uint8_t)3);
    uint8_t b1 = Wire.read();
    uint8_t b2 = Wire.read();
    uint8_t b3 = Wire.read();

    uint32_t retval=(b1<<16)+(b2 << 8) + b3;

    return retval;
}
void collectData(sensor_data_t *data) {
  // Required for ADS1115 which does not initialize i2c  
  Wire.begin();

  // Turn on Power divider
  digitalWrite(POWER_PIN_GND,HIGH);

  // Power on wind vane
  digitalWrite(WIND_VANE_PIN_VCC,1);

  adc.initialize();
  adc.setMode(ADS1115_MODE_SINGLESHOT);
  adc.setRate(ADS1115_RATE_32);
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

  adc.setMultiplexer(ADS1115_MUX_P1_NG);
  adc.triggerConversion();

  // Wait if the conversion is still not finished
  pollAlertReadyPin();
  data->direction = convertMVtoDir(adc.getMilliVolts(false));

  data->rain=rainCount * 0.011;
  rainCount=0;
  data->wind_speed=(1.492 * readS35770Count()) / (float)SLEEP_SECS;
  resetS35770();
  
  data->wakeup_reason = wakeup_reason;

  //Power off vane
  digitalWrite(WIND_VANE_PIN_VCC,0);
}
