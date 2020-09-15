/**
 *  @filename   :   main.cpp
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

#include <Arduino.h>
#include <sys/time.h>
#include "weather.h"

RTC_DATA_ATTR uint16_t windCount = 0;
RTC_DATA_ATTR uint16_t rainCount = 0;
RTC_DATA_ATTR esp_sleep_ext1_wakeup_mode_t rainState = ESP_EXT1_WAKEUP_ALL_LOW;
RTC_DATA_ATTR uint64_t targetWake = 0;

esp_sleep_wakeup_cause_t wakeup_reason;
int windState = 0;


void sendData() {
  sensor_data_t sensorData;
  
  collectData(&sensorData);

  Serial.printf("Temperature=%f *C\n",sensorData.temperature);
  Serial.printf("Pressure=%d Pa\n",sensorData.pressure);
  Serial.printf("Humidity=%f\n",sensorData.humidity);
  Serial.printf("Battery Volts=%f mV\n",sensorData.battery_millivolts);
  Serial.printf("Direction=%d\n",sensorData.direction);
  Serial.printf("Rain Count=%d\n", sensorData.rain_count);
  Serial.printf("Anenomoeter Count=%d\n", sensorData.anemometer_count);
  Serial.printf("Anemometer Gust=%d\n",sensorData.anemometer_gust);

}

uint64_t getRemainingTime() {
  struct timeval tv;
  gettimeofday(&tv,NULL);

  uint64_t tvticks=tv.tv_sec * 1E06 + tv.tv_usec;
  if(targetWake < tvticks ) {
    targetWake=tvticks + (SLEEP_SECS * 1E06);
    if(wakeup_reason != ESP_SLEEP_WAKEUP_TIMER)
      sendData();
  }

  return targetWake-tvticks;
}
void configureSleepMode() {
  esp_sleep_enable_timer_wakeup(getRemainingTime());
  pinMode(RAIN_PIN, INPUT);
  pinMode(ANEMOMETER_PIN, INPUT);
  uint64_t mask = 0x200000000;
  esp_sleep_enable_ext1_wakeup(mask,rainState);
  esp_sleep_enable_ext0_wakeup(ANEMOMETER_PIN, windState);
}

void handle_wakeup(){
  wakeup_reason = esp_sleep_get_wakeup_cause();
  uint8_t state;
  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 :
      //Serial.println("Wakeup caused by external signal using RTC_IO"); 
      state=digitalRead(ANEMOMETER_PIN);
      if(state == 1) { 
        Serial.println("High");
        windCount++;
        windState=0;
      } else {
        Serial.println("Low");
        windState=1;
      }
      break;
    case ESP_SLEEP_WAKEUP_EXT1 : 
      //Serial.println("Wakeup caused by external signal using RTC_CNTL"); 
      if(rainState == ESP_EXT1_WAKEUP_ANY_HIGH) { 
        Serial.println("High");
        rainCount++;
        rainState=ESP_EXT1_WAKEUP_ALL_LOW;
      } else {
        Serial.println("Low");
        rainState=ESP_EXT1_WAKEUP_ANY_HIGH;
      }
      break;
    case ESP_SLEEP_WAKEUP_TIMER : 
      Serial.println("Wakeup caused by timer"); 
      sendData();

      break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
  }

  configureSleepMode();
}

void setup() {
  pinMode(ADC_ALERT_PIN,INPUT_PULLUP);
  pinMode(POWER_PIN_GND,OUTPUT);
  pinMode(WIND_VANE_PIN_VCC, OUTPUT);
  pinMode(DEBUG_PIN,OUTPUT);

  digitalWrite(DEBUG_PIN,LOW);
  Serial.begin(115200);

  digitalWrite(POWER_PIN_GND,LOW);

  handle_wakeup();

  configureSleepMode();

  //Turn off GPIOs for power savings
  pinMode(ADC_ALERT_PIN,OUTPUT);
  pinMode(GPIO_NUM_21,OUTPUT);
  pinMode(GPIO_NUM_22,OUTPUT);

  digitalWrite(DEBUG_PIN,HIGH);
  esp_deep_sleep_start();

}

void loop() {

}