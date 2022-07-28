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
#include <WiFi.h>
#include <sys/time.h>
#include "weather.h"
#include "driver/adc.h"
#include "soc/rtc_cntl_reg.h"
#include "esp_task_wdt.h"

RTC_DATA_ATTR uint16_t rainCount = 0;
RTC_DATA_ATTR uint64_t targetWake = 0;
RTC_DATA_ATTR uint8_t espnow_channel;
RTC_DATA_ATTR uint8_t macAddr[6];
RTC_DATA_ATTR uint8_t wake_count=0;

esp_sleep_wakeup_cause_t wakeup_reason;
bool sendSuccess = false;
bool sendFailure = false;
uint8_t failureCount = 0;
sensor_data_t sensorData;
int rainState = 0;

void sendData() {

  collectData(&sensorData);

  Serial.printf("Wakeup Reason=%d\n", sensorData.wakeup_reason);
  Serial.printf("Temperature=%f *C\n",sensorData.temperature);
  Serial.printf("Pressure=%d Pa\n",sensorData.pressure);
  Serial.printf("Humidity=%f\n",sensorData.humidity);
  Serial.printf("Battery Volts=%f mV\n",sensorData.battery_millivolts);
  Serial.printf("Direction=%d\n",sensorData.direction);
  Serial.printf("Rain Count=%f\n", sensorData.rain);
  Serial.printf("Anemometer Count=%f\n", sensorData.wind_speed);
  send_msg(&sensorData);
}

uint64_t getRemainingTime() {
  struct timeval tv;
  gettimeofday(&tv,NULL);

  uint64_t tvticks=tv.tv_sec * 1E06 + tv.tv_usec;
  if(targetWake < tvticks ) {
    targetWake=tvticks + (SLEEP_SECS * 1E06);
    //if(wakeup_reason != ESP_SLEEP_WAKEUP_TIMER)
    //  sendData();
  }

  return targetWake-tvticks;
}
void configureSleepMode() {
  esp_sleep_enable_timer_wakeup(getRemainingTime());
  
  //Supress spurous interrupts that are occuring after restart
  if(wakeup_reason != ESP_SLEEP_WAKEUP_UNDEFINED)
    esp_sleep_enable_ext0_wakeup(RAIN_PIN, rainState);
}

void handle_wakeup(){
  wakeup_reason = esp_sleep_get_wakeup_cause();
  uint8_t state;
  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 :
      //Serial.println("Wakeup caused by external signal using RTC_IO"); 
      state=digitalRead(RAIN_PIN);
      if(state == 1) { 
        Serial.println("High");
        rainCount++;
        rainState=0;
      } else {
        Serial.println("Low");
        rainState=1;
      }
      //No data being send, so just go to sleep
      sendSuccess=true;
      break;
    case ESP_SLEEP_WAKEUP_EXT1 : 
      //Serial.println("Wakeup caused by external signal using RTC_CNTL"); 
      break;
    case ESP_SLEEP_WAKEUP_TIMER : 
      //Serial.println("Wakeup caused by timer"); 
      // Every half hour requery the SSID of the BaseStation to adjust if something has changed with the Base Station
      if(wake_count > 60) {
        wake_count = 0;
        espnowInit(true);
      } else {
        wake_count++;
        espnowInit(false);
      }

      sendData();

      break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : 
      //Serial.println("Wakeup caused by touchpad"); 
      break;
    case ESP_SLEEP_WAKEUP_ULP : 
      //Serial.println("Wakeup caused by ULP program"); 
      break;
    default : 
      //Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason);

      espnowInit(true);
      //No data being send, so just go to sleep
      sendData();
      break;
  }

}

void gotoSleep() {

  configureSleepMode();
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);

  adc_power_off();

  //Turn off GPIOs for power savings
  pinMode(ADC_ALERT_PIN,OUTPUT);
  pinMode(GPIO_NUM_21,OUTPUT);
  pinMode(GPIO_NUM_22,OUTPUT);
  pinMode(GPIO_NUM_15,OUTPUT);

  digitalWrite(DEBUG_PIN,HIGH);
  esp_deep_sleep_start();

}

void setup() {
  // Start watchdog timer
  esp_task_wdt_init(5, true);
  esp_task_wdt_add(NULL);

  pinMode(ADC_ALERT_PIN,INPUT_PULLUP);
  pinMode(POWER_PIN_GND,OUTPUT);
  pinMode(WIND_VANE_PIN_VCC, OUTPUT);
  pinMode(DEBUG_PIN,OUTPUT);
  pinMode(RAIN_PIN, INPUT);

  digitalWrite(DEBUG_PIN,LOW);

  Serial.begin(115200);

  digitalWrite(POWER_PIN_GND,LOW);

  handle_wakeup();

}

void loop() {
  if(sendSuccess) {
    // Make sure watchdog doesn't reset as we are going to sleep
    esp_task_wdt_reset();
    gotoSleep();
  }

  if(sendFailure) {
    // Keep watchdog alive as long as we are retrying sends
    // Let watchdog reset if neither sendFailure ot SendSuccess happen
    esp_task_wdt_reset();
    failureCount++;
    //If we get more than 10 failure, just pretend it worked, so the ESP goes back to sleep;
    if(failureCount> 10) {
      sendSuccess = true;
    } else {
      delay(100);
      send_msg(&sensorData);
    }
  }

}
