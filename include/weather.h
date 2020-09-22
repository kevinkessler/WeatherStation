/**
 *  @filename   :   weather.h
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

#ifndef INCLUDE_WEATHER_H_
#define INCLUDE_WEATHER_H_

#define SLEEP_SECS 30
#define ADC_ALERT_PIN GPIO_NUM_27
#define WIND_VANE_PIN_VCC GPIO_NUM_26
#define POWER_PIN_GND GPIO_NUM_25
#define ANEMOMETER_PIN GPIO_NUM_4
#define RAIN_PIN GPIO_NUM_33
#define DEBUG_PIN GPIO_NUM_14
#define DEFAULT_WIFI_CHANNEL (3)
#define STATION_NAME "WeatherBase"

typedef struct __attribute__((packed)) sensor_data_t {
    float temperature;
    int32_t pressure;
    float humidity;
    float battery_millivolts;
    uint8_t direction;
    uint16_t anemometer_count;
    uint16_t anemometer_gust;
    uint16_t rain_count;
} sensor_data_t;

void collectData(sensor_data_t *);
void espnowInit(bool);
void send_msg(sensor_data_t * msg);

#endif /* INCLUDE_WEATHER_H_ */