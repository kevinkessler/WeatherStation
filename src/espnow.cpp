/**
 *  @filename   :   espnow.cpp
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
#include <esp_now.h>
#include <esp_wifi.h>
#include "weather.h"

extern uint8_t espnow_channel;
extern uint8_t macAddr[6];
extern bool sendSuccess;
extern bool sendFailure;

static uint8_t broadcast_mac[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

static void handle_error(esp_err_t err)
{
  switch (err)
  {
    case ESP_ERR_ESPNOW_NOT_INIT:
      Serial.println("Not init");
      break;

    case ESP_ERR_ESPNOW_ARG:
      Serial.println("Argument invalid");
      break;

    case ESP_ERR_ESPNOW_INTERNAL:
      Serial.println("Internal error");
      break;

    case ESP_ERR_ESPNOW_NO_MEM:
      Serial.println("Out of memory");
      break;

    case ESP_ERR_ESPNOW_NOT_FOUND:
      Serial.println("Peer is not found");
      break;

    case ESP_ERR_ESPNOW_IF:
      Serial.println("Current WiFi interface doesn't match that of peer");
      break;

    default:
      Serial.print("Unknown Error ");Serial.println(err);
      Serial.println(esp_err_to_name(err));
      break;
  }
}

void send_msg(sensor_data_t * msg)
{
  // Pack
  uint16_t packet_size = sizeof(sensor_data_t);
  uint8_t msg_data[packet_size];
  memcpy(&msg_data[0], msg, sizeof(sensor_data_t));

  esp_err_t status = esp_now_send(macAddr, msg_data, packet_size);
  if (ESP_OK != status)
  {
    Serial.println("Error sending message");
    sendFailure=true;
    handle_error(status);
  }
}

static void msg_send_cb(const uint8_t* mac, esp_now_send_status_t sendStatus)
{

  switch (sendStatus)
  {
    case ESP_NOW_SEND_SUCCESS:
      sendSuccess=true;
      break;

    case ESP_NOW_SEND_FAIL:
      Serial.println("Send Failure");
      sendFailure=true;
      break;

    default:
      break;
  }
}

void espnowInit(bool scan)
{
  //Puts ESP in STATION MODE
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if(scan) {
    espnow_channel = DEFAULT_WIFI_CHANNEL;
    memcpy(macAddr, broadcast_mac, 6);  

    uint8_t networks = WiFi.scanNetworks();
    for(int n=0;n<networks;n++) {
        Serial.printf("%d %s   %d   %d %s \n", n,WiFi.SSID(n).c_str(),WiFi.RSSI(n), WiFi.channel(n), WiFi.BSSIDstr(n).c_str());
        /*for(int i=0;i<8;i++) {
            Serial.print(WiFi.BSSID(n)[i]);
            Serial.print(" ");
        }
        Serial.println();*/

        if(WiFi.SSID(n).indexOf(STATION_NAME) == 0)
        {
            //Serial.println("Found");
            memcpy(macAddr, WiFi.BSSID(n), 6);
            espnow_channel = WiFi.channel(n);
        }
        
    }

    WiFi.scanDelete();
  }

  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(espnow_channel,WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  if (esp_now_init() != 0)
  {
    return;
  }

  esp_now_peer_info_t peer_info;
  peer_info.channel = espnow_channel;
  memcpy(peer_info.peer_addr, macAddr, 6);
  peer_info.encrypt = false;
  peer_info.ifidx = ESP_IF_WIFI_STA;
  esp_err_t status = esp_now_add_peer(&peer_info);
  if (ESP_OK != status)
  {
    Serial.println("Could not add peer");
    handle_error(status);
  }

  status = esp_now_register_send_cb(msg_send_cb);
  if (ESP_OK != status)
  {
    Serial.println("Could not register send callback");
    handle_error(status);
  }
}