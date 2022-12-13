/*
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp-now-esp8266-nodemcu-arduino-ide/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*/
#include "all_applications.h"
#ifdef I_AM_LONG_DISTANCE_MASTER
#include <esp_now.h>
#include <WiFi.h>
#include "message.h"
#include <esp_wifi.h>
// #include "esp_netif.h"
// #include "esp_err.h"
// #include "esp_wifi_types.h"
// #include "esp_event.h"
// #include "esp_private/esp_wifi_private.h"
// #include "esp_wifi_default.h"

#define PIN_YUNXING 14  //D5
#define PIN_TINGZHI 12  //D6
#define PIN_GUZHANG 13  //D7
#define PIN_LED 2


#define MY_ID 7

// REPLACE WITH RECEIVER MAC Address  40:22:D8:F0:24:20
uint8_t rx_mac_addr[] = {0x40, 0x22, 0xD8, 0xF0, 0x24, 0x20};



// Create a struct_message called myData
struct_message myData;

unsigned long lastTime = 0;  
unsigned long timerDelay = 500;  // send readings timer
bool led_is_on = false;

esp_now_peer_info_t peerInfo;
// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  if (status == ESP_NOW_SEND_SUCCESS) { 
    Serial.println("Delivery Success");
    led_is_on = ! led_is_on;
    digitalWrite(PIN_LED, led_is_on);
   }else{
    Serial.println("Delivery Fail");
    };
}


/* WiFi should start before using ESPNOW */
static void wifi_init(void)
{
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));
  ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_start());

  // ESP_ERROR_CHECK(esp_wifi_set_protocol(ESP_IF_WIFI_STA, WIFI_PROTOCOL_LR));
  esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR);
}
 
void setup() {
  // Init Serial Monitor
  Serial.begin(115200);
 
  pinMode(PIN_YUNXING, INPUT_PULLUP);
  pinMode(PIN_TINGZHI, INPUT_PULLUP);
  pinMode(PIN_GUZHANG, INPUT_PULLUP);
  pinMode(PIN_LED, OUTPUT);
  
  // Set device as a Wi-Fi Station
  // WiFi.mode(WIFI_STA);
  // int a = esp_wifi_set_protocol( WIFI_IF_STA, WIFI_PROTOCOL_LR );

  // Init ESP-NOW
  wifi_init();
  ESP_ERROR_CHECK(esp_now_init());
  // if (esp_now_init() != 0) {
  //   Serial.println("Error initializing ESP-NOW");
  //   return;
  // }
  esp_now_register_send_cb(OnDataSent);

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  // esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
  
  // // Register peer
  // esp_now_add_peer(rx_mac_addr, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);

    // Register peer
  memcpy(peerInfo.peer_addr, rx_mac_addr, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;

    // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
}
 
void loop() {
  if ((millis() - lastTime) > timerDelay) {
    myData.station_id =  MY_ID;
    myData.io_1 = digitalRead(PIN_YUNXING);
    myData.io_2 = digitalRead(PIN_TINGZHI);
    myData.io_3 = digitalRead(PIN_GUZHANG);

    // Send message via ESP-NOW
    esp_now_send(rx_mac_addr, (uint8_t *) &myData, sizeof(myData));
    Serial.print(millis());
    Serial.print("\t\t");
    Serial.print(myData.io_1);
    Serial.print(myData.io_2);
    Serial.println(myData.io_3);

    lastTime = millis();
  }
}

#endif