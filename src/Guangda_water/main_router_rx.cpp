/*
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp-now-esp8266-nodemcu-arduino-ide/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*/
#include "all_applications.h"
#ifdef I_AM_LONG_DISTANCE_ROUTER_RX

#include <WiFi.h>
#include <esp_now.h>
#include "message.h"
#include "esp_wifi.h"


#define PIN_YUNXING_4 4
#define PIN_TINGZHI_4 5
#define PIN_GUZHANG_4 16
#define PIN_YUNXING_2 14
#define PIN_TINGZHI_2 12
#define PIN_GUZHANG_2 13

#define PIN_LED 2


// Create a struct_message called myData
// struct_message myData;
struct_message_source rx_buffer;
bool led_is_on = true;

// Callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&rx_buffer, incomingData, sizeof(rx_buffer));
  Serial.print("Bytes received: ");
  Serial.print(len);
  Serial.print("\t");
  Serial.print(rx_buffer.station_id);
  Serial.print("\t");
  Serial.print(rx_buffer.io_1);
  Serial.print(" ");
  Serial.print(rx_buffer.io_2);
  Serial.print(" ");
  Serial.println(rx_buffer.io_3);

  led_is_on = ! led_is_on;
  digitalWrite(PIN_LED, led_is_on);

  switch (rx_buffer.station_id)
  {
    case 4:
        digitalWrite(PIN_YUNXING_4, rx_buffer.io_1);
        digitalWrite(PIN_TINGZHI_4, rx_buffer.io_2);
        digitalWrite(PIN_GUZHANG_4, rx_buffer.io_3);
        break;
    case 2:
        digitalWrite(PIN_YUNXING_2, rx_buffer.io_1);
        digitalWrite(PIN_TINGZHI_2, rx_buffer.io_2);
        digitalWrite(PIN_GUZHANG_2, rx_buffer.io_3);
        break;
    
    default:
        break;
  }
}
 


void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);
  Serial.println("I_AM_LONG_DISTANCE_SLAVE");
  pinMode(PIN_YUNXING_2, OUTPUT);
  pinMode(PIN_TINGZHI_2, OUTPUT);
  pinMode(PIN_GUZHANG_2, OUTPUT);
  pinMode(PIN_YUNXING_4, OUTPUT);
  pinMode(PIN_TINGZHI_4, OUTPUT);
  pinMode(PIN_GUZHANG_4, OUTPUT);
  pinMode(PIN_LED, OUTPUT);
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  ESP_ERROR_CHECK(esp_wifi_set_protocol( WIFI_IF_STA, WIFI_PROTOCOL_LR ));

  // Init ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  // esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);
  esp_now_register_recv_cb(OnDataRecv);
  Serial.print("Hello, I am Espnow  receiver");
  
}

void loop() {
  
}


#endif