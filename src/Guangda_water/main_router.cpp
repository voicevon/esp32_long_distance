/*
Rui Santos
Complete project details at https://RandomNerdTutorials.com/esp-now-esp8266-nodemcu-arduino-ide/

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files.

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
*/
#include "all_applications.h"
#ifdef I_AM_LONG_DISTANCE_ROUTER

#include <WiFi.h>
#include <esp_now.h>
#include "message.h"
#include "esp_wifi.h"

// #define PIN_YUNXING_4 4
// #define PIN_TINGZHI_4 5
// #define PIN_GUZHANG_4 16
// #define PIN_YUNXING_2 14
// #define PIN_TINGZHI_2 12
// #define PIN_GUZHANG_2 13

#define PIN_LED 2

// Create a struct_message called myData
// struct_message myData;
struct_message_source rx_buffer;
struct_message_source tx_buffer;
bool led_is_on = true;
uint8_t target_mac_addr[] = {0xCC, 0x50, 0xE3, 0x9A, 0x89, 0xC8};

esp_now_peer_info_t peerInfo;
// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    Serial.print("\r\nLast Packet Send Status:\t");
    if (status == ESP_NOW_SEND_SUCCESS)
    {
        Serial.println("Delivery Success");
        led_is_on = !led_is_on;
        digitalWrite(PIN_LED, led_is_on);
    }
    else
    {
        Serial.println("Delivery Fail");
    };
}

// Callback function that will be executed when data is received
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len)
{
    memcpy(&rx_buffer, incomingData, sizeof(rx_buffer));
    memcpy(&tx_buffer, incomingData, sizeof(tx_buffer));
    // Not sure send can be inside this callback of rx. ??
    esp_now_send(target_mac_addr, (uint8_t *) &tx_buffer, sizeof(tx_buffer));
    

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

    led_is_on = !led_is_on;
    digitalWrite(PIN_LED, led_is_on);
}


void setup_esp_now(){
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    ESP_ERROR_CHECK(esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR));

    // Init ESP-NOW
    if (esp_now_init() != 0){
        Serial.println("Error initializing ESP-NOW");
        return;
    }
}

void setup_tx(){
    esp_now_register_send_cb(OnDataSent);
    // Register peer
    memcpy(peerInfo.peer_addr, target_mac_addr, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    // Add peer
    if (esp_now_add_peer(&peerInfo) != ESP_OK)
    {
        Serial.println("Failed to add peer");
        return;
    }
}

void setup_rx(){
    esp_now_register_recv_cb(OnDataRecv);
}

void setup(){
    // Initialize Serial Monitor
    Serial.begin(115200);
    Serial.println("I_AM_LONG_DISTANCE_SLAVE");
    pinMode(PIN_LED, OUTPUT);
    // Set device as a Wi-Fi Station

    setup_esp_now();
    setup_rx();
    setup_tx();
    // Once ESPNow is successfully Init, we will register for recv CB to
    // get recv packer info
    // esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);
    Serial.print("Hello, I am Espnow  router");
}

void loop()
{
}

#endif