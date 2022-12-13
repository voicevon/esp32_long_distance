// Complete Instructions to Get and Change ESP MAC Address: https://RandomNerdTutorials.com/get-change-esp32-esp8266-mac-address-arduino/
#include "all_applications.h"
#ifdef I_AM_LONG_DISTANCE_MAC_CHECKER

#include "WiFi.h"
 
void setup__macaddress(){
  Serial.begin(115200);
  Serial.println("Hello world, I_AM_LONG_DISTANCE_MAC_CHECKER ");
  WiFi.mode(WIFI_MODE_STA);
  Serial.println(WiFi.macAddress());
}

void setup(){
  setup__macaddress();
} 

void loop(){

}


#endif
