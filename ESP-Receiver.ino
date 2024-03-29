#include <ESP8266WiFi.h>
#include <espnow.h>
#include "ComStruct.h"
#include <SoftwareSerial.h>

#define LED_BUILTIN 1
SoftwareSerial UART(12,13); //RX,TX

//Supply voltage to analog pin
ADC_MODE(ADC_VCC);

/**
* handler function for received data
*/
void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len) {
  digitalWrite(LED_BUILTIN, HIGH);
  comStruct rcvdData;
  memcpy(&rcvdData, incomingData, sizeof(rcvdData));
  
  //write data to UART 
  UART.write((byte*)&rcvdData, sizeof(rcvdData));

  //print data on Serial Console
  Serial.print("\n---------------------\nESPNow data: ");
  Serial.print("\nid: ");
  Serial.print(rcvdData.id);
  Serial.print("\nkey: ");
  Serial.print(rcvdData.key);
  Serial.print("\nsensorType: ");
  Serial.print(rcvdData.sensorType);
  Serial.print("\ndValue: ");
  Serial.print(rcvdData.dValue);
  Serial.print("\nsValue: ");
  Serial.print(rcvdData.sValue);
  Serial.print("\ntype: ");
  Serial.print(rcvdData.type);
  
  digitalWrite(LED_BUILTIN, LOW);
}

void setup() {
   pinMode(LED_BUILTIN, OUTPUT);
   digitalWrite(LED_BUILTIN, LOW);
  
  Serial.begin(115200);
  UART.begin(115200);

  WiFi.mode(WIFI_STA);
  if(esp_now_init() != 0){
    Serial.println("Error initializing ESP-NOW");
    exit(1);
  }
  esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);
  esp_now_register_recv_cb(OnDataRecv);

}

void loop() {
  // put your main code here, to run repeatedly:

}
