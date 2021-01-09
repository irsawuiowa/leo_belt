/*********
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp-now-one-to-many-esp32-esp8266/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*********/

#include <esp_now.h>
#include <WiFi.h>

// REPLACE WITH YOUR ESP RECEIVER'S MAC ADDRESS
uint8_t broadcastAddress1[] = {0x4C, 0x11, 0xAE, 0x77, 0x9D, 0x20};
uint8_t broadcastAddress2[] = {0x4C, 0x11, 0xAE, 0x77, 0xA2, 0x74};
uint8_t broadcastAddress3[] = {0x24, 0x6F, 0x28, 0xA6, 0x8B, 0x94};
uint8_t broadcastAddress4[] = {0x4C, 0x11, 0xAE, 0x76, 0x8B, 0xAC};
uint8_t broadcastAddress5[] = {0x24, 0x6F, 0x28, 0x1E, 0x01, 0xC4};
uint8_t broadcastAddress6[] = {0x24, 0x6F, 0x28, 0xA4, 0x49, 0xB8};
uint8_t broadcastAddress7[] = {0x24, 0x6F, 0x28, 0x5F, 0x12, 0x14};
uint8_t broadcastAddress8[] = {0x24, 0x6F, 0x28, 0xA2, 0xDE, 0x64};
esp_err_t result;

typedef struct test_struct {
  char intensity;
  int y;
} test_struct;

test_struct test;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  char macStr[18];
  //Serial.print("Packet to: ");
  // Copies the sender mac address to a string
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  //Serial.print(macStr);
  //Serial.print(" send status:\t");
  //Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// Send message just to feather 1
void sendFeather1(){
    esp_err_t result = esp_now_send(broadcastAddress1, (uint8_t *) &test, sizeof(test_struct));
    Serial.write("Sent feather 1. Intensity: ");
    Serial.write(test.intensity);
}

// Send message just to feather 2
void sendFeather2(){
    esp_err_t result = esp_now_send(broadcastAddress2, (uint8_t *) &test, sizeof(test_struct));
    Serial.write("Sent feather 2. Intensity: ");
    Serial.write(test.intensity);
}

// Send message just to feather 3
void sendFeather3(){
    esp_err_t result = esp_now_send(broadcastAddress3, (uint8_t *) &test, sizeof(test_struct));
    Serial.write("Sent feather 3. Intensity: ");
    Serial.write(test.intensity);
}

// Send message just to feather 4
void sendFeather4(){
    esp_err_t result = esp_now_send(broadcastAddress4, (uint8_t *) &test, sizeof(test_struct));
    Serial.write("Sent feather 4. Intensity: ");
    Serial.write(test.intensity);
}

// Send message just to feather 5
void sendFeather5(){
    esp_err_t result = esp_now_send(broadcastAddress5, (uint8_t *) &test, sizeof(test_struct));
    Serial.write("Sent feather 5. Intensity: ");
    Serial.write(test.intensity);
}

// Send message just to feather 6
void sendFeather6(){
    esp_err_t result = esp_now_send(broadcastAddress6, (uint8_t *) &test, sizeof(test_struct));
    Serial.write("Sent feather 6. Intensity: ");
    Serial.write(test.intensity);
}

// Send message just to feather 7
void sendFeather7(){
    esp_err_t result = esp_now_send(broadcastAddress7, (uint8_t *) &test, sizeof(test_struct));
    Serial.write("Sent feather 7. Intensity: ");
    Serial.write(test.intensity);
}

// Send message just to feather 8
void sendFeather8(){
    esp_err_t result = esp_now_send(broadcastAddress8, (uint8_t *) &test, sizeof(test_struct));
    Serial.write("Sent feather 8. Intensity: ");
    Serial.write(test.intensity);
}
 
void setup() {
  Serial.begin(57600);
 
  WiFi.mode(WIFI_STA);
 
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  esp_now_register_send_cb(OnDataSent);
   
  // register peer
  esp_now_peer_info_t peerInfo;
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // register first peer  
  memcpy(peerInfo.peer_addr, broadcastAddress1, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  
  // register second peer  
  memcpy(peerInfo.peer_addr, broadcastAddress2, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  
  // register third peer  
  memcpy(peerInfo.peer_addr, broadcastAddress3, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  // register fourth peer  
  memcpy(peerInfo.peer_addr, broadcastAddress4, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  // register fifth peer  
  memcpy(peerInfo.peer_addr, broadcastAddress5, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  // register sixth peer  
  memcpy(peerInfo.peer_addr, broadcastAddress6, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  // register seventh peer  
  memcpy(peerInfo.peer_addr, broadcastAddress7, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  // register eigth peer  
  memcpy(peerInfo.peer_addr, broadcastAddress8, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
}
 
void loop() {
  test.y = random(0,20);

  delay(2000);
  while (Serial.available() > 0) { // if any data available
    char incomingByte = Serial.read(); // read byte0
    if (incomingByte == 49) { // Feather 1
      sendFeather1();
    } else if (incomingByte == 50) { // Feather 2
      sendFeather2();
    } else if (incomingByte == 51) { // Feather 3
      sendFeather3();
    } else if (incomingByte == 52) { // Feather 4
      sendFeather4();
    } else if (incomingByte == 53) { // Feather 5
      sendFeather5();
    } else if (incomingByte == 54) { // Feather 6
      sendFeather6();
    } else if (incomingByte == 55) { // Feather 7
      sendFeather7();
    } else if (incomingByte == 56) { // Feather 8
      sendFeather8();
    } else { // Intensity of vibration -- 6 levels of intensity
        if (incomingByte == 65) { // A 
          test.intensity = 42;
        } else if (incomingByte == 66) { // B
          test.intensity = 84;
        } else if (incomingByte == 67) { // C
          test.intensity = 126;
        } else if (incomingByte == 68) { // D
          test.intensity = 168;
        } else if (incomingByte == 69) { // E
          test.intensity = 210;
        } else if (incomingByte == 70) { // F
          test.intensity = 255;
        } else if (incomingByte == 48) { // 0
          test.intensity = 0;
        }
    }
    //Serial.write(incomingByte); // send it back
  }
}