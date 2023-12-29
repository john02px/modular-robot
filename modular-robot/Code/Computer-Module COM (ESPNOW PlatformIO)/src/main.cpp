#include <esp_now.h>
#include <WiFi.h>
#include <Arduino.h>

float Parameter[4];
String packet = "";

// robots id and their corresponding mac address
uint8_t broadcastAddress1[] = {0x94, 0xB5, 0x55, 0xC8, 0xCC, 0x8C}; // robot 1
uint8_t broadcastAddress2[] = {0x94, 0xB5, 0x55, 0xC8, 0xCC, 0x48}; // robot 2
uint8_t broadcastAddress3[] = {0x94, 0xB5, 0x55, 0xC8, 0xD5, 0x14}; // robot 3

typedef struct test_struct {
  float distance;
  float angle;
  float state;
} test_struct;


  test_struct robotdata;


void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  char macStr[18];
  Serial.print("Packet to: ");
  // Copies the sender mac address to a string
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print(macStr);
  Serial.print(" send status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// Decodes the incoming transmission in format: "(tag-id),(distance to target,(angle to target),(robot state)/" //
// Sample transmission: "3,200.89,56.98,1/" //

void Decode() {
  String Temp = "";
  if (packet.length() > 0) {  /* incoming packet from serial port containing id, distance, and angle data */
    int j = 0;
    for (int i = 0; i < packet.length() ; i++) {
      if (packet[i] == ',') {    // "," as seperator between values
        Parameter[j] = Temp.toFloat();   // converts string to float
        Temp = "";                 // clear temp to for next value
        j++;
      }
      else if (packet[i] == '/') {    // end of packet indicator  
        Parameter[j] = Temp.toFloat();
        Temp = "";
        j = 0;
      }
      else {
        Temp += packet[i];
      }
    }
  }
}

void setup() {

  Serial.begin(115200);

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

  memcpy(peerInfo.peer_addr, broadcastAddress1, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  memcpy(peerInfo.peer_addr, broadcastAddress2, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
  memcpy(peerInfo.peer_addr, broadcastAddress3, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  

}

void loop() {

  if (Serial.available()) {
    delay(2);

    while (Serial.available()) {
      packet += char(Serial.read());

    }
    Decode();

    robotdata.distance = Parameter[1];
    robotdata.angle = Parameter[2];
    robotdata.state = Parameter[3];

  if (Parameter[0] == 1.0) {
    esp_err_t result1 = esp_now_send(
                          broadcastAddress1,
                          (uint8_t *) &robotdata,
                          sizeof(test_struct));
  }

  else if (Parameter[0] == 2.0) {
    esp_err_t result2 = esp_now_send(
                          broadcastAddress2,
                          (uint8_t *) &robotdata,
                          sizeof(test_struct));


  }

  else if (Parameter[0] == 3.0) {
    esp_err_t result3 = esp_now_send(
                          broadcastAddress3,
                          (uint8_t *) &robotdata,
                          sizeof(test_struct));
  }

  }


}