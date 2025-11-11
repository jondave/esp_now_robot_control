/*
 * TWO-WAY ESP-NOW / SERIAL GATEWAY (Reliable Version - Fixed for ESP32 Core 3.x)
 * Upload this to BOTH boards.
 * Change the 'peerMac' variable for each board.
 */

#include <esp_now.h>
#include <WiFi.h>

// --------------------------------------------------------------------
// CONFIGURATION
// --------------------------------------------------------------------

// WHEN UPLOADING TO BOARD 1 (...49:80), use Board 2's MAC here:
// uint8_t peerMac[] = {0x68, 0x25, 0xDD, 0xEF, 0x49, 0x80};

// WHEN UPLOADING TO BOARD 2 (...49:80), use Board 1's MAC here:
uint8_t peerMac[] = {0x68, 0x25, 0xDD, 0xEE, 0x49, 0x80};

// --------------------------------------------------------------------
// MESSAGE STRUCTURE
// --------------------------------------------------------------------
typedef struct struct_message {
  char text[250];
} struct_message;

struct_message myData;
esp_now_peer_info_t peerInfo;

// --------------------------------------------------------------------
// CALLBACKS
// --------------------------------------------------------------------

// 🛰️ OnDataSent — works with both old/new ESP-IDF versions
#if ESP_IDF_VERSION_MAJOR >= 5
void OnDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
  if (status != ESP_NOW_SEND_SUCCESS) {
    Serial.printf("⚠️ Send failed (status=%d)\n", status);
  }
}
#else
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (status != ESP_NOW_SEND_SUCCESS) {
    Serial.printf("⚠️ Send failed (status=%d)\n", status);
  }
}
#endif

// 📩 OnDataRecv — safe copy with proper type casting
void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len) {
  size_t safeLen = (size_t)((len < (int)(sizeof(myData.text) - 1)) ? len : (int)(sizeof(myData.text) - 1));
  memcpy(myData.text, incomingData, safeLen);
  myData.text[safeLen] = '\0';

  Serial.println(myData.text);
}

// --------------------------------------------------------------------
// SETUP
// --------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  Serial.setRxBufferSize(256);
  delay(200);

  WiFi.mode(WIFI_STA);
  delay(500);

  Serial.print("📡 My MAC: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("❌ Error initializing ESP-NOW");
    return;
  }

  // Register callbacks
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  // Initialize peer info
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, peerMac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("❌ Failed to add peer");
    return;
  }

  Serial.println("✅ ESP-NOW initialized successfully");
}

// --------------------------------------------------------------------
// MAIN LOOP
// --------------------------------------------------------------------
void loop() {
  static char inputBuffer[256];
  static size_t index = 0;

  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      if (index > 0) {
        inputBuffer[index] = '\0';
        strncpy(myData.text, inputBuffer, sizeof(myData.text));
        myData.text[sizeof(myData.text) - 1] = '\0';

        esp_err_t result = esp_now_send(peerMac, (uint8_t *)&myData, strlen(myData.text) + 1);
        if (result != ESP_OK) {
          Serial.printf("⚠️ Send error: %d\n", result);
        }

        index = 0;
      }
    } else if (index < sizeof(inputBuffer) - 1) {
      inputBuffer[index++] = c;
    } else {
      index = 0;
      Serial.println("⚠️ Input overflow — message too long!");
    }
  }
}
