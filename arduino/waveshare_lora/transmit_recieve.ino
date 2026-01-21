#include <SPI.h>
#include <RadioLib.h>
#include <vector>

// SPI definition (Waveshare RP2040 LoRa)
#define LED_PIN      25
#define LORA_SCK     14
#define LORA_MISO    24
#define LORA_MOSI    15
#define LORA_SS      13
#define LORA_RST     23
#define LORA_DIO1    16
#define LORA_BUSY    18
#define LORA_ANT_SW  17

SX1262 radio = new Module(LORA_SS, LORA_DIO1, LORA_RST, LORA_BUSY, SPI1);

// CHANGE THIS for the second device (e.g., "DeviceB")
String deviceID = "DeviceA"; 

volatile bool receivedFlag = false;
String last_msg = "none";
std::vector<String> txQueue;

// ISR callback
#if defined(ESP8266) || defined(ESP32)
ICACHE_RAM_ATTR
#endif
void setFlag(void) {
  receivedFlag = true;
}

// 1. Read input from Serial (from IDE or Python)
void checkSerialInput() {
  if (Serial.available() > 0) {
    String incoming = Serial.readStringUntil('\n');
    incoming.trim();
    if (incoming.length() > 0) {
      txQueue.push_back(incoming);
      // Feedback to Serial so you know it was queued
      Serial.print("TX_QUEUED: ");
      Serial.println(incoming);
    }
  }
}

// 2. Transmit queue
void processNextMessage() {
  if (txQueue.empty()) return;

  String message = txQueue.front();
  digitalWrite(LED_PIN, HIGH);

  // Switch to standby
  radio.standby();

  // Transmit
  last_msg = message;
  int txState = radio.transmit(message.c_str());

  // BUG FIX: The "TxDone" event triggered the interrupt flag. 
  // We must clear it manually so we don't try to read our own output.
  receivedFlag = false;

  if (txState == RADIOLIB_ERR_NONE) {
    // Message sent successfully
    Serial.println("TX_DONE"); 
    txQueue.erase(txQueue.begin());
  } else {
    Serial.print("TX_ERR: ");
    Serial.println(txState);
  }

  // Go back to listening
  radio.startReceive();
  digitalWrite(LED_PIN, LOW);
}

// 3. Receive Messages
void handleReceivedPacket() {
  if (!receivedFlag) return;
  receivedFlag = false;

  String str;
  int state = radio.readData(str);

  radio.startReceive(); // Resume listening immediately

  if (state == RADIOLIB_ERR_NONE) {
    if (str != "" && str != last_msg) {
      // PRINT THE RECEIVED MESSAGE FOR PYTHON
      Serial.print("RX: ");
      Serial.println(str);
    }
  } 
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(9600);
  // Wait up to 3 seconds for Serial, but don't hang forever if no USB
  unsigned long t = millis();
  while (!Serial && millis() - t < 3000);

  SPI1.setRX(LORA_MISO);
  SPI1.setTX(LORA_MOSI);
  SPI1.setSCK(LORA_SCK);
  SPI1.begin(false);

  // Use SF7 for faster messaging
  radio.begin(868.0, 125.0, 7, 5, RADIOLIB_SX126X_SYNC_WORD_PRIVATE, 17, 14, 0);
  radio.setPacketReceivedAction(setFlag);
  radio.startReceive();
  
  Serial.println("READY");
}

void loop() {
  checkSerialInput();
  processNextMessage();
  handleReceivedPacket();
}