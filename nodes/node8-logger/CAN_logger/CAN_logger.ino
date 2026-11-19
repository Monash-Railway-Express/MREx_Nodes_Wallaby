/**
 * CAN Logger file 
 *
 * File:            CAN_logger.ino
 * Organisation:    MREX
 * Author:          Chiara Gillam, Nhan Nguyen
 * Date Created:    12/10/2025
 * Last Modified:   05/05/2026
 * Version:         2.1.1
 *
 */



// Libraries used:
// https://wiki.dfrobot.com/DS3231M%20MEMS%20Precise%20RTC%20SKU:%20DFR0641




#include "../../../shared/MREx.h"
const uint8_t NODE_ID = LOGGER_ID;  // Change this to set your device's node ID
#include <SPI.h>
#include <Wire.h>
#include "driver/twai.h"
#include <ArduinoJson.h> // ArduinoJson by Benoit Blanchon
#include <CAN_MREx.h>
#include "../../../shared/DualSerial/DualSerial.cpp"
#include "wshtml.h"

// --- Pin Definitions ---
#define TX_GPIO_NUM GPIO_NUM_14
#define RX_GPIO_NUM GPIO_NUM_13

const char* URL = "10.0.0.8";
const char* FILEPATH = "/files";
AsyncEventSource events("/events");
AsyncEventSource emcy("/emcy");

// Ring buffer config
const int BUFFER_SIZE = 32;
struct CANFrame {
  String timestamp;
  uint32_t id;
  uint8_t dlc;
  uint8_t data[8];
};
CANFrame buffer[BUFFER_SIZE];
int bufferIndex = 0;
unsigned long lastFlush = 0;
unsigned long lastUpdate = 0;

struct Parameter {
  String key;
  bool sign;
  uint8_t targetNode;
  uint16_t index;
  uint8_t subindex;
  size_t size;
};
struct Parameter parameters[] = {
  {"kProportional1", 0, 14, 0x60F6, 0x00, sizeof(uint8_t)},
  {"kIntegral1", 0, 14, 0x60F6, 0x01, sizeof(uint8_t)},
  {"kDerivative1", 0, 14, 0x60F6, 0x02, sizeof(uint8_t)},
  {"kProportional2", 0, 14, 0x60F6, 0x03, sizeof(uint8_t)},
  {"kIntegral2", 0, 14, 0x60F6, 0x04, sizeof(uint8_t)},
  {"kDerivative2", 0, 14, 0x60F6, 0x05, sizeof(uint8_t)},
  {"kProportional3", 0, 14, 0x60F6, 0x06, sizeof(uint8_t)},
  {"kIntegral3", 0, 14, 0x60F6, 0x07, sizeof(uint8_t)},
  {"kDerivative3", 0, 14, 0x60F6, 0x08, sizeof(uint8_t)},
  {"kProportional4", 0, 14, 0x60F6, 0x09, sizeof(uint8_t)},
  {"kIntegral4", 0, 14, 0x60F6, 0x0A, sizeof(uint8_t)},
  {"kDerivative4", 0, 14, 0x60F6, 0x0B, sizeof(uint8_t)},
  {"kProportional5", 0, 14, 0x60F6, 0x0C, sizeof(uint8_t)},
  {"kIntegral5", 0, 14, 0x60F6, 0x0D, sizeof(uint8_t)},
  {"kDerivative5", 0, 14, 0x60F6, 0x0E, sizeof(uint8_t)},
};
const int num_parameters = 15;

int getParameterIdx(String key) {
  for (int i = 0; i < num_parameters; i++) {
    if (parameters[i].key == key) {
      return i;
    }
  }
  return -1;
}

void setup() {
  DualSerial.begin(115200);
  Wire.begin();

  //Initialize CANMREX protocol
  initCANMREX(TX_GPIO_NUM, RX_GPIO_NUM, NODE_ID); // this or manual, not both
  
  DualSerial.println("CAN logging started");

  server.on("/feed", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/html", wshtml);
  });

  server.addHandler(&events);
  server.addHandler(&emcy);
  
  DualSerial.println("Webserver started.");
  DualSerial.print("SSID: ");
  DualSerial.println(DualSerial.SSID);
  DualSerial.print("Password: ");
  DualSerial.println(DualSerial.PASSPHRASE);
  DualSerial.print("SSE: http://");
  DualSerial.print(URL);
  DualSerial.println("/events");
}

void loop() {
  twai_message_t message;
  if (twai_receive(&message, pdMS_TO_TICKS(10)) == ESP_OK) {
    Serial.println("Hello");

    twai_message_t rxMsg = message;
    uint32_t canID = rxMsg.identifier;
    const uint8_t nodeID = NODE_ID;
    if (canID >= 0x081 && canID <= 0x0FF) { // Emergency messages (always processed)
      handleEMCY(rxMsg, nodeID);
    }

    if (checkMajorEMCY()) {
      uint8_t *node;
      uint32_t *code;
      getMajorByIndex(0, node, code);
      emcy.send(("Major" + String(*node) + " " + String(*code, HEX)).c_str());
    }

    if (checkMinorEMCY()) {
      uint8_t *node;
      uint32_t *code;
      getMinorByIndex(0, node, code);
      emcy.send(("Minor" + String(*node) + " " + String(*code, HEX)).c_str());
    }

    CANFrame frame;
    // // Fallback timestamp using millis
    frame.timestamp = String(millis());

    frame.id = message.identifier;
    frame.dlc = message.data_length_code;
    for (int i = 0; i < 8; i++) {
      frame.data[i] = (i < message.data_length_code) ? message.data[i] : 0;
    }

    buffer[bufferIndex++] = frame;

    // Flush if buffer full
    if (bufferIndex >= BUFFER_SIZE) {
      flushBuffer();
    }
  }

  // Periodic flush
  if (millis() - lastFlush > 500 && bufferIndex > 0) {
    flushBuffer();
  }
}

void flushBuffer() {
  for (int i = 0; i < bufferIndex; i++) {
    char id[20];
    sprintf(id, "%lX", buffer[i].id);

    String row;
    row += buffer[i].timestamp;
    row += ",0x";
    row += id;
    row += ",";
    row += buffer[i].dlc;
    for (int j = 0; j < 8; j++) {
      row += ",";
      if (j < buffer[i].dlc) {
        row += "0x";
        char str[5];
        sprintf(str, "%02lX", buffer[i].data[j]);
        row += str;
      }
    }
    events.send(row.c_str());
  }
  bufferIndex = 0;
  lastFlush = millis();
}