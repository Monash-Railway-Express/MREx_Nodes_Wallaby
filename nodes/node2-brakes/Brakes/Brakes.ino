#include <CAN_MREx.h>
#include <Adafruit_NeoPixel.h>

#include "../../../shared/MREx.h"
uint8_t NODE_ID = BRAKES_ID;

#define TX_GPIO_NUM GPIO_NUM_8 //18
#define RX_GPIO_NUM GPIO_NUM_7 //19
#define SERVICE_BRAKE_PIN GPIO_NUM_9  // 22 HIGH energises relay -> 24V to coil -> Release brakes
//LED pins

#define  LED_LIGHT GPIO_NUM_38
#define LED_COUNT 1
Adafruit_NeoPixel pixel(LED_COUNT, LED_LIGHT, NEO_GRB + NEO_KHZ800);

uint8_t lastOut = 255 ;// remembers what the brake output was last time
// manually set it high at the start because when the code runs it will se a change and print a message

void setup() {
  Serial.begin(115200);
  delay(1000); // wait to help connection
  Serial.println("Brakes node started (Node 2)");

  initCANMREX(TX_GPIO_NUM, RX_GPIO_NUM, NODE_ID);

  pinMode(SERVICE_BRAKE_PIN, OUTPUT);
  // pinMode(LED_LIGHT, OUTPUT);
  //onboard LED setup
  pixel.begin();
  pixel.clear();
  pixel.show();
  

  // Fail-safe startup: APPLY brakes in the beginning (no power to coil)
  digitalWrite(SERVICE_BRAKE_PIN, LOW);

  // IMPORTANT: Do NOT force operational mode here. ONLY for testing
  // nodeOperatingMode should be controlled by NMT messages on the CAN network.
  // nodeOperatingMode = 0x01; 
  
// register od entries
  registerODEntry(OD_SERVICE_BRAKE_MC_INDEX,  OD_SERVICE_BRAKE_MC_SUBINDEX, 2, sizeof(od_service_brake_mc), &od_service_brake_mc); // brake flag motor controls
  registerODEntry(OD_SERVICE_BRAKE_DC_INDEX,  OD_SERVICE_BRAKE_DC_SUBINDEX, 2, sizeof(od_service_brake_dc), &od_service_brake_dc); // brake flag driver controls
  registerODEntry(OD_DIRECTION_MODE_INDEX,    OD_DIRECTION_MODE_SUBINDEX,   2, sizeof(od_direction_mode),   &od_direction_mode); // direction mode

  xTaskCreatePinnedToCore(
      CAN_Task,
      "CAN Task",
      6144,
      &NODE_ID,
      3,
      NULL,
      0
  );
}
void loop() {
  uint8_t out = LOW; // variable for brake pin output

  // Priority order:
  // 1. STOPPED            => APPLY
  // 2. PRE‑OP + neutral   => RELEASE (push mode)
  // 3. PRE‑OP + not neutral => APPLY
  // 4. OPERATIONAL + service brake => APPLY
  // 5. OPERATIONAL + no service brake => RELEASE
  // 6. Unknown => APPLY
  
  // STOPPED
  if (nodeOperatingMode == MODE_STOPPED) {
    out = LOW;
    pixel.setPixelColor(0, pixel.Color(50, 0, 0)); // Red
  }

  // PRE‑OPERATIONAL
  else if (nodeOperatingMode == MODE_PREOP) {
    // Neutral => allow pushing
    // 0 od entry means brakes on
    if (od_service_brake_dc == 0) {
      out = LOW;
      pixel.setPixelColor(0, pixel.Color(50, 50, 0)); // Yellow
    }
    else {
      out = HIGH;
      pixel.setPixelColor(0, pixel.Color(50, 0, 0));  // Red
    }
  }

  // OPERATIONAL
  else if (nodeOperatingMode == MODE_OPERATIONAL) {

    // Service brake requested
    if ((od_service_brake_dc == 0) || (od_service_brake_mc == 0)) {
      out = LOW;
      pixel.setPixelColor(0, pixel.Color(50, 0, 0));  // Red
    }
    else {
      out = HIGH;
      pixel.setPixelColor(0, pixel.Color(0, 50, 0));  // Green
    }
  }
  // Unknown => fail‑safe
    else {
      out = LOW;
      pixel.setPixelColor(0, pixel.Color(50, 0, 0));    // Red
    }
  pixel.show();

  // Only update + print when it changes, to make seeing changes more clear

  if (out != lastOut) {
    lastOut = out;
    digitalWrite(SERVICE_BRAKE_PIN, out);

    Serial.print("Mode = 0x");
    Serial.print(nodeOperatingMode, HEX);
    Serial.print(" | Service brake flag = ");
    Serial.print(od_service_brake_dc);
    Serial.print(" | Friction brake: ");
    Serial.println(out == HIGH ? "RELEASED (relay ON)" : "APPLIED (relay OFF)");
  }

}



  // old code based on operating mode only

  // // STOPPED (0x02) or PRE-OP (0x80) => APPLY friction brakes => relay OFF
  // if (nodeOperatingMode == 0x02 || nodeOperatingMode == 0x80) {
  //   out = LOW;   // relay de-energised -> no 24V -> brake APPLIED
  //   // digitalWrite(LED_LIGHT,LOW);
  //   pixel.setPixelColor(0, pixel.Color(50, 0, 0)); // red = applied
  //   pixel.show();

  // }
  // // OPERATIONAL (0x01) => RELEASE friction brakes => relay ON
  // else if (nodeOperatingMode == 0x01) {
  //   out = HIGH;  // relay energised -> 24V -> brake RELEASED
  //   // digitalWrite(LED_LIGHT,HIGH);
  //   pixel.setPixelColor(0, pixel.Color(0, 50, 0)); // green = released
  //   pixel.show();

  // }
  // // Unknown state => fail-safe APPLY
  // else {
  //   out = LOW;
  //   // digitalWrite(LED_LIGHT,HIGH);
  //   pixel.setPixelColor(0, pixel.Color(50, 0, 0)); // red = applied
  //   pixel.show();

  // }
