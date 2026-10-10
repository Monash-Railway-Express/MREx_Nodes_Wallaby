#include "CAN_MREx.h"
#include "controller.h"
#include <stdlib.h>
#include <HardwareSerial.h>

/**
 * @file Controller.ino
 * @brief Driver controller code with full Nextion HMI integration.
 *        Reads all driver inputs, sends over CAN, receives telemetry
 *        from battery/motor/lights nodes and displays on Nextion 7" screen.
 *
 * @author Chiara Gillam
 * @author Audrey Tasevki
 * @author Kang Yee
 * @author Nicholas Rowe
 * @author Aditya Dinesh Kumar
 *
 * @date Created: 05/08/2025
 * @version 1.5.0
 * @organisation MREX
 *
 * @changes v1.5.0
 *   - Fixed speed scaling: od_true_speed is raw integer km/h (no /10 needed)
 *   - Fixed speed overflow: clamp od_true_speed to realistic range before display
 *   - Fixed corrupt duplicate OD registerODEntry line in setup()
 *   - Fixed duplicate nodeOperatingMode assignment in UpdateOpMode()
 *   - Fixed missing closing brace in HandleLocation()
 *   - Fixed HandleLocation() logic — now correctly receives counter from
 *     autostop node via OD entry (0x1051:00) and manual button increments
 *     and SDO-writes to both autostop and audio nodes
 *   - Added location announcement popup — shows announcement text for 5s
 *     when od_location_counter changes (auto or manual)
 *   - Added HandleEmcyClear() — button to acknowledge and dismiss EMCY display
 *   - Removed HandleCondition() — condition mode display intentionally dropped
 *   - Preserved safety check: cannot jump from STOPPED to OPERATIONAL directly
 *   - Preserved major EMCY check in loop blocking op mode changes
*/

// ═══════════════════════════════════════════════════════════════
// OD DEFINITIONS — DRIVER CONTROLS (LOCAL)
// ═══════════════════════════════════════════════════════════════

// OD 0x3012:00 – Regen Brake (0–1023). Mapped to TPDO1.
uint16_t od_regen_brake = 0;

// OD 0x606A:00 - Motor command / throttle demand (0–1023). Mapped to TPDO1.
uint16_t od_motor_command = 0;

// OD 0x6060:00 - Direction mode (1=back, 2=neutral, 3=forward).
uint8_t od_direction_mode = 3;

// OD 0x6062:00 - Challenge mode (1=throttle, 2=speed, 3=autostop, 4=regen, 5=traction).
uint8_t od_challenge_mode = 1;

// OD 0x6065:00 - Horn toggle (1=active, 0=default).
uint8_t od_horn_toggle = 0;

// OD 0x3012:02 - Service brake (0=not braking, 1=braking).
uint8_t od_service_brake_dc = BRAKE_APPLIED;

// Free
uint8_t od_switch_2 = 0;

// OD 0x1051:00 - Location counter. Written by autostop node (Node 6) via SDO.
// Increments as loco passes markers around the circuit.
uint8_t od_location_counter = 0;

// ═══════════════════════════════════════════════════════════════
// OD DEFINITIONS — RECEIVED FROM OTHER NODES VIA RPDO
// ═══════════════════════════════════════════════════════════════

// From Battery node (Node 7) via RPDO1 (COB-ID 0x187)
uint32_t od_current          = 0;  // 0x2000:00 — mA, divide by 1000 for A
uint16_t od_voltage          = 0;  // 0x2000:01 — mV, divide by 1000 for V
uint16_t od_soc              = 0;  // 0x2000:02 — x10, divide by 10 for %

// From Battery node (Node 7) via RPDO2 (COB-ID 0x287)
uint32_t od_power            = 0;  // 0x2000:03 — W direct
uint32_t od_recovered_energy = 0;  // 0x2000:04 — J direct

// From Motor node (Node 1) via RPDO1 (COB-ID 0x181)
uint32_t od_true_speed       = 0;  // 0x606C:00 — km/h x10 (divide by 10)
uint32_t od_tractive_effort  = 0;  // 0x606E:00 — > 0 means tractive effort applied

// From Lights/Sensors node (Node 4) via RPDO3 (COB-ID 0x184)
uint16_t od_temp_front       = 0;  // 0x1004:00 — °C direct
uint16_t od_temp_rear        = 0;  // 0x1004:01 — °C direct

// ═══════════════════════════════════════════════════════════════
// TIMING + ADC BUFFERS
// ═══════════════════════════════════════════════════════════════

unsigned long previousMillis = 0;
const long interval = 100;

ADCBuffer throttleBuf  = {0};
ADCBuffer brakeBuf     = {0};
ADCBuffer dirBuf       = {0};
ADCBuffer challengeBuf = {0};
ADCBuffer opModeBuf    = {0};

// ═══════════════════════════════════════════════════════════════
// NEXTION HMI
// ═══════════════════════════════════════════════════════════════

HardwareSerial nextionSerial(1); // UART1

// Previous value cache — only sends to Nextion when value changes
int      prevSpeed          = -1;
int      prevThrottle       = -1;
int      prevBrake          = -1;
int      prevDirection      = -1;
int      prevChallenge      = -1;
int      prevBrakeStatus    = -1;
bool     prevBrakeFault     = 0;
uint8_t  prevOpMode         = 255;
uint32_t prevVoltage        = 0;
uint32_t prevCurrent        = 0;
uint32_t prevPower          = 0;
uint16_t prevSoc            = 0;
uint32_t prevEnergy         = 0;
uint16_t prevTempFront      = 0;
int      prevTractiveEffort = -1;
bool     prevRegenOnly      = false;
bool     prevAutostop       = false;
bool     prevEmcy           = false;
uint8_t  prevLocCounter     = 0;

// EMCY fault tracking
bool brakeFault = false;
bool fireyFault = false;
unsigned long fireyStart = 0;
bool emcyActive = false;
String majorFaultText = "None";
String minorFaultText = "None";

// ── Nextion helper functions ────────────────────────────────────

void sendText(String component, String value) {
  nextionSerial.print(component + ".txt=\"" + value + "\"");
  nextionSerial.write(0xFF);
  nextionSerial.write(0xFF);
  nextionSerial.write(0xFF);
}

void sendProgressBar(String component, int value) {
  nextionSerial.print(component + ".val=" + String(value));
  nextionSerial.write(0xFF);
  nextionSerial.write(0xFF);
  nextionSerial.write(0xFF);
}

void sendColour(String component, String attr, int colour) {
  nextionSerial.print(component + "." + attr + "=" + String(colour));
  nextionSerial.write(0xFF);
  nextionSerial.write(0xFF);
  nextionSerial.write(0xFF);
}

void refreshComponent(String component) {
  nextionSerial.print("ref " + component);
  nextionSerial.write(0xFF);
  nextionSerial.write(0xFF);
  nextionSerial.write(0xFF);
}

void sendPage(int page) {
  nextionSerial.print("page " + String(page));
  nextionSerial.write(0xFF);
  nextionSerial.write(0xFF);
  nextionSerial.write(0xFF);
}

// ── Helper: set a pill active or inactive ──────────────────────
void setPill(String component, bool active, int activeColour) {
  sendColour(component, "bco", active ? activeColour : NEX_DARK);
  sendColour(component, "pco", active ? 0 : NEX_GREY);
  refreshComponent(component);
}

// ── Helper: Direction ───────────────────────────────────────────
String getDirectionText(int dir) {
  switch (dir) {
    case 1: return "REVERSE";
    case 2: return "NEUTRAL";
    case 3: return "FORWARD";
    default: return "--";
  }
}

int getDirectionColour(int dir) {
  switch (dir) {
    case 1: return NEX_RED;
    case 2: return NEX_YELLOW;
    case 3: return NEX_GREEN;
    default: return NEX_GREY;
  }
}

// ── Helper: Challenge ───────────────────────────────────────────
String getChallengeText(int mode) {
  switch (mode) {
    case 1: return "Throttle Ctrl";
    case 2: return "Speed Ctrl";
    case 3: return "Autostop";
    case 4: return "Regen";
    case 5: return "Traction";
    default: return "--";
  }
}

// ── Helper: Operation mode ──────────────────────────────────────
String getOpModeText(uint8_t mode) {
  switch (mode) {
    case MODE_STOPPED:     return "Stopped";
    case MODE_PREOP:       return "Pre-Op";
    case MODE_OPERATIONAL: return "Operational";
    default: return "--";
  }
}

int getOpModeColour(uint8_t mode) {
  switch (mode) {
    case MODE_STOPPED:     return NEX_RED;
    case MODE_PREOP:       return NEX_YELLOW;
    case MODE_OPERATIONAL: return NEX_CYAN;
    default: return NEX_GREY;
  }
}

// ── Helper: Location announcement text ─────────────────────────
String getLocationText(uint8_t counter) {
  switch (counter) {
    case LOC_ANN1_START:       return "MREx Wallaby is commencing its challenge run";
    case LOC_ANN2_AUTOSTOP:    return "MREx Wallaby is commencing the auto-stop challenge";
    case LOC_ANN3_COMFORT:     return "MREx Wallaby is about to commence the ride comfort challenge";
    case LOC_ANN4_COMFORT_END: return "MREx Wallaby has completed the ride comfort challenge";
    case LOC_ANN5_HAVEN:       return "MREx Wallaby is passing the Haven on its return leg";
    case LOC_ANN6_TCN:         return "MREx Wallaby is about to commence the traction challenge";
    case LOC_ANN7_TCN_END:     return "MREx Wallaby has completed the traction challenge";
    case LOC_ANN8_END:         return "MREx Wallaby has completed its challenge run";
    default: return "";
  }
}

/**
 * @brief Main Nextion update function — called every 100ms from loop()
 *        Only sends UART commands when values have changed.
 */
void updateNextion() {

  // ── SPEED — x10 scaled; readout uncapped, bar clamped to full scale
  float speedF = float(od_true_speed)/10;
  int speedInt = (int)(od_true_speed);  // for cache comparison
  if (speedInt != prevSpeed) {
    char buf[16];  // fits full uint32/10 range
    dtostrf(speedF, 4, 1, buf);
    sendText("t_speed", String(buf) + " km/h");
    sendProgressBar("j_speed", constrain((int)(speedF * 100.0f / SPEED_BAR_FULL_SCALE_KMH), 0, 100));
    prevSpeed = speedInt;
  }

  // ── THROTTLE ────────────────────────────────────────────────
  int throttlePct = map(od_motor_command, 0, 1023, 0, 100);
  // if (throttlePct != prevThrottle) {
  if (true) {
    sendText("t_throttle", String(throttlePct) + " %");
    sendProgressBar("j_throttle", throttlePct);
    prevThrottle = throttlePct;
  }

  // ── BRAKE ───────────────────────────────────────────────────
  int brakePct = map(od_regen_brake, 0, 1023, 0, 100);
  // if (brakePct != prevBrake) {
  if (true) {
    sendText("t_brakepct", String(brakePct) + " %");
    sendProgressBar("j_brake", brakePct);
    prevBrake = brakePct;
  }

  // ── BRAKE STATUS (Released / Applied / Fault) ───────────────
  int currentBrakeStatus = od_service_brake_dc;
  if (currentBrakeStatus != prevBrakeStatus || brakeFault != prevBrakeFault) {
    if (brakeFault) {
      sendText("t_brakestatus", "Fault");
      sendColour("t_brakestatus", "pco", NEX_RED);
    } else if (od_service_brake_dc == BRAKE_APPLIED) {

      sendText("t_brakestatus", "Applied");
      sendColour("t_brakestatus", "pco", NEX_RED);
    } else {
      sendText("t_brakestatus", "Released");
      sendColour("t_brakestatus", "pco", NEX_GREY);
    }
    refreshComponent("t_brakestatus");
    prevBrakeStatus = currentBrakeStatus;
    prevBrakeFault  = brakeFault;
  }

  // ── DIRECTION MODE ──────────────────────────────────────────
  if (od_direction_mode != prevDirection) {
    sendText("t_direction", getDirectionText(od_direction_mode));
    sendColour("t_direction", "pco", getDirectionColour(od_direction_mode));
    refreshComponent("t_direction");
    prevDirection = od_direction_mode;
  }

  // ── CHALLENGE MODE ──────────────────────────────────────────
  if (od_challenge_mode != prevChallenge) {
    sendText("t_challenge", getChallengeText(od_challenge_mode));
    prevChallenge = od_challenge_mode;
  }

  // ── AUTOSTOP STATUS ─────────────────────────────────────────
  // Only reads from Node 6 when autostop challenge is active
  uint32_t autostopDetected = 0;
  if (od_challenge_mode == 3) {
    autostopDetected = executeSDORead(NODE_ID, AUTOSTOP_ID, 0x1050, 0x00);
  }
  bool autostopActive = (autostopDetected == 1);
  if (autostopActive != prevAutostop) {
    sendText("t_autostop", autostopActive ? "ACTIVE" : "Standby");
    sendColour("t_autostop", "pco", autostopActive ? NEX_GREEN : NEX_GREY);
    refreshComponent("t_autostop");
    prevAutostop = autostopActive;
  }

  // ── OPERATION MODE ──────────────────────────────────────────
  if (nodeOperatingMode != prevOpMode) {
    sendText("t_opmode", getOpModeText(nodeOperatingMode));
    sendColour("t_opmode", "pco", getOpModeColour(nodeOperatingMode));
    refreshComponent("t_opmode");
    prevOpMode = nodeOperatingMode;
  }

  // ── BATTERY — VOLTAGE ───────────────────────────────────────
  if (od_voltage != prevVoltage) {
    float v = od_voltage / 1000.0;
    char buf[10];
    dtostrf(v, 4, 1, buf);
    sendText("t_voltage", String(buf) + " V");
    prevVoltage = od_voltage;
  }

  // ── BATTERY — CURRENT ───────────────────────────────────────
  if (od_current != prevCurrent) {
    float a = (int32_t)od_current / 1000.0;
    char buf[10];
    dtostrf(a, 4, 1, buf);
    sendText("t_current", String(buf) + " A");
    prevCurrent = od_current;
  }

  // ── BATTERY — POWER ─────────────────────────────────────────
  if (od_power != prevPower) {
    sendText("t_power", String((int32_t)od_power) + " W");
    prevPower = od_power;
  }

  // ── BATTERY — SOC ───────────────────────────────────────────
  if (od_soc != prevSoc) {
    float soc = od_soc / 10.0;
    char buf[8];
    dtostrf(soc, 4, 1, buf);
    sendText("t_soc",    String(buf) + " %");
    sendText("t_charge", String(buf) + " %");
    prevSoc = od_soc;
  }

  // ── BATTERY — RECOVERED ENERGY ──────────────────────────────
  if (od_recovered_energy != prevEnergy) {
    sendText("t_energy", String(od_recovered_energy) + " J");
    prevEnergy = od_recovered_energy;
  }

  // ── TEMPERATURE (front sensor) ──────────────────────────────
  if (od_temp_front != prevTempFront) {
    sendText("t_temp", String(od_temp_front) + " C");
    prevTempFront = od_temp_front;
  }

  // ── TRACTIVE EFFORT PILL ────────────────────────────────────
  bool tractiveActive = (od_tractive_effort > 0);
  if ((int)tractiveActive != prevTractiveEffort) {
    setPill("t_tractive", tractiveActive, NEX_GREEN);
    prevTractiveEffort = (int)tractiveActive;
  }

  // ── REGEN ONLY PILL ─────────────────────────────────────────
  // Active when recovered energy > 0 AND power is positive (drawing from recovered store)
  bool regenOnly = (od_recovered_energy > 0 && (int32_t)od_power > 0);
  if (regenOnly != prevRegenOnly) {
    setPill("t_regen", regenOnly, NEX_GREEN);
    prevRegenOnly = regenOnly;
  }

  // ── LOCATION ANNOUNCEMENT POPUP ─────────────────────────────
  // Triggers when od_location_counter changes (set by autostop node via SDO
  // or manually via HandleLocation() button press)
  if (od_location_counter != prevLocCounter && od_location_counter > 0) {
    String locText = getLocationText(od_location_counter);
    if (locText.length() > 0) {
      sendText("t_location", locText);
      nextionSerial.print("vis t_location,1\xFF\xFF\xFF");
      nextionSerial.print("tm_location.en=1\xFF\xFF\xFF");
    }
    prevLocCounter = od_location_counter;
  }

  // ── EMCY POLLING ────────────────────────────────────────────
  if (checkMajorEMCY()) {
    uint8_t  node;
    uint32_t code;
    if (getMajorByIndex(0, &node, &code)) {  // 0 = newest
      emcyActive = true;
      brakeFault = (code == 0x02000010 || code == 0x02000011);
      fireyFault = fireyFault || (0x00000505 <= code && code <= 0x00000507);
      if (fireyFault) {
        fireyStart = millis();
      }
      switch (code) {
        case 0x00000505: majorFaultText = "Smoke Detected";              break;
        case 0x00000506: majorFaultText = "Temp Front High";             break;
        case 0x00000507: majorFaultText = "Temp Rear High";              break;
        case 0x00000008: majorFaultText = "SDO Timeout N" + String(node); break;
        case 0x00000101: majorFaultText = "Heartbeat Lost N" + String(node); break;
        case 0x00000201: majorFaultText = "NMT Failure";                 break;
        case 0x00000301: majorFaultText = "No Shunt Data";               break;
        default: majorFaultText = "Fault 0x" + String(code, HEX);       break;
      }
    }
    setPill("t_emcy", emcyActive, NEX_RED);
    sendText("t_major", majorFaultText);
    sendColour("t_major", "pco", NEX_RED);
    refreshComponent("t_major");
  }

  if (checkMinorEMCY()) {
    uint8_t  node;
    uint32_t code;
    if (getMinorByIndex(0, &node, &code)) {  // 0 = newest
      switch (code) {
        case 0x00000510: minorFaultText = "Speed Cap Exceeded";           break;
        case 0x02000010: minorFaultText = "Brake Fault";
                         brakeFault = true;                               break;
        case 0x02000011: minorFaultText = "Brake Speed Error";
                         brakeFault = true;                               break;
        case 0x00000500: minorFaultText = "Audio SD Fault";               break;
        case 0x00000701: minorFaultText = "No Shunt Data";                break;
        case 0x00000008: minorFaultText = "SDO Timeout N" + String(node); break;
        default: minorFaultText = "Warn 0x" + String(code, HEX);         break;
      }
    }
    sendText("t_minor", minorFaultText);
    sendColour("t_minor", "pco", NEX_YELLOW);
    refreshComponent("t_minor");
  }

  // ── HORN INDICATOR ──────────────────────────────────────────
  // Handled directly in HandleHorn() on change
}

// ═══════════════════════════════════════════════════════════════
// SETUP
// ═══════════════════════════════════════════════════════════════

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Serial Coms started at 115200 baud");
  analogReadResolution(ADC_RES_BITS);

  // Nextion UART
  nextionSerial.begin(115200, SERIAL_8N1, NEXTION_RX_PIN, NEXTION_TX_PIN);

  // Input pin modes
  pinMode(BRAKE_PIN,            INPUT);
  pinMode(THROTTLE_PIN,         INPUT);
  pinMode(HORN_PIN,             INPUT_PULLUP);
  pinMode(EMCY_CLEAR_PIN,       INPUT_PULLUP);
  pinMode(LOCATION_BUTTON_PIN,  INPUT_PULLUP);
  pinMode(SERVICE_BRAKE_PIN,    INPUT_PULLUP);
  pinMode(SWITCH_2_PIN,         INPUT_PULLUP);
  pinMode(CHALLENGE_BUTTON_PIN,         INPUT_PULLUP);
  pinMode(DIRECTION_MODE_PIN,   INPUT);
  pinMode(OP_MODE_PIN,          INPUT);
  pinMode(CHALLENGE_MODE_PIN,   INPUT);

  // ADC attenuation
  analogSetPinAttenuation(BRAKE_PIN,          ADC_11db);
  analogSetPinAttenuation(THROTTLE_PIN,       ADC_11db);
  analogSetPinAttenuation(DIRECTION_MODE_PIN, ADC_11db);
  analogSetPinAttenuation(OP_MODE_PIN,        ADC_11db);
  analogSetPinAttenuation(CHALLENGE_MODE_PIN, ADC_11db);

  // ADC buffer init
  InitBuffer(&throttleBuf,  THROTTLE_PIN);
  InitBuffer(&brakeBuf,     BRAKE_PIN);
  InitBuffer(&dirBuf,       DIRECTION_MODE_PIN);
  InitBuffer(&challengeBuf, CHALLENGE_MODE_PIN);
  InitBuffer(&opModeBuf,    OP_MODE_PIN);

  // CAN init
  initCANMREX(TX_GPIO_NUM, RX_GPIO_NUM, NODE_ID);

  // FreeRTOS tasks
  xTaskCreatePinnedToCore(CAN_Task,  "CAN Task",  6144, &NODE_ID, 3, NULL, 0);
  xTaskCreatePinnedToCore(InputTask, "Input Task",6144, NULL,     2, NULL, 0);

  // ── OD registrations ───────────────────────────────────────
  // Local driver controls OD entries
  registerODEntry(0x606A, 0x00, 2, sizeof(od_motor_command),    &od_motor_command);
  registerODEntry(0x3012, 0x00, 2, sizeof(od_regen_brake),      &od_regen_brake);
  registerODEntry(0x6060, 0x00, 2, sizeof(od_direction_mode),   &od_direction_mode);
  registerODEntry(0x6062, 0x00, 2, sizeof(od_challenge_mode),   &od_challenge_mode);
  registerODEntry(0x6065, 0x00, 2, sizeof(od_horn_toggle),      &od_horn_toggle);
  registerODEntry(0x3012, 0x02, 2, sizeof(od_service_brake_dc), &od_service_brake_dc);

  // Location counter — written by autostop node (Node 6) via SDO
  registerODEntry(0x1051, 0x00, 2, sizeof(od_location_counter), &od_location_counter);

  // Incoming telemetry OD entries — populated via RPDO
  registerODEntry(0x606C, 0x00, 2, sizeof(od_true_speed),       &od_true_speed);
  registerODEntry(0x606E, 0x00, 2, sizeof(od_tractive_effort),  &od_tractive_effort);
  registerODEntry(0x2000, 0x00, 2, sizeof(od_current),          &od_current);
  registerODEntry(0x2000, 0x01, 2, sizeof(od_voltage),          &od_voltage);
  registerODEntry(0x2000, 0x02, 2, sizeof(od_soc),              &od_soc);
  registerODEntry(0x2000, 0x03, 2, sizeof(od_power),            &od_power);
  registerODEntry(0x2000, 0x04, 2, sizeof(od_recovered_energy), &od_recovered_energy);
  registerODEntry(0x1004, 0x00, 2, sizeof(od_temp_front),       &od_temp_front);
  registerODEntry(0x1004, 0x01, 2, sizeof(od_temp_rear),        &od_temp_rear);

  // ── TPDO — driver controls transmit ────────────────────────
  configureTPDO(0, 0x180 + NODE_ID, 255, 100, 100);
  PdoMapEntry tpdoEntries[] = {
    {0x3012, 0x00, 16},  // regen brake
    {0x606A, 0x00, 16},  // motor command
  };
  mapTPDO(0, tpdoEntries, 2);

  // ── RPDOs — receive telemetry from other nodes ──────────────

  // RPDO1 — Motor TPDO1 (COB-ID 0x181): true speed
  configureRPDO(0, 0x181, 255, 0);
  PdoMapEntry rpdoMotor[] = {
    {0x606C, 0x00, 32},  // true speed
  };
  mapRPDO(0, rpdoMotor, 1);

  // RPDO2 — Battery TPDO1 (COB-ID 0x187): current + voltage + SOC
  configureRPDO(1, 0x187, 255, 0);
  PdoMapEntry rpdoBatt1[] = {
    {0x2000, 0x00, 32},  // current (mA)
    {0x2000, 0x01, 16},  // voltage (mV)
    {0x2000, 0x02, 16},  // SOC (x10)
  };
  mapRPDO(1, rpdoBatt1, 3);

  // RPDO3 — Battery TPDO2 (COB-ID 0x287): power + recovered energy
  configureRPDO(2, 0x287, 255, 0);
  PdoMapEntry rpdoBatt2[] = {
    {0x2000, 0x03, 32},  // power (W)
    {0x2000, 0x04, 32},  // recovered energy (J)
  };
  mapRPDO(2, rpdoBatt2, 2);

  // RPDO4 — Lights/Sensors TPDO (COB-ID 0x184): temperature
  configureRPDO(3, 0x184, 255, 0);
  PdoMapEntry rpdoLights[] = {
    {0x1004, 0x00, 16},  // temp front (°C)
    {0x1004, 0x01, 16},  // temp rear (°C)
  };
  mapRPDO(3, rpdoLights, 2);
}

// ═══════════════════════════════════════════════════════════════
// MAIN LOOP
// ═══════════════════════════════════════════════════════════════

void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    UpdateOpMode();

    switch (nodeOperatingMode) {
      case MODE_STOPPED:     StoppedMode();     break;
      case MODE_PREOP:       PreOpMode();       break;
      case MODE_OPERATIONAL: OperationalMode(); break;
      default:               StoppedMode();     break;
    }

    updateNextion();

    Serial.println(" ");
  }
}

// ═══════════════════════════════════════════════════════════════
// OPERATING MODE FUNCTIONS
// ═══════════════════════════════════════════════════════════════

void StoppedMode() {
  Serial.println("Stopped Mode");
}

void PreOpMode() {
  Serial.print("Pre-Op Mode");
  HandleDirection();
  HandleChallenge();
  HandleParking();
  HandleHorn();
  HandleLocation();
  HandleEmcyClear();
}

void OperationalMode() {
  Serial.print("Op Mode");
  HandleChallenge();
  HandleParking();
  HandleHorn();
  HandleInputs();
  HandleLocation();
  HandleEmcyClear();
}

void UpdateOpMode() {
  int newOpModeRaw = ReadStable3PosBuffered(&opModeBuf);
  uint8_t enumOpMode = opModes[newOpModeRaw];
  if (nodeOperatingMode != enumOpMode) {
    // Safety: cannot jump directly from STOPPED to OPERATIONAL
    if (nodeOperatingMode == MODE_STOPPED && enumOpMode == MODE_OPERATIONAL) return;
    nodeOperatingMode = enumOpMode;
    Serial.print(nodeOperatingMode);
    nodeOperatingMode = enumOpMode;
    SendAllNMT(enumOpMode);
  }
}

void SendAllNMT(uint8_t operatingMode) {
  sendNMT(operatingMode, MOTOR_ID); 
  sendNMT(operatingMode, BRAKES_ID);
  sendNMT(operatingMode, LIGHTS_ID);
  sendNMT(operatingMode, AUDIO_ID);
  sendNMT(operatingMode, AUTOSTOP_ID);
  sendNMT(operatingMode, BATTERY_ID);
}

// ═══════════════════════════════════════════════════════════════
// INPUT HANDLERS
// ═══════════════════════════════════════════════════════════════

void HandleInputs() {
  od_regen_brake = 1023 - GetAverage(&brakeBuf);
  if (od_service_brake_dc == BRAKE_RELEASED) {
    od_motor_command = 1023 - GetAverage(&throttleBuf);
  } else {
    od_motor_command = 0;
  }
  Serial.print("   ||   Brake: ");    Serial.print(od_regen_brake);
  Serial.print("   ||   Throttle: "); Serial.print(od_motor_command);
}

void HandleHorn() {
  Serial.print("   ||   Horn Handle: ");
  int newHornToggle = !(digitalRead(HORN_PIN));
  Serial.print(newHornToggle);

  if (fireyFault) {
    unsigned long fireyTime = millis() - fireyStart;
    if (fireyTime < 1000) {
      newHornToggle = true;
    } else if (fireyTime < 2000) {
      newHornToggle = false;
    } else if (fireyTime < 3000) {
      newHornToggle = true;
    } else if (fireyTime < 4000) {
      newHornToggle = false;
    } else if (fireyTime < 5000) {
      newHornToggle = true;
    } else {
      newHornToggle = false;
      fireyFault = false;
    }
  }

  if (od_horn_toggle != newHornToggle) {
    od_horn_toggle = newHornToggle;
    executeSDOWrite(NODE_ID, AUDIO_ID, 0x6065, 0x00, sizeof(od_horn_toggle), &od_horn_toggle);
    if (od_horn_toggle) {
      sendText("t_horn", "Active");
      sendColour("t_horn", "pco", NEX_RED);
    } else {
      sendText("t_horn", "Inactive");
      sendColour("t_horn", "pco", NEX_GREY);
    }
    refreshComponent("t_horn");
  }
}

/**
 * @brief Handles location announcement.
 *        Automatic: od_location_counter is SDO-written by autostop node (Node 6)
 *        when loco crosses a marker. Nextion popup triggers in updateNextion().
 *        Manual override: if a marker is missed, button press increments counter
 *        and SDO-writes to both autostop and audio nodes so correct announcement
 *        plays for the next marker.
 */
void HandleLocation() {
  static bool prevBtn = false;
  bool pressed = !(digitalRead(LOCATION_BUTTON_PIN));

  if (pressed && !prevBtn) {
    // Reset location counter to 0 on autostop node only
    od_location_counter = 0;
    executeSDOWrite(NODE_ID, AUTOSTOP_ID, 0x1051, 0x00,
                    sizeof(od_location_counter), &od_location_counter);
    // Show reset confirmation on screen
    sendText("t_location", "Location Counter Reset");
    nextionSerial.print("vis t_location,1\xFF\xFF\xFF");
    nextionSerial.print("tm_location.en=1\xFF\xFF\xFF");
    Serial.println("   ||   Location counter reset to 0");
  }
  prevBtn = pressed;
}

/**
 * @brief Acknowledges and clears EMCY fault display on screen.
 *        Does NOT clear the actual EMCY from the CAN bus — the underlying
 *        fault must be resolved by the relevant node independently.
 *        This is an acknowledgement button only.
 */
void HandleEmcyClear() {
  static bool prevClearBtn = false;
  bool pressed = !(digitalRead(EMCY_CLEAR_PIN));

  if (pressed && !prevClearBtn) {
    // Reset display
    emcyActive     = false;
    brakeFault     = false;
    majorFaultText = "None";
    minorFaultText = "None";
    setPill("t_emcy", false, NEX_RED);
    sendText("t_major", "None");
    sendColour("t_major", "pco", NEX_GREEN);
    refreshComponent("t_major");
    sendText("t_minor", "None");
    sendColour("t_minor", "pco", NEX_GREEN);
    refreshComponent("t_minor");
  }
}

void HandleParking() {
  Serial.print("   ||   Parking Handle: ");
  int newServiceBrake = digitalRead(SERVICE_BRAKE_PIN);
  Serial.print(newServiceBrake);
  if (od_service_brake_dc != newServiceBrake) {
    od_service_brake_dc = newServiceBrake;
    executeSDOWrite(NODE_ID, BRAKES_ID, 0x3012, 0x02, sizeof(od_service_brake_dc), &od_service_brake_dc);
    Serial.print("Sending Parking: "); Serial.println(od_service_brake_dc);
  }
}

void HandleDirection() {
  Serial.print("   ||   Direction Handle: ");
  int newDirectionMode = ReadStable3PosBuffered(&dirBuf);
  Serial.print(newDirectionMode);
  if ((od_direction_mode != newDirectionMode) && (newDirectionMode > 0)) {
    od_direction_mode = newDirectionMode;
    executeSDOWrite(NODE_ID, MOTOR_ID,  0x6060, 0x00, sizeof(od_direction_mode), &od_direction_mode);
    executeSDOWrite(NODE_ID, LIGHTS_ID, 0x6060, 0x00, sizeof(od_direction_mode), &od_direction_mode);
    Serial.print("Sending direction: "); Serial.println(od_direction_mode);
  }
}

void HandleChallenge() {
  
    Serial.print("   ||   Challenge Handle: ");
    int newChallengeMode = ReadStable5PosBuffered(&challengeBuf);
    Serial.print(newChallengeMode);
    if ((od_challenge_mode != newChallengeMode) && (newChallengeMode > 0)) {
      od_challenge_mode = newChallengeMode;
      executeSDOWrite(NODE_ID, MOTOR_ID,    0x6062, 0x00, sizeof(od_challenge_mode), &od_challenge_mode);
      executeSDOWrite(NODE_ID, AUTOSTOP_ID, 0x6062, 0x00, sizeof(od_challenge_mode), &od_challenge_mode);
      Serial.print("Sending Challenge: "); Serial.println(od_challenge_mode);
    }
}

// ═══════════════════════════════════════════════════════════════
// ADC BUFFER FUNCTIONS
// ═══════════════════════════════════════════════════════════════

void UpdateADCBuffer(ADCBuffer* buf, int pin) {
  buf->samples[buf->index] = analogRead(pin);
  buf->index = (buf->index + 1) % BUF_SIZE;
}

int GetAverage(ADCBuffer* buf) {
  int sum = 0;
  for (int i = 0; i < BUF_SIZE; i++) sum += buf->samples[i];
  return sum / BUF_SIZE;
}

int ReadStable3PosBuffered(ADCBuffer* buf) {
  return DecodeNearest3(GetAverage(buf));
}

int ReadStable5PosBuffered(ADCBuffer* buf) {
  return DecodeNearest5(GetAverage(buf));
}

int DecodeNearest3(int raw) {
  int bestIndex = 0;
  int bestErr = abs(raw - THREE_LEVELS[0]);
  for (int i = 1; i < 3; i++) {
    int err = abs(raw - THREE_LEVELS[i]);
    if (err < bestErr) { bestErr = err; bestIndex = i; }
  }
  return bestIndex + 1;
}

int DecodeNearest5(int raw) {
  int bestIndex = 0;
  int bestErr = abs(raw - FIVE_LEVELS[0]);
  for (int i = 1; i < 5; i++) {
    int err = abs(raw - FIVE_LEVELS[i]);
    if (err < bestErr) { bestErr = err; bestIndex = i; }
  }
  return bestIndex + 1;
}

// ═══════════════════════════════════════════════════════════════
// FREERTOS TASKS
// ═══════════════════════════════════════════════════════════════

void InputTask(void* pvParameters) {
  const TickType_t delayTicks = pdMS_TO_TICKS(5); // 200Hz
  while (true) {
    UpdateADCBuffer(&throttleBuf,  THROTTLE_PIN);
    UpdateADCBuffer(&brakeBuf,     BRAKE_PIN);
    UpdateADCBuffer(&dirBuf,       DIRECTION_MODE_PIN);
    UpdateADCBuffer(&opModeBuf,    OP_MODE_PIN);
    UpdateADCBuffer(&challengeBuf, CHALLENGE_MODE_PIN);
    vTaskDelay(delayTicks);
  }
}

void InitBuffer(ADCBuffer* buf, int pin) {
  for (int i = 0; i < BUF_SIZE; i++) buf->samples[i] = analogRead(pin);
  buf->index = 0;
}
