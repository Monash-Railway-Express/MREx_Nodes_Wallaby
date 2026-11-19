/**
 * @file autostop_detector.ino
 * @brief Autostop detector node — QS18VP6LP photoelectric sensor interface and
 *        marker-triggered motor stop for the MREx autostop challenge.
 *
 * @details
 * This node (Node ID 6) reads a QS18VP6LP photoelectric sensor whose two
 * complementary voltage-divided outputs are wired to SENSOR_A_PIN (positive
 * detection, HIGH when object present) and SENSOR_B_PIN (inverse/negative
 * output, must always equal logical-NOT of A). Both outputs are checked each
 * poll cycle; if they are identical the circuit is considered faulty and a
 * minor EMCY is sent once per fault onset.
 *
 * Autostop trigger sequence:
 *   1. Receive od_challenge_mode from Node 3 (driver controls) via SDO write.
 *      Node 3 writes 0x6062:00 on this node whenever the operator changes the
 *      challenge selector switch. Gate all autostop actions on == 3.
 *   2. Poll sensor outputs every SENSOR_POLL_MS. Require
 *      AUTOSTOP_CONFIRM_COUNT consecutive positive reads before triggering
 *      (anti-false-positive debounce). At 20 ms intervals this is >= 60 ms of
 *      continuous detection.
 *   3. On trigger: set od_autostop_detection = 1 (local state), latch, and
 *      SDO-write binary alert value 1 to Node 3 (driver controls) at
 *      OD address 0x3016:00. Write fires once — Node 3 owns the response.
 *   4. Latch releases when challenge mode changes away from 3 or the node
 *      leaves OPERATIONAL.
 *
 * NOTE: Hardware is ESP32-WROOM-32UE. GPIO 32 and 33 are both ADC1 channels
 * (CH4 and CH5) — reliable analog inputs with no peripheral conflicts.
 *
 * @author Patrick McCarthy
 * @author Audrey Tasevski
 * @author Aung Hpone Thant
 * @author Nhan Nguyen
 *
 * @date 19/10/2026
 *
 * @version 1.1.3
 *
 * @organisation MREX
 */

#include "../../../shared/MREx.h"
uint8_t NODE_ID = AUTOSTOP_ID;
#include <CAN_MREx.h>
#include <Preferences.h>


// User code begin: -------------------------------------------------------

//Defining Preferences object to store current location for location announcement
//docs: https://docs.espressif.com/projects/arduino-esp32/en/latest/tutorials/preferences.html
Preferences locAnnouncePrefs;


// --- CAN transceiver pins ---
#define TX_GPIO_NUM GPIO_NUM_14
#define RX_GPIO_NUM GPIO_NUM_13

// --- Location Announce Stuff ---
#define LOC_ANN1_START 1
#define LOC_ANN2_AUTOSTOP 4
#define LOC_ANN3_COMFORT 5
#define LOC_ANN4_COMFORT_END 6
#define LOC_ANN5_HAVEN 7
#define LOC_ANN6_TCN 8
#define LOC_ANN7_TCN_END 9
#define LOC_ANN8_END 10

#define LOC_ANN_CD 1000 //cooldown between location annoucement changes 5s
#define LOC_ANN_MAX_COUNT 10
bool overrideIncrement = LOW; //this is set when a challenge mode (eg. Autostop, Energy Recovery, Traction) is selected, to override the normal incrementing
bool standstillFlag = LOW; //flag for when a standstill announcement is triggered (eg. ride comfort start)

//Private functions
uint8_t _ChangeLocation();

// --- Sensor GPIO pins ---
// Hardware: QS18VP6LP outputs are voltage-divided so max voltage <= 3.3V but
// may be as low as 1V when HIGH. digitalRead is unreliable at 1V (WROOM-32UE
// digital HIGH threshold is ~2.475V), so analogRead + threshold is used.
// A (GPIO 32) = positive detection output (HIGH when reflector detected).
// B (GPIO 33) = inverse/negative output — must always equal logical-NOT of A.
static const uint8_t SENSOR_A_PIN = 1U;  // Reflector detected (ADC1_CH4)
static const uint8_t SENSOR_B_PIN = 2U;  // Not detected / inverse output (ADC1_CH5)

// ADC threshold for treating a sensor output as logic HIGH.
// 200 / 4095 * 3.3V ~ 0.16V — safely above noise, well below the 1V minimum
// HIGH voltage. Adjust if noise floor is higher on the physical board.
static const uint16_t SENSOR_DETECT_THRESHOLD_ADC = 200;

// --- Timing constants (all in milliseconds) ---
static const uint32_t SENSOR_POLL_MS = 20;   // Sensor read interval

// --- Safety threshold ---
// Number of consecutive SENSOR_POLL_MS reads that must all be HIGH before the
// autostop triggers. Prevents transient reflections or EMI from causing a
// false stop. At 20 ms interval: 3 reads = minimum 60 ms of continuous
// detection required.
static const uint8_t AUTOSTOP_CONFIRM_COUNT   = 2;

// --- EMCY error codes (Node 6 range: 0x0006xxxx) ---
// Priority 0 = major (halts system), priority 1 = minor (logged warning, no halt).
static const uint32_t EMCY_SENSOR_CIRCUIT_FAULT = 0x00060601;  // A == B (circuit open/short)
static const uint32_t EMCY_AUTOSTOP_TRIGGERED   = 0x00060602;  // Marker detected, alert sent

// --- Misc ---
static const uint32_t SERIAL_BAUD_RATE = 115200;
static const uint32_t STARTUP_DELAY_MS = 1000;

// od_challenge_mode was 0 but is now initalised as CHALLENGE_THROTTLE=1 in the common header

// User code end ----------------------------------------------------------

// --- Internal node state ---
// These are reset on every entry to StoppedMode so state does not carry
// across NMT cycles.
static bool    autostopLatched    = false; // True once marker confirmed; stays until mode changes
static bool    circuitFaultLatch  = false; // Prevents EMCY flood on sustained circuit fault
static uint8_t sensorConfirmCount = 0U;   // Consecutive positive sensor reads


// =============================================================================
// Internal helpers (static, not in public interface)
// =============================================================================

/**
 * @brief Read both sensor outputs and classify each as HIGH or LOW.
 *
 * @details Uses analogRead because the sensor HIGH voltage may be as low as
 * 1V — below the ESP32-S3 digital HIGH threshold of ~2.475V. Returns true
 * when A and B are binary opposites (circuit healthy), false when equal
 * (circuit fault — open wire, short, or sensor failure).
 *
 * @param[out] aHigh  True if Sensor A output is above detection threshold.
 * @param[out] bHigh  True if Sensor B output is above detection threshold.
 * @return true  Circuit is healthy (A != B).
 * @return false Circuit is faulty  (A == B).
 */
static bool _ReadSensor(bool *aHigh, bool *bHigh) {
    int aAdc = analogRead(SENSOR_A_PIN);
    int bAdc = analogRead(SENSOR_B_PIN);

    *aHigh = (aAdc >= SENSOR_DETECT_THRESHOLD_ADC);
    *bHigh = (bAdc >= SENSOR_DETECT_THRESHOLD_ADC);

    // A and B are wired to complementary sensor outputs. They must always
    // be logical opposites. Equality means a wire is open or shorted.
    return (*aHigh != *bHigh);
}


/**
 * @brief Non-blocking sensor circuit health check. Sends minor EMCY on fault onset.
 *
 * @details Called in both PreOp and Operational modes. A minor EMCY (priority 1)
 * is sent once when the fault first appears — it logs the warning on the CAN bus
 * without halting the system. circuitFaultLatch prevents repeated sends.
 * Latch clears automatically when the circuit recovers.
 */
static void _CheckSensorCircuit(void) {
    static uint32_t lastMs = 0;
    uint32_t nowMs = millis();

    if ((nowMs - lastMs) < SENSOR_POLL_MS) {
        return;
    }
    lastMs = nowMs;

    bool aHigh, bHigh;
    bool valid = _ReadSensor(&aHigh, &bHigh);

    if (!valid) {
        // Send a minor EMCY once per fault onset — logs the warning on the CAN
        // bus without halting the system. Priority 1 = minor (not a major fault).
        if (!circuitFaultLatch) {
            sendEMCY(1, NODE_ID, EMCY_SENSOR_CIRCUIT_FAULT);
            circuitFaultLatch = true;
            Serial.println("[Autostop] WARN: Sensor circuit fault — A == B.");
        }
    } else {
        if (circuitFaultLatch) {
            Serial.println("[Autostop] INFO: Sensor circuit recovered.");
        }
        circuitFaultLatch = false;
    }
}

/**
 * @brief Print a warning if the sensor detects the reflector but the node is
 *        not in the correct state to act on it.
 *
 * @details Runs every SENSOR_POLL_MS. Prints nothing when the node is
 * correctly in OPERATIONAL + autostop challenge mode (that path is handled by
 * _HandleAutostop). Only fires when a detection would otherwise be silently
 * ignored.
 */
static void _WarnIfDetectedWrongMode(void) {
    static uint32_t lastMs      = 0;
    static bool     warnPrinted = false;
    uint32_t nowMs = millis();
    if ((nowMs - lastMs) < SENSOR_POLL_MS) return;
    lastMs = nowMs;

    int aAdc = analogRead(SENSOR_A_PIN);
    bool detected = (aAdc >= SENSOR_DETECT_THRESHOLD_ADC);

    if (!detected) {
        warnPrinted = false;
        return;
    }

    if (warnPrinted) return;

    if (nodeOperatingMode != MODE_OPERATIONAL) {
        Serial.println("[Autostop] WARNING: detection but not in operational mode.");
        warnPrinted = true;
    } 
    // else if (od_challenge_mode != AUTOSTOP_CHALLENGE_VALUE) {
    //     Serial.println("[Autostop] WARNING: detection but not in autostop mode.");
    //     warnPrinted = true;
    // }
}

/**
 * @brief Confirmation counter, latch logic, and periodic stop assertion.
 *
 * @details Requires AUTOSTOP_CONFIRM_COUNT consecutive HIGH readings on
 * Sensor A (with valid circuit) before triggering. Any non-HIGH or invalid
 * reading resets the counter to zero. Once latched:
 *  - od_autostop_detection is set to 1 and SDO-written to Node 3 (0x1050:00).
 *  - _SendDetectionAlert() is stubbed pending Node 3 implementation (0x3016:00).
 *  - The latch stays until challenge mode changes (Node 3 writes a new value
 *    to od_challenge_mode; OperationalMode detects and releases the latch)
 *    or the node leaves OPERATIONAL (StoppedMode resets all state).
 */
static void _HandleAutostop(void) {
    static uint32_t lastSensorMs = 0;
    static uint32_t lastSDOWrite = 0;
    uint32_t nowMs = millis();

    // Latch holds until Node 3 writes a new od_challenge_mode != 3.
    // OperationalMode() detects that change and resets the latch.
    if (autostopLatched) {
        return;
    }

    // Poll sensor at SENSOR_POLL_MS interval.
    if ((nowMs - lastSensorMs) < SENSOR_POLL_MS) {
        return;
    }
    lastSensorMs = nowMs;

    bool aHigh, bHigh;
    bool valid = _ReadSensor(&aHigh, &bHigh);

    if (!valid) {
        // Circuit fault — do not trigger. EMCY is handled by _CheckSensorCircuit.
        sensorConfirmCount = 0;
        return;
    }

    if (aHigh) {
        sensorConfirmCount++;

        if (sensorConfirmCount >= AUTOSTOP_CONFIRM_COUNT) {
            autostopLatched       = true;
            od_autostop_detection = 1;

            // SDO write od_autostop_detection to Node 3 so it knows the
            // marker was detected. Node 3 must register 0x1050:00 as a
            // writable uint8_t OD entry to receive this.
            if((nowMs-lastSDOWrite)>500){
                executeSDOWrite(NODE_ID, MOTOR_ID,
                                OD_AUTOSTOP_DETECTION_INDEX, OD_AUTOSTOP_DETECTION_SUBINDEX,
                                sizeof(od_autostop_detection), &od_autostop_detection);
                lastSDOWrite = millis();
                // _SendDetectionAlert();
                // sendEMCY(1, NODE_ID, EMCY_AUTOSTOP_TRIGGERED);
                Serial.println("[Autostop] TRIGGERED — marker confirmed, detection sent to Node 1 - Motor.");
            }
        }
    } else {
        sensorConfirmCount = 0;
    }
}

static void _SendPassing() {
    executeSDOWrite(NODE_ID, AUDIO_ID,  OD_LOCATION_COUNTER_INDEX, OD_LOCATION_COUNTER_SUBINDEX, sizeof(od_location_counter), &od_location_counter);
    executeSDOWrite(NODE_ID, DRIVER_ID, OD_LOCATION_COUNTER_INDEX, OD_LOCATION_COUNTER_SUBINDEX, sizeof(od_location_counter), &od_location_counter);
}

/*
@brief function to call a standstill announcement to be sent. Sets a flag when called, and only sends location announcement  when loco stops
*/
static void _SendStandStill(){
    if(!standstillFlag){
      standstillFlag = HIGH;
    }
}

uint8_t _ChangeLocation(){
    static uint32_t lastMs      = 0;
    static uint32_t lastValidLoc = 0; //the last time a valid location was detected
    static uint8_t prev_challenge_mode = 0;

    static bool autostopArmed;
    static bool tractionArmed;

    //change detection
    //set flag for autostop and traction announcement start.
    if(prev_challenge_mode != od_challenge_mode){
      prev_challenge_mode = od_challenge_mode;
      switch(od_challenge_mode){
        Serial.print("CHALLENGE VALUE:");
        Serial.println(od_challenge_mode);
        case CHALLENGE_AUTO_STOP:
        autostopArmed = HIGH;
        tractionArmed = LOW;
        break;
        
        case CHALLENGE_TRACTION:
        autostopArmed = LOW;
        tractionArmed = HIGH;
        break;

        default:
        autostopArmed = LOW;
        tractionArmed = LOW;
        break;
      }
    }
    
    uint32_t nowMs = millis();
    uint8_t newLocation = od_location_counter;
    if ((nowMs - lastMs) < SENSOR_POLL_MS) return od_location_counter;
    lastMs = nowMs;

    bool aHigh, bHigh;
    bool valid = _ReadSensor(&aHigh, &bHigh);

    if (!valid) {
        // Circuit fault — do not trigger. EMCY is handled by _CheckSensorCircuit.
        sensorConfirmCount = 0;
        return od_location_counter;

    }

    if (aHigh) {
        sensorConfirmCount++;


        if (sensorConfirmCount >= AUTOSTOP_CONFIRM_COUNT) {
            if((nowMs - lastValidLoc) >= LOC_ANN_CD){
              if(od_location_counter < LOC_ANN_MAX_COUNT){
                if(autostopArmed){
                  newLocation = LOC_ANN2_AUTOSTOP;
                }else if(tractionArmed){
                  newLocation = LOC_ANN6_TCN;
                }else{
                  newLocation = od_location_counter + 1;
                }
                
              }else{
                newLocation = 0;
              }
              
              lastValidLoc = nowMs;
              Serial.println("Sensor TRIGGERED — marker confirmed, location updated" );
            }
           

        
        }
    }else {
        sensorConfirmCount = 0;
    }

    return newLocation;
}

/**
 * @brief function to iterate the location of the train
 *
 * @details Point Positions per RULES
    1 - Past station Box (Passing)
    // Have 2 traction markers here
    2 - Autostop activation (Passing)
    3 - Ready for Ride comfort (Stand still)
    4 - Completing Ride comfort (Stand Still)
    5 - Haven on return (Passing)
    6 - Ready for traction (Stand Still)
    7 - Completion traction (Passing)
    8 - Before Head Shunt (Stand Still)
 */
static void _HandleLocationAnnoucement() {
    //Check the location counter and update 
    uint8_t newLocation = _ChangeLocation();
    
    
    
    if(od_true_speed<=0 && standstillFlag){
        executeSDOWrite(NODE_ID, AUDIO_ID,  OD_LOCATION_COUNTER_INDEX, OD_LOCATION_COUNTER_SUBINDEX, sizeof(od_location_counter), &od_location_counter);
        executeSDOWrite(NODE_ID, DRIVER_ID, OD_LOCATION_COUNTER_INDEX, OD_LOCATION_COUNTER_SUBINDEX, sizeof(od_location_counter), &od_location_counter);
        standstillFlag = LOW;
    }

    if(newLocation != od_location_counter){
        od_location_counter = newLocation;
        Serial.print("At location: " );
        Serial.println(od_location_counter);
        locAnnouncePrefs.putUChar("Location", od_location_counter);
        switch (od_location_counter) {
            case LOC_ANN1_START: _SendPassing()   ; break; //Point 1
            //Have traction markers here in circuit
            case LOC_ANN2_AUTOSTOP: _SendPassing()   ; break; //Point 2
            case LOC_ANN3_COMFORT: _SendStandStill(); break; //Point 3
            case LOC_ANN4_COMFORT_END: _SendStandStill(); break; //Point 4
            case LOC_ANN5_HAVEN: _SendPassing()   ; break; //Point 5
            case LOC_ANN6_TCN: _SendStandStill(); break; //Point 6
            case LOC_ANN7_TCN_END: _SendPassing()   ; break;
            case LOC_ANN8_END:_SendStandStill(); break;
            default: break;  // fail-safe
        }   
    }
}

// =============================================================================
// Operating mode handlers
// =============================================================================

/**
 * @brief Stopped state — reset all autostop state. No CAN writes.
 */
void StoppedMode(void) {
    autostopLatched       = false;
    circuitFaultLatch     = false;
    od_autostop_detection = 0;
    sensorConfirmCount    = 0;
    // od_challenge_mode is NOT reset here — it is owned by Node 3 and should
    // retain the last written value so it is immediately valid on re-entry
    // to OPERATIONAL without needing Node 3 to re-send it.
}


/**
 * @brief Pre-operational state — monitor sensor circuit for faults only.
 *
 * @details Autostop trigger is disabled in PreOp. Only circuit health is
 * checked so faults are caught before the challenge begins.
 */
void PreOpMode(void) {
    _CheckSensorCircuit();
}


/**
 * @brief Operational state — full autostop logic active.
 *
 * @details od_challenge_mode is written by Node 3 via SDO whenever the
 * operator changes the challenge selector switch. This node reads it directly
 * from its own OD each loop. When challenge mode == 3, the sensor confirmation
 * and latch logic runs. Circuit health is always checked so faults are
 * reported in any challenge mode.
 */
void OperationalMode(void) {
    // Release latch if Node 3 has written a new challenge mode that is not
    // autostop. od_challenge_mode is updated in the background by the CAN task
    // whenever Node 3 SDO-writes it.
    if (autostopLatched && (od_challenge_mode != CHALLENGE_AUTO_STOP)) {
        Serial.println("[Autostop] INFO: Challenge mode changed — latch released.");
        autostopLatched       = false;
        od_autostop_detection = 0;
        sensorConfirmCount    = 0;
    }

    _CheckSensorCircuit();

    _HandleLocationAnnoucement();

    if (od_challenge_mode == CHALLENGE_AUTO_STOP) {
        _HandleAutostop();
    }
}


// =============================================================================
// Arduino entry points
// =============================================================================

void setup() {
    Serial.begin(SERIAL_BAUD_RATE);
    delay(STARTUP_DELAY_MS);
    Serial.println("Serial Coms started at 115200 baud");

    // Initialise CAN MREx and start the FreeRTOS CAN task.
    // NODE_ID must NOT be const — task takes &NODE_ID as pvParameters.
    initCANMREX(TX_GPIO_NUM, RX_GPIO_NUM, NODE_ID);

    xTaskCreatePinnedToCore(
        CAN_Task,
        "CAN Task",
        4096,
        &NODE_ID,
        3,
        NULL,
        0
    );

    // User code Setup Begin: -----------------------------------------------

    // --- Register OD entries ---
    // od_challenge_mode: written by Node 3 (driver controls) via SDO on switch
    // change. Registered as RW so the CAN library accepts incoming SDO writes.
    registerODEntry(OD_CHALLENGE_MODE_INDEX, OD_CHALLENGE_MODE_SUBINDEX,
                    2, sizeof(od_challenge_mode), &od_challenge_mode);

    // od_autostop_detection: written by this node; readable by others via SDO.
    registerODEntry(OD_AUTOSTOP_DETECTION_INDEX, OD_AUTOSTOP_DETECTION_SUBINDEX,
                    2, sizeof(od_autostop_detection), &od_autostop_detection);

    registerODEntry(OD_LOCATION_COUNTER_INDEX, OD_LOCATION_COUNTER_SUBINDEX, 2, sizeof(od_location_counter), &od_location_counter);

    registerODEntry(OD_TRUE_SPEED_INDEX, OD_TRUE_SPEED_SUBINDEX, 2, sizeof(od_true_speed), &od_true_speed);
    //map speed rpdo
    configureRPDO(0, PDO_TS_COB_ID, 255, 0);
    mapRPDO(0, PDO_TS_MAP, PDO_TS_COUNT);

    // --- Set pin modes ---
    // Sensor pins are analog inputs. analogRead does not require pinMode but
    // INPUT is set explicitly to prevent accidental driving of the pins.
    pinMode(SENSOR_A_PIN, INPUT);
    pinMode(SENSOR_B_PIN, INPUT);

    //TO DO - set up PDO


    //initialise Preferences object
    locAnnouncePrefs.begin("myPrefs", false);
    bool doesLocationValueExist = locAnnouncePrefs.isKey("Location");

    if(!doesLocationValueExist){
      locAnnouncePrefs.putUChar("Location", 0);
    }else{
      od_location_counter = locAnnouncePrefs.getUChar("Location");
    }

    // User code Setup end --------------------------------------------------
}


void loop() {
    //User Code begin loop() -----------------------------------------------

    _WarnIfDetectedWrongMode();

    switch (nodeOperatingMode) {
        case MODE_STOPPED:      StoppedMode();      break;
        case MODE_PREOP:        PreOpMode();        break;
        case MODE_OPERATIONAL:  OperationalMode();  break;
        default:                StoppedMode();      break;  // fail-safe
    }

    //User code end loop() -------------------------------------------------
}
