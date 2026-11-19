/**
 * @file PCB_motor_node.ino
 * @brief PCB motor controller node.
 *
 * @details
 * Motor controller node for the PCB implementation.
 * Includes implementation of different challenge modes.
 *
 * @author Chiara Gillam
 * @author Sean Larkin
 * @author Nhan Nguyen
 *
 * @date 17/10/2026
 *
 * @version 2.1.1
 *
 * @organisation MREX
 *
 * @see motor_PCB.h
 * @see CHANGELOG.md
 */

#include <CAN_MREx.h>
#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include "motor_PCB.h"
#include "driver/pcnt.h"
#include "../../../shared/DualSerial/DualSerial.cpp"


// =============================================================================
// Global Variables
// =============================================================================

uint8_t prev_challenge_mode = 0; // od_challenge_mode = 1 on start

bool reset_energy_recovery = false; // this is to reset the energy recovery challenge

bool    entered_auto_stop = false;
int32_t pulse_start       = 0;

// Cumulative pulse count for distance tracking — updated each loop before counter clear
int32_t total_pulse_accum = 0;

// PI controller integrator accumulator
float integrator = 0.0f;

// Anti-windup clamp for integrator
float integrator_limit = 300.0f;

// Timing for 10 Hz control loop
unsigned long previous_millis = 0;

// Preferences API for non-volatile memory
Preferences preferences;

const int numFloatPairs = 15;

struct FloatPair floatPairs[numFloatPairs];

bool new_autostop_instance = true;


uint16_t motor_dac = 0;
uint16_t brake_dac = 0;
float over_speed_damping = 1;


// =============================================================================
// SETUP
// =============================================================================



void setup() {
	DualSerial.begin(115200);
    delay(1000);
    DualSerial.println("DualSerial Coms started at 115200 baud");

    preferences.begin("od_params", false);
    GetPreferences();                           // obtains existing PID parameters

    initCANMREX(TX_GPIO_NUM, RX_GPIO_NUM, NODE_ID);

    xTaskCreatePinnedToCore(
        CAN_Task,
        "CAN Task",
        6144,
        &NODE_ID,
        3,
        NULL,
        0
    );

    // Setting up DAC
    Wire.begin(SDA_PIN, SCL_PIN);
    CommonConfig();

    // Setting up Rotary Encoder
    SetupPCNT();

    pinMode(REVERSING_CONTACTOR, OUTPUT);
    pinMode(ISOLATING_RELAY, OUTPUT);
    pinMode(REGEN_BRAKE_SWITCH, OUTPUT);

    // --- Register OD entries ---
    registerODEntry(OD_TRUE_SPEED_INDEX,            OD_TRUE_SPEED_SUBINDEX,         2, sizeof(od_true_speed),           &od_true_speed);                // TPDO - 32bit
    registerODEntry(OD_REGEN_BRAKE_INDEX,           OD_REGEN_BRAKE_SUBINDEX,        2, sizeof(od_regen_brake),          &od_regen_brake);              // RPDO - 16bit
    registerODEntry(OD_SERVICE_BRAKE_MC_INDEX,      OD_SERVICE_BRAKE_MC_SUBINDEX,   2, sizeof(od_service_brake_mc),     &od_service_brake_mc);    // SDO - 8bit
    registerODEntry(OD_MOTOR_COMMAND_INDEX,         OD_MOTOR_COMMAND_SUBINDEX,      2, sizeof(od_motor_command),        &od_motor_command);          // RPDO - 16bit
    registerODEntry(OD_CONDITION_MODE_INDEX,        OD_CONDITION_MODE_SUBINDEX,     2, sizeof(od_condition_mode),       &od_condition_mode);        // SDO - 8bit
    registerODEntry(OD_DIRECTION_MODE_INDEX,        OD_DIRECTION_MODE_SUBINDEX,     2, sizeof(od_direction_mode),       &od_direction_mode);        // SDO - 8bit
    registerODEntry(OD_CHALLENGE_MODE_INDEX,        OD_CHALLENGE_MODE_SUBINDEX,     2, sizeof(od_challenge_mode),       &od_challenge_mode);        // SDO - 8bit
    registerODEntry(OD_POWER_INDEX,                 OD_POWER_SUBINDEX,              2, sizeof(od_power),                &od_power);    // RPDO - 32bit
    registerODEntry(OD_RECOVERED_ENERGY_INDEX,      OD_RECOVERED_ENERGY_SUBINDEX,   2, sizeof(od_recovered_energy),     &od_recovered_energy);    // RPDO - 32bit
    registerODEntry(OD_AUTOSTOP_DETECTION_INDEX,    OD_AUTOSTOP_DETECTION_SUBINDEX, 2, sizeof(od_autostop_detection),   &od_autostop_detection);    // SDO - 8bit
    registerODEntry(OD_CUMULATIVE_ENERGY_INDEX,     OD_CUMULATIVE_ENERGY_SUBINDEX,  2, sizeof(od_cumulative_energy),    &od_cumulative_energy);   
    
    
    // When adding a new pair, ensure to update numFloatPairs
    // Assign values here rather than statically above as floats do not initialise properly otherwise
    floatPairs[0] = {"od_kp_1", 0.016442f, OD_KP_1_INDEX, OD_KP_1_SUBINDEX};
    floatPairs[1] = {"od_ki_1", 1.7177f, OD_KI_1_INDEX, OD_KI_1_SUBINDEX};
    floatPairs[2] = {"od_kd_1", 0.3f, OD_KD_1_INDEX, OD_KD_1_SUBINDEX};
    floatPairs[3] = {"od_kp_2", 0.4f, OD_KP_2_INDEX, OD_KP_2_SUBINDEX};
    floatPairs[4] = {"od_ki_2", 0.5f, OD_KI_2_INDEX, OD_KI_2_SUBINDEX};
    floatPairs[5] = {"od_kd_2", 0.6f, OD_KD_2_INDEX, OD_KD_2_SUBINDEX};
    floatPairs[6] = {"od_kp_3", 0.7f, OD_KP_3_INDEX, OD_KP_3_SUBINDEX};
    floatPairs[7] = {"od_ki_3", 0.8f, OD_KI_3_INDEX, OD_KI_3_SUBINDEX};
    floatPairs[8] = {"od_kd_3", 0.9f, OD_KD_3_INDEX, OD_KD_3_SUBINDEX};
    floatPairs[9] = {"od_kp_4", 0.0f, OD_KP_4_INDEX, OD_KP_4_SUBINDEX};
    floatPairs[10] = {"od_ki_4", 0.1f, OD_KI_4_INDEX, OD_KI_4_SUBINDEX};
    floatPairs[11] = {"od_kd_4", 0.2f, OD_KD_4_INDEX, OD_KD_4_SUBINDEX};
    floatPairs[12] = {"od_kp_5", 0.3f, OD_KP_5_INDEX, OD_KP_5_SUBINDEX};
    floatPairs[13] = {"od_ki_5", 0.4f, OD_KI_5_INDEX, OD_KI_5_SUBINDEX};
    floatPairs[14] = {"od_kd_5", 0.5f, OD_KD_5_INDEX, OD_KD_5_SUBINDEX};
    for (int i = 0; i < numFloatPairs; i++) {
        // Assigning floatPairs[i] to a variable first does not work to update floatPairs[i].value on SDO writes
        registerODEntry(floatPairs[i].index, floatPairs[i].subindex, 2, sizeof(floatPairs[i].value), &(floatPairs[i].value)); // SDO
    }

    // --- Register TPDOs ---
    // od_true_speed
    configureTPDO(0, PDO_TS_COB_ID, 255, 100, 100);
    mapTPDO(0, PDO_TS_MAP, PDO_TS_COUNT);

    // --- Register RPDOs ---
    // od_regen_brake 
    configureRPDO(0, PDO_RGB_MTCM_COB_ID, 255, 0);
    mapRPDO(0, PDO_RGB_MTCM_MAP, PDO_RGB_MTCM_COUNT);

    // od_cumulative_energy
    configureRPDO(2, PDO_CE_COB_ID, 255, 0);
    mapRPDO(2, PDO_CE_MAP, PDO_CE_COUNT);

    /**
    * TODO
    */
    //
    configureRPDO(1, PDO_PWR_RE_COB_ID, 255, 0);
    mapRPDO(1, PDO_PWR_RE_MAP, PDO_PWR_RE_COUNT);
}



// =============================================================================
// LOOP
// =============================================================================



void loop() {
    // completing operation mode functionality
    switch (nodeOperatingMode) {
        case MODE_STOPPED:     StoppedMode();     break;
        case MODE_PREOP:       PreOpMode();       break;
        case MODE_OPERATIONAL: OperationalMode(); break;
        default:               StoppedMode();     break;  // fail-safe
    }
}


// =============================================================================
// OPERATING MODE HANDLERS
// =============================================================================

/**
 * @brief Stopped state — zero all outputs, reset integrator and put preferences to NVM.
 */
void StoppedMode() {
    WriteDAC(THROTTLE_CHANNEL, 0);
    WriteDAC(REGEN_CHANNEL, 0);
    digitalWrite(REGEN_BRAKE_SWITCH, REGEN_OFF);
    integrator = 0.0f;
    PutPreferences(); // we need to add a params dirty od entry so we only update these when they change
}

/**
 * @brief Pre-operational state — zero outputs, reset integrator, set direction contactor.
 *
 * @details
 * Zeroes all drive outputs and sets the reversing contactor based on od_direction_mode.
 * Mode 1 = reverse, 2 = neutral (contactor off), 3 = forward.
 */
void PreOpMode() {
    WriteDAC(THROTTLE_CHANNEL, 0);
    WriteDAC(REGEN_CHANNEL, 0);
    digitalWrite(REGEN_BRAKE_SWITCH, REGEN_OFF);


    integrator = 0.0f;

    switch (od_direction_mode) {
        case REVERSE_MODE:  // Reverse
            digitalWrite(REVERSING_CONTACTOR, HIGH);
            DualSerial.println("[PreOp] Direction: Reverse");
            break;
        case NEUTRAL_MODE:  // Neutral — open contactor
            digitalWrite(REVERSING_CONTACTOR, LOW); // forward
            DualSerial.println("[PreOp] Direction: Neutral");
            break;
        case FORWARD_MODE:  // Forward
            digitalWrite(REVERSING_CONTACTOR, LOW);
            DualSerial.println("[PreOp] Direction: Forward");
            break;
        default:  // Unknown — fail safe to neutral
            digitalWrite(REVERSING_CONTACTOR, LOW); // forward
            DualSerial.print("[PreOp] Unknown direction mode: ");
            DualSerial.println(od_direction_mode);
            break;
    }
}

void OperationalMode() {
    unsigned long current_millis = millis();
    if (current_millis - previous_millis < LOOP_INTERVAL_MS) return;

    // --- Read pulse count BEFORE ReadSpeedKMH() clears the counter ---
    int16_t pulse_count = 0;
    pcnt_get_counter_value(PCNT_UNIT_0, &pulse_count);      // grabs count value from rotary encoder

    // Accumulate total pulses for distance tracking
    total_pulse_accum += (int32_t)pulse_count;              // adds value to accumulated pulses

    // Rolling window for reported speed. A single 10ms loop only resolves one
    // encoder pulse = ~1.56 km/h, so averaging over a longer window is needed
    // for a usable display reading.
    static int16_t  win_pulses[SPEED_WINDOW_SAMPLES] = {0};
    static uint16_t win_ms[SPEED_WINDOW_SAMPLES]     = {0};
    static uint8_t  win_idx       = 0;
    static int32_t  win_pulse_sum = 0;
    static uint32_t win_ms_sum    = 0;

    uint16_t loop_ms = (uint16_t)(current_millis - previous_millis);
    win_pulse_sum += pulse_count - win_pulses[win_idx];
    win_ms_sum    += loop_ms     - win_ms[win_idx];
    win_pulses[win_idx] = pulse_count;
    win_ms[win_idx]     = loop_ms;
    win_idx = (win_idx + 1) % SPEED_WINDOW_SAMPLES;

    float speed_kmh = ReadSpeedKMH(previous_millis);        // clears counter here
    previous_millis = current_millis;

    float reported_kmh = 0.0f;
    if (win_ms_sum > 0) {
        float meters = (win_pulse_sum / PULSES_PER_REV) * WHEEL_CIRCUMFERENCE_M;
        reported_kmh = (meters / (win_ms_sum / 1000.0f)) * 3.6f;
    }
    od_true_speed   = (uint32_t)(reported_kmh*10 + 0.5f);   // x10 scaled


    // Detect mode transitions
    if (od_challenge_mode != prev_challenge_mode) {
        OnChallengeModeExit(prev_challenge_mode);
        OnChallengeModeEnter(od_challenge_mode);
    }


    switch (od_challenge_mode) {
        case CHALLENGE_THROTTLE:        
            entered_auto_stop = false;
            ThrottleControl(speed_kmh);   
            break;

        case CHALLENGE_SPEED_CONTROL:   
            entered_auto_stop = false;
            SpeedControl(speed_kmh);                            
            break;

        case CHALLENGE_AUTO_STOP:       
            AutoStopChallenge(speed_kmh, total_pulse_accum);    
            break;

        case CHALLENGE_ENERGY_RECOVERY: 
            entered_auto_stop = false;
            EnergyRecoveryChallenge(speed_kmh);                 
            break;

        case CHALLENGE_TRACTION:        
            entered_auto_stop = false;
            TractionChallenge(speed_kmh);                       
            break;

        default:                        
            entered_auto_stop = false;
            ThrottleControl(speed_kmh);                         
            break;
    }

    prev_challenge_mode = od_challenge_mode;

    limit_speed(speed_kmh, MAX_SPEED_KMH);

}

void limit_speed(float current_speed, float max_speed){
    
    //accesses global variables, motor_dac and brake_dac
    float damping_factor = 0.999;
    if(current_speed > max_speed){
        over_speed_damping = over_speed_damping * damping_factor;
    } else {
        over_speed_damping = over_speed_damping / (damping_factor*damping_factor); // divide ~0.998
    }
    
    if(over_speed_damping > 1){
        over_speed_damping = 1;
    }

    if(over_speed_damping < 0.1){
        over_speed_damping = 0.1;
    }
    return;
}

/**
 * @brief When exiting a challenge mode it will change these 
 */
void OnChallengeModeExit(uint8_t old_mode) {
    if (old_mode == CHALLENGE_AUTO_STOP) {
        SetServiceBrake(1);   // release
        new_autostop_instance = 1;
        pulse_start       = 0;
        total_pulse_accum = 0;

    }
    if (old_mode == CHALLENGE_ENERGY_RECOVERY) {
        digitalWrite(ISOLATING_RELAY, LOW);
        reset_energy_recovery = 1;
        SetServiceBrake(1);   // release
    }
}


/**
 * @brief Gets variable values from non-volatile memory.
 */
void OnChallengeModeEnter(uint8_t new_mode) {
    // entry setup per mode, if needed
}


// =============================================================================
// CONTROL MODES
// =============================================================================
/**
 * @brief Throttle control using motor_command.
 *
 * @details
 * Uses od_motor_command directly (like your snippet):
 * - high command + no regen  -> motor on
 * - high command + regen     -> brake
 * - low command + regen      -> brake + service brake
 * - otherwise                -> stop + service brake
 *
 * @param speed_kmh Current measured speed in km/h.
 */
void ThrottleControl(float speed_kmh) {
    motor_dac = od_motor_command;
    brake_dac = od_regen_brake;

    // Write to Throttle and Regen DACs
    WriteDAC(THROTTLE_CHANNEL, uint16_t(motor_dac*over_speed_damping));
    WriteDAC(REGEN_CHANNEL, brake_dac);
    if(brake_dac > 0){
        digitalWrite(REGEN_BRAKE_SWITCH, REGEN_ON);
    } else {
        digitalWrite(REGEN_BRAKE_SWITCH, REGEN_OFF);
    }

    DualSerial.print(" | Speed: ");
    DualSerial.print(speed_kmh);
    DualSerial.print(" | Motor DAC: ");
    DualSerial.print(motor_dac);
    DualSerial.print(" | Brake DAC: ");
    DualSerial.print(brake_dac);
}

/**
 * @brief PI speed controller — drives to od_motor_command setpoint.
 *
 * @details
 * Uses a PI loop to compute motor DAC output. Regen brake overrides
 * throttle when active. Service brake applied below SERVICE_BRAKE_SPEED_KMH.
 * Runs every LOOP_INTERVAL_MS (0.1s).
 *
 * @param speed_kmh Current measured speed in km/h.
 */
void SpeedControl(float speed_kmh) {
    motor_dac = 0;
    brake_dac = 0;

    float speed_setpoint = ((float)od_motor_command / (float)DAC_MAX) * MAX_SPEED_KMH;

    if (od_regen_brake > REGEN_BRAKE_THRESHOLD) {
        motor_dac  = 0;
        brake_dac  = od_regen_brake;
        integrator = 0.0f;
    } else {
        // Scale od_motor_command (0–1023) to speed setpoint (0–MAX_SPEED_KMH)
        // float speed_setpoint = ((float)od_motor_command / 1023.0f) * MAX_SPEED_KMH;
        float error = speed_setpoint - speed_kmh;

        // CONSTANT TRACTION MODE 1
        float_t ki = floatPairs[1].value;
        float_t kp = floatPairs[0].value;

        integrator += error * ki * LOOP_INTERVAL_MS/1000;

        if (integrator > integrator_limit) integrator = integrator_limit;
        if (integrator < 0.0f)            integrator = 0.0f;

        float control = (error * kp) + integrator;
        if (control < 0.0f)    control = 0.0f;
        // if (control > DAC_MAX) control = (float)DAC_MAX;
        
        //multiplying control by 1024;
        //I think PI params were caculated for 0 to 1 output signal not 0 to 1024
        //just multiplying by 100
        control = control * 25;
        if (control > DAC_MAX) control = (float)DAC_MAX;

        motor_dac = (uint16_t)control;
        brake_dac = 0;
    }

    WriteDAC(THROTTLE_CHANNEL, motor_dac);
    WriteDAC(REGEN_CHANNEL, brake_dac);
    if(brake_dac > 0){
        digitalWrite(REGEN_BRAKE_SWITCH, REGEN_ON);
    } else {
        digitalWrite(REGEN_BRAKE_SWITCH, REGEN_OFF);
    }
    
    DualSerial.print("[SpeedControl] Setpoint: "); DualSerial.print(speed_setpoint);
    DualSerial.print(" | Speed: ");                DualSerial.print(speed_kmh);
    DualSerial.print(" | Motor DAC: ");            DualSerial.print(motor_dac);
    DualSerial.print(" | Brake DAC: ");            DualSerial.println(brake_dac);
}



/**
 * @brief Auto-stop challenge — throttle tapers to zero at 25m then braking ramp to stop.
 *
 * @details
 * On first call, snapshots total_pulse_accum as the start reference.
 * Distance is computed from pulses accumulated in OperationalMode() before
 * each counter clear, avoiding interference with ReadSpeedKMH().
 * Throttle scales proportionally from full at 0m to zero at 25m, naturally
 * decelerating the train. Once stopped, service brake is applied.
 * Non-blocking — uses static state variables across calls.
 *
 * @param speed_kmh   Current measured speed in km/h.
 * @param pulse_accum Total accumulated pulse count from OperationalMode().
 */
void AutoStopChallenge(float speed_kmh, int32_t pulse_accum) {
    static uint16_t throttle_snapshot = 0; 

    if (!entered_auto_stop && new_autostop_instance) {
        if (od_autostop_detection == 0) {
            ThrottleControl(speed_kmh);
            return;
        }

        // Autostop marker detected od_autostop_detection == 1
        pulse_start       = pulse_accum;
        throttle_snapshot = od_motor_command;
        entered_auto_stop = true;
        DualSerial.println("[AutoStop] Challenge started");
    }

    if (entered_auto_stop) {
        int32_t pulses_since_start = pulse_accum - pulse_start;
        float distance_m = ((float)pulses_since_start / PULSES_PER_REV) * WHEEL_CIRCUMFERENCE_M;



        // --- Zone 2: Stopped ---
        // if (speed_kmh <= 0.1f) {
        //     WriteDAC(THROTTLE_CHANNEL, 0);
        //     WriteDAC(REGEN_CHANNEL, 0);
        //     digitalWrite(REGEN_BRAKE_SWITCH, REGEN_OFF);
        //     integrator = 0.0f;
        //     SetServiceBrake(true);
        //     od_autostop_detection = 0;
        //     entered_auto_stop     = false;
        //     new_autostop_instance = false;
        //     DualSerial.println("[AutoStop] Stopped — service brake applied.");
        //     return;
        // }

        // --- Zone 3: 25m reached — full regen brake ---
        if (distance_m >= AUTO_STOP_DISTANCE_M) {
            integrator = 0.0f;
            WriteDAC(THROTTLE_CHANNEL, 0);
            WriteDAC(REGEN_CHANNEL, DAC_MAX);
            if (od_service_brake_mc == 1){
                od_service_brake_mc = 0;
                executeSDOWrite(NODE_ID, BRAKES_ID, OD_SERVICE_BRAKE_MC_INDEX, OD_SERVICE_BRAKE_MC_SUBINDEX, sizeof(od_service_brake_mc), &od_service_brake_mc);
            }

            od_autostop_detection = 0;
            entered_auto_stop     = false;
            new_autostop_instance = false;

            digitalWrite(REGEN_BRAKE_SWITCH, REGEN_ON);
            DualSerial.print("[AutoStop] Regen braking to stop — Distance: "); DualSerial.print(distance_m);
            DualSerial.print("m | Speed: "); DualSerial.print(speed_kmh);
            DualSerial.print(" | Brake DAC: "); DualSerial.println(DAC_MAX);
            return;
        }

        // --- Zone 1: Throttle taper (under 25m, still moving) ---
        float distance_fraction = 1.0f - (distance_m / AUTO_STOP_DISTANCE_M); // distance remaining
        if (distance_fraction < 0.0f) distance_fraction = 0.0f;
        motor_dac = (uint16_t)(throttle_snapshot * distance_fraction);
        WriteDAC(THROTTLE_CHANNEL, motor_dac);
        WriteDAC(REGEN_CHANNEL, 0);
        digitalWrite(REGEN_BRAKE_SWITCH, REGEN_OFF);
        DualSerial.print("[AutoStop] Tapering — Distance: "); DualSerial.print(distance_m);
        DualSerial.print("m | Fraction: ");                    DualSerial.print(distance_fraction);
        DualSerial.print(" | Motor DAC: ");                   DualSerial.print(motor_dac);
        DualSerial.print(" | Speed: ");                       DualSerial.println(speed_kmh);
    }
}
/**
 * @brief Traction challenge — speed control with wheel slip detection.
 *
 * @details
 * Runs PI speed control. If measured speed exceeds desired speed by more than
 * a fixed slip margin, throttle is cut to prevent wheel spin.
 *
 * @param speed_kmh Current measured speed in km/h.
 *
 * 
 */
void TractionChallenge(float speed_kmh) {
    // const float TRACTION_SLIP_MARGIN_KMH = 2.0f;

    // float speed_setpoint = ((float)od_motor_command / 1023.0f) * MAX_SPEED_KMH;

    // if (speed_kmh > speed_setpoint + TRACTION_SLIP_MARGIN_KMH) {
    //     WriteDAC(THROTTLE_CHANNEL, uint16_t(motor_dac * over_speed_damping));
    //     WriteDAC(REGEN_CHANNEL, 0);
    //    //od_service_brake_mc = 0;
    //     integrator = 0.0f;

    //     DualSerial.print("[Traction] Slip detected. Speed: "); DualSerial.println(speed_kmh);
    //     return;
    // }

    // SpeedControl(speed_kmh);
    ThrottleControl(speed_kmh);
}


/**
 * @brief Energy recovery challenge — limit motoring to recovered energy.
 *
 * @details
 * Mode to apply to waiting and driving phase only following regeneration phase.
 *
 * @param speed_kmh Current measured speed in km/h.
 */
void EnergyRecoveryChallenge(float speed_kmh) {
    digitalWrite(ISOLATING_RELAY, HIGH);

    if (od_recovered_energy > 0) {
        ThrottleControl(speed_kmh);
    } else {
        WriteDAC(THROTTLE_CHANNEL, 0);
        WriteDAC(REGEN_CHANNEL, 0);
        digitalWrite(REGEN_BRAKE_SWITCH, REGEN_OFF);
    }
}


// =============================================================================
// HARDWARE ABSTRACTION
// =============================================================================

/**
 * @brief Reads the quadrature encoder and returns vehicle speed in km/h.
 *
 * @details
 * Reads and clears the PCNT counter, converts pulse count to revolutions,
 * then to linear speed using the elapsed time since prev_time.
 *
 * @param prev_time Timestamp of the last speed reading in milliseconds (from millis()).
 *
 * @return Measured speed in km/h.
 */
float ReadSpeedKMH(unsigned long prev_time) {
    int16_t count = 0;
    pcnt_get_counter_value(PCNT_UNIT_0, &count);
    pcnt_counter_clear(PCNT_UNIT_0);

    float elapsed_s = (millis() - prev_time) / 1000.0f;  // Elapsed time in seconds

    if (elapsed_s <= 0.0f) return 0.0f;  // Guard against divide-by-zero

    float rev       = count / PULSES_PER_REV;
    float meters    = rev * WHEEL_CIRCUMFERENCE_M;
    float speed_mps = meters / elapsed_s;
    return speed_mps * 3.6f; // m/s to km/h
}


/**
 * @brief Powers up both DAC output channels with internal voltage reference.
 */
void CommonConfig() {
    Wire.beginTransmission(DAC_ADDR);
    Wire.write(0x1F);  // COMMON-CONFIG register
    Wire.write(0x02);  // VOUT0 ON
    Wire.write(0x01);  // VOUT1 ON
    Wire.endTransmission();
}


/**
 * @brief Writes a 10-bit value to the specified DAC channel over I2C.
 *
 * @param channel DAC channel (0 = motor, 1 = regen brake).
 * @param value   10-bit output value (0–1023). Clamped if exceeded.
 */
void WriteDAC(uint8_t channel, uint16_t value) {
    if (value > DAC_MAX) value = DAC_MAX;

    uint16_t reg_val = value << 6;  // Left-align 10-bit value into 16-bit register
    uint8_t  reg     = (channel == 0) ? 0x1C : 0x19;  // DAC-0-DATA / DAC-1-DATA

    Wire.beginTransmission(DAC_ADDR);
    Wire.write(reg);
    Wire.write(reg_val >> 8);
    Wire.write(reg_val & 0xFF);
    Wire.endTransmission();
}


/**
 * @brief Configures the PCNT peripheral for quadrature encoder input.
 *
 * @details
 * ENCODER_A is the pulse input. ENCODER_B controls count direction.
 * Filter set to 1000 APB clock cycles to debounce encoder signals.
 */
void SetupPCNT() {
    pcnt_config_t pcnt_config = {};
    pcnt_config.pulse_gpio_num = ENCODER_A;
    pcnt_config.ctrl_gpio_num  = ENCODER_B;
    pcnt_config.channel        = PCNT_CHANNEL_0;
    pcnt_config.unit           = PCNT_UNIT_0;
    pcnt_config.pos_mode       = PCNT_COUNT_INC;     // Count on rising edge
    pcnt_config.neg_mode       = PCNT_COUNT_DIS;     // Ignore falling edge
    pcnt_config.lctrl_mode     = PCNT_MODE_REVERSE;  // B low = reverse direction
    pcnt_config.hctrl_mode     = PCNT_MODE_KEEP;     // B high = forward direction
    pcnt_config.counter_h_lim  = PCNT_HIGH_LIMIT;
    pcnt_config.counter_l_lim  = PCNT_LOW_LIMIT;

    pcnt_unit_config(&pcnt_config);
    pcnt_set_filter_value(PCNT_UNIT_0, 1000);
    pcnt_filter_enable(PCNT_UNIT_0);
    pcnt_counter_pause(PCNT_UNIT_0);
    pcnt_counter_clear(PCNT_UNIT_0);
    pcnt_counter_resume(PCNT_UNIT_0);
}


/**
 * @brief Sets od_service_brake_mc variable as brakes engaged or unengaged
 */
void SetServiceBrake(bool engaged) {
    od_service_brake_mc = engaged ? 1 : 0;   // 1 = braking, 0 = not braking
    executeSDOWrite(NODE_ID, BRAKES_ID, OD_SERVICE_BRAKE_MC_INDEX, OD_SERVICE_BRAKE_MC_SUBINDEX, sizeof(od_service_brake_mc), &od_service_brake_mc);
    // // ^ check your CAN_MREx signature — adjust args to match
}

/**
 * @brief Gets variable values from non-volatile memory.
 */
void GetPreferences() {
    // for (int i = 0; i < numFloatPairs; i++) {
    //     floatPairs[i].value = preferences.getFloat(floatPairs[i].key.c_str(), 0.f);
    // }
    
    floatPairs[0].value = 0.0021616f;
    floatPairs[1].value = 0.20276f;
}

/**
 * @brief Puts variable values into non-volatile memory.
 */
void PutPreferences() {
    for (int i = 0; i < numFloatPairs; i++) {
        preferences.putFloat(floatPairs[i].key.c_str(), floatPairs[i].value);
    }
}