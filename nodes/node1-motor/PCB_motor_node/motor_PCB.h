/**
 * @file motor_PCB.h
 * @brief Header file for PCB motor node.
 *
 * @details
 * Pin definitions, constants, enums, and function prototypes
 * for the PCB motor controller node.
 *
 * @author Chiara Gillam
 * @author Sean Larkin
 *
 * @date 13/03/2026
 *
 * @version 2.0.0
 *
 * @organisation MREX
 *
 * @see PCB_motor_node.ino
 */

#ifndef MOTOR_PCB_H
#define MOTOR_PCB_H

#include <stdint.h>

// --- Node ID ---
uint8_t NODE_ID = 1;

//Defining Node ID's
#define MOTOR_ID 0x01
#define BRAKES_ID 0x02
#define DRIVER_ID 0x03
#define LIGHTS_ID 0x04
#define AUDIO_ID 0x05
#define AUTOSTOP_ID 0x06
#define BATTERY_ID 0x07
#define LOGGER_ID 0x08
#define LCD_ID 0x09


// --- CAN Pins ---
#define TX_GPIO_NUM GPIO_NUM_14
#define RX_GPIO_NUM GPIO_NUM_13

// --- DAC I2C ---
#define DAC_ADDR  0x48
#define SDA_PIN   GPIO_NUM_2
#define SCL_PIN   GPIO_NUM_1

#define THROTTLE_CHANNEL 0
#define REGEN_CHANNEL 1

// --- GPIO Pins ---
#define REVERSING_SWITCH_MC  GPIO_NUM_5
#define REGEN_BRAKE_SWITCH         GPIO_NUM_10
#define FORWARD_SWITCH       GPIO_NUM_4 
#define MOTOR1_GREEN_LED     GPIO_NUM_7
#define MOTOR2_GREEN_LED     GPIO_NUM_15
#define MOTOR1_RED_LED       GPIO_NUM_16
#define MOTOR2_RED_LED       GPIO_NUM_17
#define ENCODER_A            GPIO_NUM_18
#define ENCODER_B            GPIO_NUM_8
#define ENCODER_Z            GPIO_NUM_9

//Free pins
//#define FREE_PIN_1 GPIO_NUM_10
#define ISOLATING_RELAY   GPIO_NUM_11
// #define FREE_PIN_1 GPIO_NUM_12
// #define FREE_PIN_1 GPIO_NUM_48

#define REVERSING_CONTACTOR GPIO_NUM_47
//#define ISOLATING_RELAY   GPIO_NUM_11

// --- Pulse Counter ---
#define PCNT_HIGH_LIMIT  32767
#define PCNT_LOW_LIMIT   0

// --- Physical Constants ---
const float PULSES_PER_REV       = 200.0f;
const float WHEEL_DIAMETER_M = 0.275f;
const float WHEEL_CIRCUMFERENCE_M = WHEEL_DIAMETER_M*3.14159f;

// --- Control Limits ---
#define DAC_MAX              1023
#define REGEN_BRAKE_THRESHOLD  100   // Minimum regen brake value to activate
#define LOOP_INTERVAL_MS      10   // Control loop period in ms
#define SPEED_WINDOW_SAMPLES  20   // Loops averaged for reported speed (20 x 10ms = 200ms, ~0.08 km/h resolution)
#define MAX_SPEED_KMH  15.0f   // Hard speed cap — throttle cut above this speed (km/h)

// --- Speed Thresholds ---
#define SERVICE_BRAKE_SPEED_KMH  2.0f   // Apply service brake below this speed (km/h)
#define AUTO_STOP_SPEED_KMH      5.0f   // Target stop speed for AutoStop challenge (km/h)

// --- Challenge Mode Values ---
#define CHALLENGE_THROTTLE        1
#define CHALLENGE_SPEED_CONTROL   2
#define CHALLENGE_AUTO_STOP       3
#define CHALLENGE_ENERGY_RECOVERY 4
#define CHALLENGE_TRACTION        5

//challenge variable
#define AUTO_STOP_DISTANCE_M  21.0f   // Target stop distance for AutoStop challenge (m)

// pin values
#define REGEN_ON HIGH
#define REGEN_OFF LOW

// --- Operating Modes ---
enum OperatingMode : uint8_t {
    MODE_STOPPED        = 0x02,
    MODE_PREOP          = 0x80,
    MODE_OPERATIONAL    = 0x01
};

// --- Direction Mode ---
enum DirectionMode : uint8_t {
    REVERSE_MODE        = 1,
    NEUTRAL_MODE        = 2,
    FORWARD_MODE        = 3
};

// --- Function Prototypes ---
void CommonConfig();
void WriteDAC(uint8_t channel, uint16_t value);
void SetupPCNT();
float ReadSpeedKMH(unsigned long prev_time);
void StoppedMode();
void PreOpMode();
void OperationalMode();
void SpeedControl(float speed_kmh);
void ThrottleControl(float speed_kmh);
void AutoStopChallenge(float speed_kmh, int32_t pulse_accum);
void TractionChallenge(float speed_kmh);
void EnergyRecoveryChallenge(float speed_kmh);
void GetPreferences();
void PutPreferences();

// PI controller gains — tuned by MUNT
struct FloatPair {
    String key;
    float_t value;
    uint16_t index;
    uint8_t subindex;
};

#endif // MOTOR_PCB_H

