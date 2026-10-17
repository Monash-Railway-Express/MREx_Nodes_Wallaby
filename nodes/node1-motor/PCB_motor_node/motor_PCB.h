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
 * @author Nhan Nguyen
 *
 * @date 17/10/2026
 *
 * @version 2.1.1
 *
 * @organisation MREX
 *
 * @see PCB_motor_node.ino
 */

#ifndef MOTOR_PCB_H
#define MOTOR_PCB_H

#include "../../../shared/MREx.h"
uint8_t NODE_ID = MOTOR_ID;

#include <stdint.h>

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
#define REGEN_BRAKE_SWITCH   GPIO_NUM_10
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

//challenge variable
#define AUTO_STOP_DISTANCE_M  21.0f   // Target stop distance for AutoStop challenge (m)

// pin values
#define REGEN_ON HIGH
#define REGEN_OFF LOW

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

