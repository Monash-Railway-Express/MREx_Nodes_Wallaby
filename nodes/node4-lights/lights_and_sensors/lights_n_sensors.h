/**
 * @file lights_n_sensors.h
 * @brief Header File for Lights & Sensors Node
 *
 * @details
 * Pin definitions, constants, enums, and function prototypes
 * for the Lights & Sensors node.
 *
 * @author Oscar Boulter
 * @author Nhan Nguyen
 *
 * @date 		19/10/2026
 *
 * @version 1.3.1
 *
 * @organisation MREX
 *
 * @see CAN_MREx.h
 *
 */

#ifndef LIGHTS_N_SENSORS

#define LIGHTS_N_SENSORS

#include "../../../shared/MREx.h"
uint8_t NODE_ID = LIGHTS_ID;

#include <stdint.h>


// --- CAN Pins ---
#define TX_GPIO_NUM GPIO_NUM_1 // Set GPIO pin for CAN Transmit
#define RX_GPIO_NUM GPIO_NUM_2 // Set GPIO pins for CAN Receive


// --- Light Pins ---
#define LIGHT_PREOP GPIO_NUM_17
#define LIGHT_FWD GPIO_NUM_15
#define LIGHT_REV GPIO_NUM_16

// --- Sensors Pins ---
#define SMOKE_PIN GPIO_NUM_13
#define TEMPERATURE_FRONT_PIN GPIO_NUM_12
#define TEMPERATURE_REAR_PIN GPIO_NUM_14


// --- Drive States ---
enum DriveState : uint8_t {OFF, PREOP, NEUTRAL, FORWARD, REVERSE};



// --- Function Prototypes ---
void HandleDirStates();
void LightsSelfTest();
void HandleOpMode();
void CheckSensors();


#endif // LIGHTS_N_SENSORS

