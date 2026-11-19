#include <stdint.h>

#ifndef CONTROLLER
#define CONTROLLER

#include "../../../shared/MREx.h"
uint8_t NODE_ID = DRIVER_ID;

// Defining Function prototypes
void SendAllNMT(uint8_t operatingMode);

void UpdateOpMode();
void StoppedMode();
void PreOpMode();
void OperationalMode();

void HandleInputs();
void HandleHorn();
void HandleParking();
void HandleDirection();
void HandleChallenge();

int DecodeNearest3(int raw);
int DecodeNearest5(int raw);

#define BUF_SIZE 15

typedef struct {
    int samples[BUF_SIZE];
    int index;
} ADCBuffer;

void InputTask(void* pvParameters);
int ReadStable5PosBuffered(ADCBuffer* buf);
int ReadStable3PosBuffered(ADCBuffer* buf);
int GetAverage(ADCBuffer* buf);
void UpdateADCBuffer(ADCBuffer* buf, int pin);
void InitBuffer(ADCBuffer* buf, int pin);

// Defining Pins
#define TX_GPIO_NUM GPIO_NUM_41
#define RX_GPIO_NUM GPIO_NUM_40

#define BRAKE_PIN       GPIO_NUM_1
#define THROTTLE_PIN    GPIO_NUM_2

#define HORN_PIN          GPIO_NUM_15
#define EMCY_CLEAR_PIN    GPIO_NUM_16
#define LOCATION_BUTTON_PIN    GPIO_NUM_12
// #define BUTTON_2_PIN   GPIO_NUM_16  // commented out — GPIO16 used for Nextion RX
#define CHALLENGE_BUTTON_PIN      GPIO_NUM_14   // temp reassignment — button unconnected
#define SERVICE_BRAKE_PIN GPIO_NUM_36
#define SWITCH_2_PIN      37

#define DIRECTION_MODE_PIN  GPIO_NUM_6
#define CHALLENGE_MODE_PIN  GPIO_NUM_3
#define OP_MODE_PIN         GPIO_NUM_4

// Speed at which the Nextion speed bar reads 100% (km/h)
#define SPEED_BAR_FULL_SCALE_KMH  15.0f

// Nextion UART pins
#define NEXTION_TX_PIN  GPIO_NUM_18
#define NEXTION_RX_PIN  GPIO_NUM_17

/*
location announcement counter values. 
Location announcement counter is incremented via SDO every time train passes a marker.
At these set values, the corresponding announcement will play
Note: 2 and 3 are skipped as these are the traction challenge markers passed over when challenge starts
*/
#define LOC_ANN1_START 1
#define LOC_ANN2_AUTOSTOP 4
#define LOC_ANN3_COMFORT 5
#define LOC_ANN4_COMFORT_END 6
#define LOC_ANN5_HAVEN 7
#define LOC_ANN6_TCN 8
#define LOC_ANN7_TCN_END 9
#define LOC_ANN8_END 10
// ── Nextion colour constants ────────────────────────────────────
#define NEX_GREEN  1339
#define NEX_YELLOW 65504
#define NEX_RED    63488
#define NEX_WHITE  65535
#define NEX_GREY   33808
#define NEX_CYAN   1055
#define NEX_DARK   10

#define BRAKE_APPLIED 0
#define BRAKE_RELEASED 1

const ODOperatingMode opModes[4] = {MODE_STOPPED, MODE_STOPPED, MODE_PREOP, MODE_OPERATIONAL};

// ADC / filtering settings
const int ADC_RES_BITS = 10;
const int ADC_SAMPLES  = 20;
const int POT_DEADBAND = 10;

// Expected raw ADC levels
const int THREE_LEVELS[3] = {256, 512, 768};
const int FIVE_LEVELS[5]  = {171, 341, 512, 682, 853};

#endif // CONTROLLER
