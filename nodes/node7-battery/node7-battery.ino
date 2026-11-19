/**
 * @file battery.ino
 * @brief Code to compute recovered energy
 * @details The code computes recovered energy using the power values received from the smart shunt. It also transmits current, voltage, SOC, power , and recovered energy onto the can bus.
 * It uses VedirectFrameHandler.cpp and VeDirectFrameHandler.h files which parse data received from the shunt and stores in a buffer that can be read.
 * @author Hoor E Jannat Urboshi
 * @author Arjuna Edirisinghe
 * @author Nhan Nguyen
 * @date 		19/10/2026
 * @version 2.0.1
 * @organisation MREX
 * @see VeDirectFrameHandler.c and VeDirectFrameHandler.h
 */

#include "../../shared/MREx.h"
uint8_t NODE_ID = BATTERY_ID;
#include <Arduino.h>
#include <stdint.h>
#include <CAN_MREx.h>
#include <HardwareSerial.h>

#include "VeDirectFrameHandler.h" // ve direct parser

// =============================================================================
// Constants and other
// =============================================================================

// --- Pin Definitions ---
#define TX_GPIO_NUM GPIO_NUM_5 // GPIO pin for CAN Transmit
#define RX_GPIO_NUM GPIO_NUM_4 // GPIO pins for CAN Receive

// =============================================================================
// Global Variables
// =============================================================================
int32_t current = 0; 
int32_t power = 0;
int32_t recovered_energy = 0;
uint32_t prev_power_sample = 0;
uint32_t new_power_sample = 0;
uint32_t power_sample = 0;
int32_t slice_area = 0;
bool prev_sample_1 = false; // flag that is set to true after the first power sample after regen starts is received
bool prev_sample_2 = false; // flag that is set to true after the first power sample after loco starts up again is received4
unsigned long currentMillis = 0;
unsigned long last_data_received_time = 0;
const unsigned long shunt_data_interval = 1500; // If 1.5s pass without receiving data from the shunt, then send error message
int32_t prev_time = 0;

VeDirectFrameHandler veParser;

HardwareSerial veSerial(2); // Use UART2 for shunt data

// User code end ---------------------------------------------------------

void setup() {

  Serial.begin(115200); 
  delay(1000);
  Serial.println("Serial Coms started at 115200 baud");

  veSerial.begin(19200, SERIAL_8N1, 16, 17); // GPIO 16 - RX; GPIO 17 -TX. // ESP32 <-> Shunt
  delay(1000);
  Serial.println("Reading values from shunt started at 19200 baud");
  
  //Initialize CANMREX protocol
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

  // --- Register OD entries ---
  registerODEntry(OD_CURRENT_INDEX,           OD_CURRENT_SUBINDEX,            2, sizeof(od_current),            &od_current); 
  registerODEntry(OD_VOLTAGE_INDEX,           OD_VOLTAGE_SUBINDEX,            2, sizeof(od_voltage),            &od_voltage); 
  registerODEntry(OD_SOC_INDEX,               OD_SOC_SUBINDEX,                2, sizeof(od_soc),                &od_soc); 
  registerODEntry(OD_POWER_INDEX,             OD_POWER_SUBINDEX,              2, sizeof(od_power),              &od_power);
  registerODEntry(OD_RECOVERED_ENERGY_INDEX,  OD_RECOVERED_ENERGY_SUBINDEX,   2, sizeof(od_recovered_energy),   &od_recovered_energy);
  registerODEntry(OD_CUMULATIVE_ENERGY_INDEX, OD_CUMULATIVE_ENERGY_SUBINDEX,  2, sizeof(od_cumulative_energy),  &od_cumulative_energy);
  registerODEntry(OD_TRUE_SPEED_INDEX,        OD_TRUE_SPEED_SUBINDEX,         2, sizeof(od_true_speed),         &od_true_speed);
  registerODEntry(OD_REGEN_BRAKE_INDEX,       OD_REGEN_BRAKE_SUBINDEX,        2, sizeof(od_regen_brake),        &od_regen_brake);              // RPDO - 16bit
  registerODEntry(OD_MOTOR_COMMAND_INDEX,     OD_MOTOR_COMMAND_SUBINDEX,      2, sizeof(od_motor_command),      &od_motor_command); 
  // --- Configure TPDOs and RPDOs ---

  configureTPDO(0, PDO_IVQ_COB_ID, 255, 100, 1000);  // TPDO 1, COB-ID, transType, inhibit, timer
  mapTPDO(0, PDO_IVQ_MAP, PDO_IVQ_COUNT); //TPDO 1, entries, num entries

  configureTPDO(1, PDO_PWR_RE_COB_ID, 255, 100, 1000);  // TPDO 2, COB-ID, transType, inhibit, timer
  mapTPDO(1, PDO_PWR_RE_MAP, PDO_PWR_RE_COUNT); //TPDO 2, entries, num entries

  configureTPDO(2, PDO_CE_COB_ID, 255, 100, 1000);  // TPDO 3
  mapTPDO(2, PDO_CE_MAP, PDO_CE_COUNT);

  configureRPDO(0, PDO_TS_COB_ID, 255, 0);
  mapRPDO(0, PDO_TS_MAP, PDO_TS_COUNT);

  configureRPDO(1, PDO_RGB_MTCM_COB_ID, 255, 0);
  mapRPDO(1, PDO_RGB_MTCM_MAP, PDO_RGB_MTCM_COUNT);

}

/**
 * @brief reads the bytes from the shunt and passes them into rxData which puts the name-value pairs in the buffer veData
 * @return nothing
 */

void ReadVEData() {
    while (veSerial.available()){
        veParser.rxData(veSerial.read()); 
    }
    yield(); 
}

/**
 * @brief prints values including current, voltage, SOC, instantenous power AND recovered energy to serial monitor every second
 * @return nothing
 */

void EverySecond() {
    static unsigned long prev_millis;
    if (millis() - prev_millis > 1000) {
        PrintData();
        Serial.print("Recovered Energy");
        Serial.print(" = ");
        Serial.println(recovered_energy);
        prev_millis = millis();
    }
}

/**
 * @brief prints values in the buffer including current, voltage, SOC, and instantenous power to serial monitor every second
 * @return nothing
 */
void PrintData() {
    for ( int i = 0; i < veParser.veEnd; i++ ) {
    Serial.print(veParser.veData[i].veName);
    Serial.print(" = ");
    Serial.println(veParser.veData[i].veValue);    
    }
}

/**
 * @brief computes recovered energy
 * @return nothing
 */
void findRecoveredEnergy() {
  if (power < 0) {
    prev_sample_2 = false; 
    power_sample = -power; 
    if (prev_sample_1 == false) {
      prev_power_sample = power_sample;
      prev_sample_1 = true;
      od_recovered_energy = recovered_energy;
      return;
    }
    new_power_sample = power_sample;
    slice_area = ((prev_power_sample + new_power_sample) / 2) * 1;
    recovered_energy += slice_area;
    od_cumulative_energy += slice_area;               // accumulate recovered energy
    prev_power_sample = new_power_sample;
  }
  else if ((power > 0) && (od_motor_command >= 50)) {
    prev_sample_1 = false;
    if (recovered_energy <= 0) {
      recovered_energy = 0;
      prev_sample_2 = false;
      od_recovered_energy = 0;
      return;
    }
    power_sample = power;
    if (prev_sample_2 == false) {
      prev_power_sample = power_sample;
      prev_sample_2 = true;
      od_recovered_energy = recovered_energy;
      return;
    }
    new_power_sample = power_sample;
    slice_area = ((prev_power_sample + new_power_sample) / 2) * 1;
    if (slice_area >= recovered_energy) {
      od_cumulative_energy -= recovered_energy;       // only subtract what was actually spent
      recovered_energy = 0;
      prev_sample_2 = false;
    } else {
      recovered_energy -= slice_area;
      od_cumulative_energy -= slice_area;             // accumulate spent energy
    }
    prev_power_sample = new_power_sample;
  }
  // int energy = 0;

  // if (power < 0){  
  //     if (od_motor_command > 80)){
  //         power_sample = power; 
  //         energy = power_sample;
  //     }
  // } else {
  //     power_sample = power; 
  //     energy = power_sample; 
  // }

  // recovered_energy = recovered_energy - energy;

  od_recovered_energy = (uint32_t)recovered_energy;
}
/**
 * @brief updates OD entries. OD entries are updated every second when new data is received into the buffer
 * @return nothing
 */

void updateODentries(){
    // Iterate through all name-value pairs in the buffer. Find voltage, current, SOC, power in the buffer and assign the values to the variables in the object dictionary of the node.
    for (int i = 0; i < veParser.veEnd; ++i) {
       if (strcmp(veParser.veData[i].veName, "V") == 0){
        od_voltage = atoi(veParser.veData[i].veValue);
       }
       else if (strcmp(veParser.veData[i].veName, "I") == 0){
        current = -atoi(veParser.veData[i].veValue);
        od_current = current; 
       }
       else if (strcmp(veParser.veData[i].veName, "SOC") == 0){
        od_soc = atoi(veParser.veData[i].veValue); // needs to be divided by 10 to get it in percentage. 995/10 = 99.5%.
       }
       else if (strcmp(veParser.veData[i].veName, "P") == 0){
        power = -atoi(veParser.veData[i].veValue);
        power = power - 80;
        od_power = power; 
       }
  }
  veParser.clearData();
}


void loop(){

  ReadVEData();  // read data from shunt and put into buffer
  EverySecond(); // Debug: print the data contained in the buffer every second.

  switch (nodeOperatingMode) {

    // In stopped mode, do nothing
    case MODE_STOPPED: break;

    // In pre-op mode, we check if the data is received from the shunt, parsed and stored in the buffer 
    case MODE_PREOP:
      if (veParser.isDataAvailable()) {
        updateODentries();  
        last_data_received_time = millis(); // if new data is received update the OD entries and record the time
      } else {
        // if new data is not received
        currentMillis = millis();
        if (currentMillis - last_data_received_time >= shunt_data_interval) { // if more than 1.5s pass and still no data in buffer then raise error because a new block should be received every sec.
          // sendEMCY(1,nodeID, 0x00000701); // Minor Emergency
          Serial.println("No Data in the buffer!");
          last_data_received_time = currentMillis;
        }
      }
      break;

    // in operational mode, 
    case MODE_OPERATIONAL:
      if (nodeOperatingMode == MODE_OPERATIONAL){ 
        if (veParser.isDataAvailable()) {
          updateODentries();
          printf("--- OD Entries ---\n");
          printf("current_can:          %ld mA\n",  (int32_t)od_current);
          printf("voltage:              %u mV\n",   od_voltage);
          printf("state_of_charge:      %.1f %%\n", od_soc / 10.0);
          printf("power_can:            %ld W\n",   (int32_t)od_power);
          printf("recovered_energy_can: %lu J\n",   od_recovered_energy);
          printf("cumulative_energy:    %d J\n",    od_cumulative_energy);
          printf("------------------\n");
        }
        if((millis() - prev_time) > 1000){
          prev_time = millis();
          findRecoveredEnergy();

        }
      }
      break;

    default: break;
  }
}