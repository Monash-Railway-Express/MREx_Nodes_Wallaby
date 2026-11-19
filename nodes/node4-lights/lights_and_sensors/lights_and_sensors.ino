/**
 * @file lights_and_sensors.ino
 * @brief This code is for the lights & sensor node (Node 4 on the loco). 
 *
 * @details
 * LIGHTS_FWD will be the control for the white lights on the front and red on the back,
 * LIGHTS_REV will be the control for the red lights on the front and white on the back
 * LIGHTS_PREOP control the yellow lights on all faces of the loc
 * TEMPER
 *
 * @author Aung Hpone Thant 
 * @author Chiara Gillam
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

#include <CAN_MREx.h> // inlcudes all CAN MREX files
#include <lights_n_sensors.h>


//misc variables
unsigned long next_poll_time; //used for non blocking delay to request the current motor direction from motor controller
uint32_t dir_mode32; // Will these eventually be ODs?
uint8_t drive_state = OFF;

//all of this is sensor stuff which I believe Oscar is putting on another board.
bool smokeLock = false;
bool heatFrontLock = false;
bool heatRearLock = false;
unsigned long errorSTime = 0;
unsigned long errorTFTime = 0;
unsigned long errorTRTime = 0;
unsigned long next_sensor_time = 0; 

// User code end ---------------------------------------------------------

/**
 * @brief Initial Set up of Object Dictionary & Pin communication
 */
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Serial Coms started at 115200 baud");
  
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
  enableHeartbeatMonitoring(true);


  // User code Setup Begin: -------------------------------------------------
  // --- Register OD entries ---
  registerODEntry(OD_TEMPERATURE_FRONT_INDEX, OD_TEMPERATURE_FRONT_SUBINDEX,  2, sizeof(od_temperature_front),  &od_temperature_front);
  registerODEntry(OD_TEMPERATURE_REAR_INDEX,  OD_TEMPERATURE_REAR_SUBINDEX,   2, sizeof(od_temperature_rear),   &od_temperature_rear);
  registerODEntry(OD_DIRECTION_MODE_INDEX,    OD_DIRECTION_MODE_SUBINDEX,     2, sizeof(od_direction_mode),     &od_direction_mode); 

  // --- Register TPDOs ---
  //TPDOs for sensors. Oscar is working on a separate board for this now, can ignore.
  // originally (COB-ID seems wrong and mismatched from driver controller RPDO?):
  // configureTPDO(0, 0x184 + NODE_ID, 255, 100, 100);
  // Using COB-ID 0x184 NOT 0x188:
  configureTPDO(0, PDO_TEMP_COB_ID, 255, 100, 100);
  mapTPDO(0, PDO_TEMP_MAP, PDO_TEMP_COUNT);
   

  // --- Register RPDOs ---


  // --- Set pin modes ---
  pinMode(LIGHT_PREOP, OUTPUT);
  pinMode(LIGHT_FWD, OUTPUT);
  pinMode(LIGHT_REV, OUTPUT);
  pinMode(SMOKE_PIN, INPUT);
  pinMode(TEMPERATURE_FRONT_PIN, INPUT);
  pinMode(TEMPERATURE_REAR_PIN, INPUT);


  //run lights self test. Flash all lights
  LightsSelfTest();

  //permanently turn yellow lights on
  digitalWrite(LIGHT_PREOP, HIGH); 

  // User code Setup end ------------------------------------------------------


}

/**
 * @brief Main loop of the program, checks the sensors each loop, assigns the drive state depending on mode
 */
void loop() {
  //User Code begin loop() ----------------------------------------------------
  unsigned long  currentMillis = millis();
  
  // Check the sensors no matter what mode we are in
  if (currentMillis >= next_sensor_time)
    {
      CheckSensors();
      next_sensor_time = currentMillis + 200;
    }


  // --- Stopped mode (This is default starting point) ---
  if (nodeOperatingMode == MODE_STOPPED){ 
    //handleCAN(NODE_ID);
    
    drive_state = OFF;
  }

  // --- Pre operational state (This is where you can do checks and make sure that everything is okay) ---
  if (nodeOperatingMode == MODE_PREOP){ 
    //handleCAN(NODE_ID);
    drive_state = PREOP;
  }

  // --- Operational state (Normal operating mode) ---
  if (nodeOperatingMode == MODE_OPERATIONAL){ 
    //handleCAN(NODE_ID);
    //request the state of the motor drive direction every 200ms
    if (currentMillis >= next_poll_time)
    {
      HandleDirStates();
      next_poll_time = currentMillis + 200;
    }

  }
  HandleOpMode();
  //User code end loop() --------------------------------------------------------
}

/**
 * @brief Handles the direction selector in operational state
 */
void HandleDirStates()
{
  //reads the motor direction from the controller.
  //dir_mode32 = executeSDORead(NODE_ID, 3, OD_DIRECTION_MODE_INDEX, OD_DIRECTION_MODE_SUBINDEX); 
  //od_direction_mode = 1;
  //executeSDOWrite(NODE_ID, 3, OD_DIRECTION_MODE_INDEX, OD_DIRECTION_MODE_SUBINDEX, sizeof(od_direction_mode), &od_direction_mode);
  //od_direction_mode = (uint8_t)dir_mode32;

  //switches the drive state based on the motor direction
  if(od_direction_mode == 2)
  {
    drive_state = NEUTRAL;
  }
  if(od_direction_mode == 3)
  {
    drive_state = FORWARD;
  }
  if(od_direction_mode == 1)
  {
    drive_state = REVERSE;
  }
}

/**
 * @brief Function that flashes all lights on power on. 
 */
void LightsSelfTest()
{
  digitalWrite(LIGHT_PREOP, LOW);
  digitalWrite(LIGHT_FWD, HIGH);
  digitalWrite(LIGHT_REV, LOW);
  delay(200);
  digitalWrite(LIGHT_PREOP, HIGH);
  digitalWrite(LIGHT_FWD, LOW);
  digitalWrite(LIGHT_REV, LOW);
  delay(200);
  digitalWrite(LIGHT_PREOP, LOW);
  digitalWrite(LIGHT_FWD, LOW);
  digitalWrite(LIGHT_REV, HIGH);
  delay(200);
  digitalWrite(LIGHT_PREOP, HIGH);
  digitalWrite(LIGHT_FWD, HIGH);
  digitalWrite(LIGHT_REV, HIGH);
  delay(500);
  digitalWrite(LIGHT_PREOP, LOW);
  digitalWrite(LIGHT_FWD, LOW);
  digitalWrite(LIGHT_REV, LOW);
}

/**
 * @brief Function that writes to the Light pins depending on the drive state (direction/operating mode) of the locomotive
 */
void HandleOpMode()
{
  switch (drive_state)
  {
    case FORWARD:
      digitalWrite(LIGHT_FWD, HIGH);
      digitalWrite(LIGHT_REV, LOW);
      digitalWrite(LIGHT_PREOP, HIGH);
      break;
    
    case REVERSE:
      digitalWrite(LIGHT_FWD, LOW);
      digitalWrite(LIGHT_REV, HIGH);
      digitalWrite(LIGHT_PREOP, HIGH);
      break;

    case PREOP:
      digitalWrite(LIGHT_FWD, LOW);
      digitalWrite(LIGHT_REV, LOW);
      digitalWrite(LIGHT_PREOP, HIGH);
      break;

    case NEUTRAL:
    digitalWrite(LIGHT_FWD, HIGH);
    digitalWrite(LIGHT_REV, HIGH);
    digitalWrite(LIGHT_PREOP, HIGH);
      break;

    case OFF:
      digitalWrite(LIGHT_FWD, LOW);
      digitalWrite(LIGHT_REV, LOW);
      digitalWrite(LIGHT_PREOP, HIGH);
      break;
  }
}


/**
 * @brief Function for Checking the Temperature and Air Quality of the Sensors. Assumes we are using the digital Output of the Smoke Detector
 * Smoke & Temperature Sensors will output EMCY onto CANBus when their respective sensors have triggered for more than 5 seconds
 * EMCY's will continue to be sent every 5 seconds if the sensors are still triggered to not flood the canbus.
 */
void CheckSensors(){

  bool smokeEMCY = false;
  bool heatFrontEMCY = false;
  bool heatRearEMCY = false;

  int32_t temperatureFront;
  int32_t temperatureRear;
  int32_t allowableTemperature = 70;


  // Sensor is low when Gas is detected
  smokeEMCY = (digitalRead(SMOKE_PIN) == LOW); 
  
  // Between 0 - 4095
  temperatureFront = analogRead(TEMPERATURE_FRONT_PIN)*10;
  temperatureRear = analogRead(TEMPERATURE_REAR_PIN)*10;

  /*
  - Turn the thermistor reading into a celsius temperature (will need callibration)
  - Use only integers
  - Assuming MCP970X Thermistor
    Vout = Tc * Ta + V0c
    

    V0c: 400mV / 500mV

    Tc: 10.0 / 19.5      Depending on model

    Ta: The Temperature (C)
  */
  
  int voltage0 = 6204; // 500mv Converted to 0-40950 Scale
  int temperature_Coef = 124; // 10mV Converted to 0-40950 Scale

  temperatureFront = ( temperatureFront - voltage0 ) / temperature_Coef;

  temperatureRear = ( temperatureRear - voltage0 ) / temperature_Coef;

  od_temperature_front = temperatureFront;
  od_temperature_rear = temperatureRear;


  //Debugging

  Serial.println("Temperature:");
  Serial.print("  Rear: ");
  Serial.println(temperatureRear);
  Serial.print("  Front: ");
  Serial.println(temperatureFront);

  
  heatFrontEMCY = temperatureFront > allowableTemperature;
  heatRearEMCY = temperatureRear > allowableTemperature;
  Serial.println();
  
  if (smokeEMCY){

    if(smokeLock == false){
      errorSTime = millis();
      smokeLock = true;
    }

    unsigned long currentTime = millis();

    if (currentTime - errorSTime >= 5000){
      Serial.println("Smoke Detected in the Locomotive!");
      sendEMCY(0, NODE_ID, 0x00505);
      smokeLock = false;
    }
  }
  else{
    smokeLock = false;
  }

  if (heatFrontEMCY) {
    
    if(heatFrontLock == false){
      errorTFTime = millis();
      heatFrontLock = true;
    }

    unsigned long currentTime = millis();

    if (currentTime - errorTFTime >= 5000){
      Serial.println("Temperature inside Locomotive Front is Too High!");
      sendEMCY(0, NODE_ID, 0x00506);
      heatFrontLock = false;
    }
  }
  else{
    heatFrontLock = false;
  }

  if (heatRearEMCY) {

    if(heatRearLock == false){
      errorTRTime = millis();
      heatRearLock = true;
    }

    unsigned long currentTime = millis();

    if (currentTime - errorTRTime >= 5000){
      Serial.println("Tempetaure inside Locomotive Rear is Too High!");
      sendEMCY(0, NODE_ID, 0x00507);
      heatRearLock = false;
    }
  }
  else{
    heatRearLock = false;
  }

}
