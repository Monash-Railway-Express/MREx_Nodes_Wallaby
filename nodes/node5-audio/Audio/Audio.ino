/**
 * @file Audio.ino
 * @brief CAN MREX Audio Control file 
 *
 * @details 
 * Audio controller for horn and announcements. Uses HardwareSerial to interface
 * between ESP32 and DFPlayerMini via UART. Uses DFRobotDFPlayerMini.h
 * 
 * @author Aung Hpone Thant
 * @author Nhan Nguyen
 *
 * @date 19/10/2026
 *
 * @organisation MREx
 *
 */

/**
NOTE: invert horn signal before use PLEASE!!!!!
*/
 /***************************************************
DFPlayer - A Mini MP3 Player For Arduino
 <https://www.dfrobot.com/product-1121.html>
 
 ***************************************************
 DFPlayer
 Created 2016-12-07
 By [Angelo qiao](Angelo.qiao@dfrobot.com)
 
 GNU Lesser General Public License.
 See <http://www.gnu.org/licenses/> for details.
 All above must be included in any redistribution
 ****************************************************/

/***********Notice and Trouble shooting***************
 1.Connection and Diagram can be found here
 <https://www.dfrobot.com/wiki/index.php/DFPlayer_Mini_SKU:DFR0299#Connection_Diagram>
 ****************************************************/

/*
USE 16 bit .wav files for horn. Using MP3 adds small silences at start and end of clip, making looping sound choppy.
*/

#include "../../../shared/MREx.h"
uint8_t NODE_ID = AUDIO_ID;
#include <CAN_MREx.h> // inlcudes all CAN MREX files
#include <Arduino.h>
#include <DFRobotDFPlayerMini.h>
#include <HardwareSerial.h>
#include <map>



// User code begin: ------------------------------------------------------
// --- CAN MREx initialisation ---
// --- Pin Definitions ---
#define TX_GPIO_NUM GPIO_NUM_36 // Set GPIO pin for CAN Transmit
#define RX_GPIO_NUM GPIO_NUM_35 // Set GPIO pins for CAN Receive

#define STATUS_LED 5 
#define PREOP_LED 4

//DFPlayer Setup
#define DFBUSY 13 //used to check if DFPlayer is playing tracks or not. Connect to BUSY pin on DFPlayer. If High, its free.

/*
Sound index definition
Note: These are the sounds in the order that they are copied to the SD card.
The DFPlayer indexes songs in order that they were copied into the SD Card. These values reflect that.
*/
#define STARTUP_FB_SOUND 1
#define HORN_LOOP_SOUND 2
#define HORN_END_SOUND 3
//location announcement sound numbers
#define LOC_ANN1_START_SOUND 4
#define LOC_ANN2_AUTOSTOP_SOUND 5
#define LOC_ANN3_COMFORT_SOUND 6
#define LOC_ANN4_COMFORT_END_SOUND 7
#define LOC_ANN5_HAVEN_SOUND 8
#define LOC_ANN6_TCN_SOUND 9
#define LOC_ANN7_TCN_END_SOUND 10
#define LOC_ANN8_END_SOUND 11

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


/*
Location announcement data. 
First number is the "location counter" sent from autostop node
Second number is the index for the mp3 module for the associated announcement
*/
std::map<uint8_t, int> locationAnnounceSoundMap = { {LOC_ANN1_START,      LOC_ANN1_START_SOUND},
                                                    {LOC_ANN2_AUTOSTOP,   LOC_ANN2_AUTOSTOP_SOUND},
                                                    {LOC_ANN3_COMFORT,    LOC_ANN3_COMFORT_SOUND},
                                                    {LOC_ANN4_COMFORT_END,LOC_ANN4_COMFORT_END_SOUND},
                                                    {LOC_ANN5_HAVEN,      LOC_ANN5_HAVEN_SOUND},
                                                    {LOC_ANN6_TCN,        LOC_ANN6_TCN_SOUND},
                                                    {LOC_ANN7_TCN_END,    LOC_ANN7_TCN_END_SOUND},
                                                    {LOC_ANN8_END,        LOC_ANN8_END_SOUND}};

HardwareSerial player_serial(1); //initialise UART 1 of ESP as the serial for the DFPlayer module
DFRobotDFPlayerMini myDFPlayer;


//user variables
//function prototypes
void printDetail(uint8_t type, int value);
void StoppedMode();
void PreOpMode();
void OperationalMode();
void HandleHorn();


// User code end ---------------------------------------------------------


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
  // User code Setup Begin: -------------------------------------------------
  // --- Register OD entries ---
  registerODEntry(OD_HORN_TOGGLE_INDEX, OD_HORN_TOGGLE_SUBINDEX, 1, sizeof(od_horn_toggle), &od_horn_toggle);
  registerODEntry(OD_LOCATION_COUNTER_INDEX, OD_LOCATION_COUNTER_SUBINDEX, 1, sizeof(od_location_counter), &od_location_counter);
  // --- Register TPDOs ---
  

  // --- Register RPDOs ---


  // --- DFPlayer Setup ---
  player_serial.begin(9600, SERIAL_8N1, /*rx =*/18, /*tx =*/17); //this is the UART port to communicate with DFPlayer module. 
  Serial.println();
  Serial.println(F("Initializing DFPlayer ... (May take 3~5 seconds)"));
  
  if (!myDFPlayer.begin(player_serial, /*isACK = */true, /*doReset = */true)) {  //Use serial to communicate with mp3.
    Serial.println(F("Unable to begin:"));
    Serial.println(F("1.Please recheck the connection!"));
    Serial.println(F("2.Please insert the SD card!"));
    sendEMCY(0x01,5,0x00000500); //send minor EMCY message when SD card read failed on startup
    while(true){
      delay(0); // Code to compatible with ESP8266 watch dog.
    }
  }
  delay(500);
  Serial.println(F("Audio system online."));
  myDFPlayer.volume(30);  //Set volume value. From 0 to 30
  delay(500);
  myDFPlayer.play(STARTUP_FB_SOUND); 
  // --- Set pin modes ---
  pinMode(DFBUSY, INPUT);
  pinMode(STATUS_LED, OUTPUT);
  pinMode(PREOP_LED, OUTPUT);
  digitalWrite(STATUS_LED, HIGH);
  // User code Setup end ------------------------------------------------------


}


void loop() {
  //User Code begin loop() ----------------------------------------------------
  // completing operation mode functionality
    switch (nodeOperatingMode) {
        case MODE_STOPPED:     StoppedMode();     break;
        case MODE_PREOP:       PreOpMode();       break;
        case MODE_OPERATIONAL: OperationalMode(); break;
        default:               StoppedMode();     break;  // fail-safe
    }

  //User code end loop() --------------------------------------------------------
}

void StoppedMode(){
  digitalWrite(STATUS_LED, LOW);
  digitalWrite(PREOP_LED, LOW);
}

void PreOpMode(){
  digitalWrite(STATUS_LED, LOW);
  digitalWrite(PREOP_LED, HIGH);
  HandleHorn();
}

void OperationalMode(){
  digitalWrite(STATUS_LED, HIGH);
  digitalWrite(PREOP_LED, LOW);
  HandleLocation();
  HandleHorn();
  Serial.println("Loop run");
}
/*
* @brief Handles logic for playing the horn. This used to have support to play a "start" "middle" and "end" sound to make
* the horn sound more real, but this has entirely been scrapped for a single looping sound because the ESP kept crashing.
* it was tested just before train was loaded and seems to be working fine, hence I (AP) will not touch it to make it cleaner in case
* I break it again.
*
* @return Nothing
*/
void HandleHorn() {
  static bool hornStatePrev; //previous state of the horn button. Used for edge detection.
  static uint8_t hornStateMNo = 1; //the counter value for the horn state machine. 
  hornStatePrev = (bool)od_horn_toggle;
  //Serial.print("Horn state:");
  //Serial.println(hornStatePrev);

  switch (hornStateMNo)
  {
    //State 1: Horn Off
    case 1:
      //1--->2
      if(hornStatePrev)
      {
        //play the looping segment of the horn
        myDFPlayer.loop(HORN_LOOP_SOUND);
        Serial.println("Horn looping");
        hornStateMNo = 2;
      }
      
      break;

    //State 2: Horn Started
    case 2:

      //2--->1
      if(!hornStatePrev)
      {
        myDFPlayer.stop();
        Serial.println("Horn end");
        hornStateMNo = 1;
      }
      
      break;
  } 

}

/*
* @brief Detects change to autostop counter. Every change, play a sound related to current marker
*
* @return Nothing
*/
void HandleLocation(){
  static uint8_t prevCounter = 0; //state variable used for tracking changes to the autostop counter


  //play sound every time location counter updates
  if(prevCounter != od_location_counter){
    prevCounter = od_location_counter;
    myDFPlayer.play(locationAnnounceSoundMap[od_location_counter]);  //retrieves the appropriate track number from the map. 
    Serial.print("Played location");
    Serial.println(od_location_counter);
  }

}

/*
* @brief Receives status messages from DFPlayer and prints them to console. Add link to CAN error messages further down the line.
*
* @return Nothing
*/
// TODO(AP): Add links to CAN MREx error logging (send minor errors on certain messages)
void printDetail(uint8_t type, int value){
  switch (type) {
    case TimeOut:
      Serial.println(F("Time Out!"));
      break;
    case WrongStack:
      Serial.println(F("Stack Wrong!"));
      break;
    case DFPlayerCardInserted:
      Serial.println(F("Card Inserted!"));
      break;
    case DFPlayerCardRemoved:
      Serial.println(F("Card Removed!"));
      break;
    case DFPlayerCardOnline:
      Serial.println(F("Card Online!"));
      break;
    case DFPlayerUSBInserted:
      Serial.println("USB Inserted!");
      break;
    case DFPlayerUSBRemoved:
      Serial.println("USB Removed!");
      break;
    case DFPlayerPlayFinished:
      Serial.print(F("Number:"));
      Serial.print(value);
      Serial.println(F(" Play Finished!"));
      break;
    case DFPlayerError:
      Serial.print(F("DFPlayerError:"));
      switch (value) {
        case Busy:
          Serial.println(F("Card not found"));
          break;
        case Sleeping:
          Serial.println(F("Sleeping"));
          break;
        case SerialWrongStack:
          Serial.println(F("Get Wrong Stack"));
          break;
        case CheckSumNotMatch:
          Serial.println(F("Check Sum Not Match"));
          break;
        case FileIndexOut:
          Serial.println(F("File Index Out of Bound"));
          break;
        case FileMismatch:
          Serial.println(F("Cannot Find File"));
          break;
        case Advertise:
          Serial.println(F("In Advertise"));
          break;
        default:
          break;
      }
      break;
    default:
      break;
  }
  
}