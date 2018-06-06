// ---------------------------------------------------------------------------
// Created by Daniel Polanco - Jdanypa@gmail.com
//
// See "QURHexapod.h" syntax, methods index, version history, links, and more.
// ---------------------------------------------------------------------------

#include "QURHexapod.h"
#include <Servo.h> 
#include <Arduino.h>
#include <RF24.h>

RF24 RFController(RF_CE, RF_CSN);  // RFController: Instance of the class RF24 to control the antenna.
byte addresses[][6] = {"0"};       // Addresses: Address of comunication

// ---------------------------------------------------------------------------
// DEBUG Functions
// Methods:
//      - DEBUGER: Function that print data to the PC to see how is working 
//                 the code.
//      - STOP: Function that stop the code in a specific moment.
// ---------------------------------------------------------------------------

// Function that recives a Argument String and if DEBUG is enabled print it into the Serial PORT
void DEBUGER(String __INFO__){
  #ifndef DEBUG == true
    PCSerial.println("DEBUG: " + String(__INFO__));
  #endif
}

// ---------------------------------------------------------------------------
// QURHexapod constructor
// This constructor read 2 vector bidimencional and move them i+pñ*pto the MAX and MIN angles per Servo
// ---------------------------------------------------------------------------
QURHexapod::QURHexapod(int __LIMITx__[__SERVOS__][__LEGS__], int __LIMITy__[__SERVOS__][__LEGS__]){
  // This cicle go over any leg, with the iterator 'x'
  for(int x = 0; x < __LEGS__; x++){
    // Go to 'ServoDriver' -> 'Legs' and call the function SetMinMaxAngles and put in the data.
    ServoDriver.Legs[x].SetMinMaxAngles(__LIMITx__[0][x], __LIMITx__[1][x], 0);
    ServoDriver.Legs[x].SetMinMaxAngles(__LIMITy__[0][x], __LIMITy__[1][x], 1);
    // Go to 'ServoDriver' -> 'Legs' -> 'Joint' and Sets the AXIS per Servo
    ServoDriver.Legs[x].Joint[ANGLE_X].AXIS = ANGLE_X;
    ServoDriver.Legs[x].Joint[ANGLE_Y].AXIS = ANGLE_Y;
  }
}

// ---------------------------------------------------------------------------
// Methods for Legs control
//    - SetMinMaxAngles(int __MIN__, int __MAX__, int __SELECTOR__)
//    - GoToSetpoint()
// ---------------------------------------------------------------------------

/**
  @Struct QURHexapod -> SERVO_DRIVER -> Leg
  @Function SetMinMaxAngles
  @purpuse Function that allows to configure the Maximum and Minimum angle for the servo in especific Axis

  @param __MIN__ Angle limit minimun for the servo
  @param __MAX__ Angle limit minimun for the servo
*/
void QURHexapod::Leg::SetMinMaxAngles(int __MIN__, int __MAX__, int __SELECTOR__){
  Joint[__SELECTOR__].MAX_Angle = __MAX__;    // Set Servo Max limit to the argument __MAX__
  Joint[__SELECTOR__].MIN_Angle = __MIN__;    // Set Servo Min limit to the argument __MIN__
}

/**
  @Struct QURHexapod -> SERVO_DRIVER -> Leg
  @Function GoToSetpoint
  @purpuse Function to move the servo a STEP to the current setpoint
*/
void QURHexapod::Leg::GoToSetpoint(){
  for(int x = 0; x < __SERVOS__; x++){                    // Cicle pointer 'x' to go over each leg
    if(Joint[x].Estado != Joint[x].Setpoint){             // Check if the servo angle is equal to the setpoint
      Joint[x].Is_Finished = false;                       // Flag 'Is_Finished' set False
      Joint[x].Estado += (Joint[x].Estado < Joint[x].Setpoint ? STEP_SERVO : -STEP_SERVO); // Sets the STEP for the angle
      Joint[x].Control.write(Joint[x].Estado);            // Write the angle updated to the Servo
    }
    else{                                                   // If servo angle is equal to the setpoint
       Joint[x].Is_Finished = true;                         // Flag 'Is_Finished' set true
    }
  }
}

// ---------------------------------------------------------------------------
// Methods for Servos control
//      - ConfigPinServo(int __SELECTOR__)
// ---------------------------------------------------------------------------

/**
 @Struct QURHexapod -> SERVO_DRIVER -> Leg -> Joints
 @Function ConfigPinServo
 @purpuse Read a value int and sets the ID and PIN for the Servo

 @param __SELECTOR__ Is the new ID value for the servo
*/
void QURHexapod::Joints::ConfigPinServo(int __SELECTOR__){
  ID = __SELECTOR__;                          // ID is equal to the __SELECTOR__
  Control.attach(PIN_AVAILABLES[AXIS][ID]);   // Sets the new pin for servo
  DEBUGER(" ID: " + String(ID) + " / PIN: " + String(PIN_AVAILABLES[AXIS][ID]));  // DEBUG of the data
}

// ---------------------------------------------------------------------------
// Methods for Robot control
//       - ProcessFinished()
//       - BackgroundProcess()
//       - UpdateSetpoints(int AnglesX[__LEGS__], int AnglesY[__LEGS__])
// ---------------------------------------------------------------------------

/**
  @Struct QURHexapod -> SERVO_DRIVER
  @Function ProcessFinished
  @purpuse Check if all servos finished

  @return Returns true if all Servos are in their place or false if not.
*/
bool QURHexapod::SERVO_DRIVER::ProcessFinished(){
  for(int x = 0; x < __LEGS__; x++){          // Cicle with a iterator 'x' that go over each LEG
    for(int y = 0; y < __SERVOS__; y++){    // Cicle with a iterator 'y' that go over each SERVO
      if(!Legs[x].Joint[y].Is_Finished){  // If this servo<x, y> has not finished
        return false;                   // Broke the Funtion and return false
                        // (Because still one servo that has not finished)
      }
    }
  }
  return true;                                // If all Servos has finished return true
}

/**
  @Struct QURHexapod -> SERVO_DRIVER
  @Function BackgroundProcess
  @purpuse Funtion that go over all servos and move them to their Setpoints
*/
void QURHexapod::SERVO_DRIVER::BackgroundProcess(){
  for(int x = 0; x < __LEGS__; x++){          // Cicle with a iterator 'x' that go over each LEG
    Legs[x].GoToSetpoint();                 // Call function from LEG->'GoToSetpoint' to move the leg
    delay(TIMEOUT_LEG);                     // Wait the movement from the SERVO and the delay of speed
  }
}

/**
  @Struct QURHexapod -> SERVO_DRIVER
  @Function UpdateSetpoints
  @purpuse Function that update the Setpoint of all servos.

  @param AnglesX Vector that contains the values for the Setpoints in AXIS X
  @param AnglesY Vector that contains the values for the Setpoints in AXIS Y
*/
void QURHexapod::SERVO_DRIVER::UpdateSetpoints(int AnglesX[__LEGS__], int AnglesY[__LEGS__]){
  for(int ptr = 0; ptr < __LEGS__; ptr++){     // Cicle with a iterator 'ptr' that go over each LEG
    // This funcions works converting the Setpoints from a Dimention Linear(0 - 100) into a Dimention of angles(LIMIT_MIN, LIMIT_MAX)
    // After the convertion saves it into the Servo.
    //      Definition map function:
    //          - map -> Function that converts a value from a specific range into a other value in other range
    //      Example:
    //          - map(Value to converto, Initial range MIN, Initial range MAX, To range MIN, To range MAX);
    Legs[ptr].Joint[ANGLE_X].Setpoint = map(AnglesX[ptr], 0, 100, Legs[ptr].Joint[ANGLE_X].MIN_Angle, Legs[ptr].Joint[ANGLE_X].MAX_Angle);
    Legs[ptr].Joint[ANGLE_Y].Setpoint = map(AnglesY[ptr], 0, 100, Legs[ptr].Joint[ANGLE_X].MIN_Angle, Legs[ptr].Joint[ANGLE_X].MAX_Angle);
    DEBUGER(" Setpoint X -> " + String(Legs[ptr].Joint[ANGLE_X].Setpoint));   // DEBUG of Data
    DEBUGER(" Setpoint Y -> " + String(Legs[ptr].Joint[ANGLE_Y].Setpoint));   // DEBUG of Data
  }
}

// ---------------------------------------------------------------------------
// Methods for Timers controller
//       - BackgroundTime()
//       - SetTimer(int __FINAL__)
// ---------------------------------------------------------------------------

/**
  @Struct QURHexapod -> TIMES
  @Function BackgroundTime
  @purpuse Checks if the timer ended

  @return Return a boolean value if the timer is not ended (true if is ended or false if not)
*/
bool QURHexapod::TIMES::BackgroundTime(){
  // If (<Current Time> less <Initial Time> is less than <Final time>) return true, else return false
  return ((millis() - TimeInitial) < TimeFinal);
}

/**
  @Struct QURHexapod -> TIMES
  @Function SetTimer
  @purpuse Read a int argument that is the end time, and initialize the counter.

  @param __FINAL__ Variable that indicates the term time
*/
void QURHexapod::TIMES::SetTimer(int __FINAL__){
  TimeInitial = millis();     // Sets the <Initial Time> to NOW
  TimeFinal   = __FINAL__;    // Sets the Final Time to the argument __FINAL__
}

// ---------------------------------------------------------------------------
// RF DRIVER Methods
//       - Start()
//       - ReadData()
// ---------------------------------------------------------------------------

/**
  @Struct QURHexapod -> RF_DRIVER
  @Function Start
  @purpuse Initializes the RFController and configure it
*/
void QURHexapod::RF_DRIVER::Start(){
  RFController.begin();                           // Initialize the antenna
  RFController.setChannel(115);                   // Set the channel into 115
  RFController.setPALevel(RF24_PA_MAX);           // Set the Level into Maximum
  RFController.setDataRate( RF24_250KBPS );       // Set the speed of reading
  RFController.openReadingPipe(1, addresses[0]);  // Set the Address of comunication
  RFController.startListening();                  // Start into lisent data.
}

/**
  @Struct QURHexapod -> RF_DRIVER
  @Function ReadData
  @purpuse Read the data from the RFController
*/
void QURHexapod::RF_DRIVER::ReadData(){
  if (RFController.available()){                                      // If the Antenna is ON
    TIMES Timer;                                                    // Instance a Timer from the Struct TIMES
    Timer.SetTimer(TIMEOUT_RF);                                     // Set the Timer into the Limit of TIMEOUT for the Antenna
    while (RFController.available() && !Timer.BackgroundTime()){    // While the RF is available and the timer has not ended
      RFController.read( &Data, sizeof(Data));                    // Read the data and save they in the Struct Data
    }
  }
}

// ---------------------------------------------------------------------------
// Hexapaod Methods
//       - Routine()
//       - SelectMode(bool __MODE__)
//       - SetAnglesLeg(int __SETPOINTS__[], bool __SERVO__)
//       - SetAngleServo(int __SETPOINT__, int __LEG__, bool __SERVO__)
//       - ServosFinished()
// ---------------------------------------------------------------------------

/**
  @Struct QURHexapod
  @Function Routine
  @purpuse Function Main and manage synchronize the RF to ROBOT, Additionally 
       control the servos and their SetPoints
*/
void QURHexapod::Routine(){
  if(!ServoDriver.ManualMode){                    // If Robot is not in MANUAL
    Timer.SetTimer(TIMEOUT_RF);                 // Initialize the Time rAfor read data from RF
    while(!Timer.BackgroundTime()){             // While the timer is not ended
      RFdriver.ReadData();                    // Read data from the RFController
    }   
  }
  ServoDriver.UpdateSetpoints(AnglesX, AnglesY);  // Call the function 'UpdateSetpoints' and update the setpoints
  All_Finished = ServoDriver.ProcessFinished();   // Updates the state of the servos to check if they has finished
  if(!All_Finished){                              // If the servos has not finished
    ServoDriver.BackgroundProcess();            // Do the funtion 'BackgroundProcess' to move the servos
  }
}

/**
  @Struct QURHexapod
  @Function SelectMode
  @purpuse Change the current mode

  @param __MODE__ Boolean selector to change the current mode
*/
void QURHexapod::SelectMode(bool __MODE__){
  ServoDriver.ManualMode = __MODE__;        // __MODE__ is False Robot is AUTOMATIC if is true Robot is MANUAL
}

/**
  @Struct QURHexapod
  @Function SetAnglesLeg
  @purpuse Allows change the current setpoint

  @param __SETPOINTS__  Vector that contais the new Setpoints
  @param __SERVO__      Boolean selector for selection the AXIS (true = AXIS X, false = AXIS Y)
*/
void QURHexapod::SetAnglesLeg(int __SETPOINTS__[], bool __SERVO__){
  for(int x = 0; x < 6; x++){             // Cicle with iterator 'x' for go over each leg
    if(__SERVO__)                       // If boolean selector is true, AXIS X enables
      AnglesX[x] = __SETPOINTS__[x];  // Save the new SETPOINT int AXIS X
    else                                // Else is false so AXIS Y enables
      AnglesY[x] = __SETPOINTS__[x];  // Save the new SETPOINT in AXIS Y
  }
}

/**
  @Struct QURHexapod
  @Function SetAngleServo
  @purpuse Allows change the current setpoint for a specific servo

  @param __SETPOINT__  Int that contains the new Setpoint for the servo
  @param __LEG__       Int selector to select the LEG (from 0 to <MAX_LEGS>)
  @param __SERVO__     Boolean selector to select the AXIS (true = AXIS X, false = AXIS Y)
*/
void QURHexapod::SetAngleServo(int __SETPOINT__, int __LEG__, bool __SERVO__){
  if(__SERVO__)                           // If boolean selector is true, AXIS X enables
    AnglesX[__LEG__] = __SETPOINT__;    // Save the new SETPOINT int AXIS X
  else                                    // Else is false so AXIS Y enables
    AnglesY[__LEG__] = __SETPOINT__;    // Save the new SETPOINT in AXIS Y
}

/**
  @Struct QURHexapod
  @Function ServosFinished
  @purpuse Check if all servos finished

  @return Returns true if all Servos are in their place or false if not.
*/
bool QURHexapod::ServosFinished(){
  return All_Finished;    // Return the state of the FLAG 'All_Finished'
}

void QURHexapod::Debug(bool __ALL__, int __SERVO__, int __LEG__){
  int ID   = ServoDriver.Legs[__LEG__].Joint[__SERVO__].ID;
  int AXIS = ServoDriver.Legs[__LEG__].Joint[__SERVO__].AXIS;
  int pin  = ServoDriver.Legs[__LEG__].Joint[__SERVO__].PIN_AVAILABLES[AXIS][ID];
  int i = __ALL__ == true ? ServoDriver.Legs[__LEG__].Joint[__SERVO__].MIN_Angle : 0;
  for (; i < (__ALL__ == true ? ServoDriver.Legs[__LEG__].Joint[__SERVO__].MAX_Angle : 180); ++i)
  {
    ServoDriver.Legs[__LEG__].Joint[__SERVO__].Control.write(i);
    delay(100);
  }
}