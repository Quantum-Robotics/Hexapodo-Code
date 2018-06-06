// ---------------------------------------------------------------------------
// Hexapod Quantum robotics Library - v1.0 - 06/20/2018
//
// AUTHOR:
// Created by Daniel Polanco - Jdanypa@gmail.com
//
// LINKS:
// GitHub Project: https://github.com/Elemeants/Hexapod-QuantumRobotics
//
// BACKGROUND:
// This library was created to control a Hexapod 2DOF, this project is part of my titling so I saw myself in 
// need to create a code to be able to control the robot from a RF-Controler, this library is Open-source so 
// you can change, add or copy anything you need to use it, everything is documented so that you understand
// all the code. If you need help with something you can send me an email to jdanypa@gmail.com.
//
// FEATURES:
// * This code is made for  Arduino Due or Mega.
// * Is able to control 12 servos at the same time.
// * Mode automatic: This mode moves servos automatic from the data readed from the RF-Control
// * Mode Manual: This mode allows you to operate the servos from the console
//
// CONSTRUCTOR:
//   QURHexapod Hexapod(Limit_Angles_x, Limit_Angles_x )
//     Limit_Angles_x & Limit_Angles_y - Vector to specify the limit angles for the servos in X and Y.
//
// METHODS:
//   Robot.Routine() - Is the routine that the robots follows (Read data from RF(Automatic) or Read data from Vector(Manual), and after moves the servos to the setPoints)
//   Robot.SelectMode(_MODE) - This change the mode to mode Manual or Automatic(MODE_ is true -> Automatic, _MODE_ is false -> Manual)
//   Robot.SetAnglesLeg(_SETPOINTS[], __SERVO)   - Sets the setpoints for the Servos in X or Y from the vector "SETPOINTS_"
//   Robot.SetAngle(_SETPOINT, __LEG, __SERVO) - Sets the setpoint to a specific LEG("LEG" a value from 0 to the number of Legs) and SERVO("SERVO_" -> true is X and false is Y). 
//   Robot.ServosFinished() - Returns a value true if the servos are in the setpoints or false if they are not
//
// HISTORY:
// 06/20/2018 v1.0 - Initial release.
// ---------------------------------------------------------------------------

#ifndef QURHEXAPOD_H
#define QURHEXAPOD_H

// ---------------------------------------------------------------------------
// Required libraries for Hexapod Control 
// ---------------------------------------------------------------------------
#include <Servo.h> 
#include <RF24.h>
#include <Arduino.h>

// ---------------------------------------------------------------------------
// COMUNICATION DEFINE'S
// ---------------------------------------------------------------------------

// Defined the port to comunicate to the plataform for Debugging or Interpretation
#define PCSerial Serial
// Define the baudrate for the Serial's PORTS
#define BAUDRATE 115200
#ifndef BAUDRATE
  #define BAUDRATE 115200
#endif
// Define MODULESD works if you are using a Arduino Due or Mega and you connect a Module for controlling
// a microSD this MODULE Works like Master -> Slave to a another Atmega328p conected to a MicroSD.
// if you are'nt using the ModuleSD set this.
#define MODULESD false
#ifdef MODULESD == true
  #define SDcontrol Serial1
#endif
// If is'nt defined DEBUG set DEBUG to False (NOT DEBUGGING)
// DEBUG Allows to the Hexapod to send data about the things is he doing to the PC from the Serial Port
#ifndef DEBUG == true
  #define DEBUG false
#endif

// ---------------------------------------------------------------------------
// SERVO DRIVER DEFINE'S
// ---------------------------------------------------------------------------
#define __SERVOS__    2     // Number of Servos per Leg
#define __LEGS__      6     // Number of Legs in the Robot
#define ANGLE_X       0     // Selector for the Angle X
#define ANGLE_Y       1     // Selector for the Angle Y
#define STEP_SERVO    1     // STEPs for the Servo motion 

// ---------------------------------------------------------------------------
// TIMING DEFINE'S
// ---------------------------------------------------------------------------
#define TIMEOUT_RF    50    // Maximum microseconds to read data from RF RF-Controller
#define TIMEOUT_LEG   10    // Maximum microseconds for the movement of the legs

// ---------------------------------------------------------------------------
// AVAILABLES MODES DEFINE'S
// ---------------------------------------------------------------------------
#define MANUAL    true      // Define the Macro for Manual Mode
#define AUTOMATIC false     // Define the Macro for Automatic Mode

// ---------------------------------------------------------------------------
// SELECTORS FOR SERVOS AND LEGS DEFINE'S
// ---------------------------------------------------------------------------
#define SERVO_X true        // Selector to Servo in Axis X
#define SERVO_Y false       // Selector to Servo in Axis Y
#define LEG_X   true        // Selector to Leg in Axis X
#define LEG_Y   false       // Selector to Leg in Axis Y

// ---------------------------------------------------------------------------
// RF PROTOCOL DEFINE'S
// ---------------------------------------------------------------------------
#define RF_CSN 49
#define RF_CE  48

#define RGB_RED   A4
#define RGB_GREEN A5
#define RGB_SET_ERROR()    analogWrite(RGB_GREEN, 0); analogWrite(RGB_RED, 255);
#define RGB_SET_OK()       analogWrite(RGB_GREEN, 255); analogWrite(RGB_RED, 0);
#define RGB_SET_WAITING()  analogWrite(RGB_GREEN, 127); analogWrite(RGB_RED, 127);  
#define RGB_SET_OFF()      analogWrite(RGB_GREEN, 0); analogWrite(RGB_RED, 0);
#define RGB_ANIMATION(x)   if (x == 0) {RGB_SET_OK();} if (x == 1) {RGB_SET_ERROR();} if (x == 2) {RGB_SET_WAITING();} delay(50); RGB_SET_OFF(); delay(50);  
#define RGB_OK      0
#define RGB_ERROR   1
#define RGB_WAIT    2

// ---------------------------------------------------------------------------
// HEXAPOD PRINCIPAL CLASS
// ---------------------------------------------------------------------------
class QURHexapod
{
  // ---------------------------------------------------------------------------
  // STRUCT FOR SERVOS
  // Contains the variables thats allows to control every servo.
  // Methods:
  //      - void ConfigPinServo(int)
  // ---------------------------------------------------------------------------
  typedef struct Joints
  {   
    // Variable vector, contains the pins availables for the Servos
    const int PIN_AVAILABLES[2][6] = {
      {2, 3, 4, 5, 6, 7},
      {8, 9, 10, 11, 12, 13}
    };
    int ID = 0;                 // ID: This declares what ID have the servo, this help to the algorithm
    int Setpoint = 50;           // Setpoint: Is the Angle which the servo is going
    int MAX_Angle = 0;          // MAX_Angles: Is the Maximum Angles available for the servo
    int MIN_Angle = 0;          // MIN_Angles: Is the Minimum Angles available for the servo
    int Estado = 0;             // Estado:: Is the actual Angle of the servo.
    int AXIS = 0;               // AXIS: Variable selector between Axis X and Y. (AXIS == 0 is X and AXIS == 1 is Y)
    bool Is_Finished = false;   // Is_Finished = Shows if the servo angles is equal to the setpoint
    void ConfigPinServo(int);   // ConfigPinServo: Read a value int and sets the ID and PIN for the Servo
    Servo Control;              // Control: Is the instancie of the Servo
  };

  // ---------------------------------------------------------------------------
  // STRUCT FOR LEGS
  // Contains the servos and function to control the Leg
  // ---------------------------------------------------------------------------
  typedef struct Leg
  {
    Joints Joint[__SERVOS__];               // Joint: Is the instance of the Struct Joints and instance a vector with a size equal to the number of Servos per Leg (DEFAULT: 2)
    void SetMinMaxAngles(int, int, int);    // SetMinMaxAngles: Function that allows to configure the Maximum and Minimum angle for the servo in especific Axis
    void GoToSetpoint();                    // GoToSetpoint: Function to move the servo a STEP to the current setpoint.
  };

  // ---------------------------------------------------------------------------
  // STRUCT FOR LEGS AND SERVO CONTROL
  // Contanis the Legs and function to move the legs at the same time.
  // Control the setpoints and Servos.
  // Buffer of movement.
  // Methods:
  //      - bool ProcessFinished()
  //      - void BackgroundProcess()
  //      - void UpdateSetpoints(int[], int[]);
  // ---------------------------------------------------------------------------
  typedef struct SERVO_DRIVER
  {
    Leg Legs[__SERVOS__];                 // Legs: Is a Instancie of the Struct Leg and create a vector with a size equal to the number of Legs in the Hexapod (DEFAULT: 6)
    bool ManualMode = AUTOMATIC;          // ManualMode: Variable that specifies the mode of control (DEFAULT: false)
    bool ProcessFinished();               // ProcessFinished: Function that returns true if all Servos are in their place or false if not.
    void BackgroundProcess();             // BackgroundProcess: Funtion that go over all servos and move them to their Setpoints
    void UpdateSetpoints(int[], int[]);   // UpdateSetpoints: Function that update the Setpoint of all servos.
  };

  // ---------------------------------------------------------------------------
  // STRUCT FOR RF COMUNICATION
  // Contains the functions to manage and read data from RF, and the Struct
  // that have the information of the instruction from the RF-Controller
  // Methods:
  //      - Start()
  //      - ReadData()
  // ---------------------------------------------------------------------------
  typedef struct RF_DRIVER
  {
    // ---------------------------------------------------------------------------
    // STRUCT OF PACKAGE RF
    // This contains all data recived from the RF Controler in mode Vector 
    // or mode Automatic
    // ---------------------------------------------------------------------------
    struct package
    {
      int VectorAnglesX[6] = {0, 0, 0, 0, 0, 0};  // Contains the values of the setpoints for the servos in Axis X
      int VectorAnglesY[6] = {0, 0, 0, 0, 0, 0};  // Contains the values of the setpoints for the servos in Axis Y
      int VectorPush[3] = {0, 0, 0};              // Contains the states of the customized push
    };
    typedef struct package Package;
    Package Data;                       // Data: Instance of Struct Package
    void Start();                       // Start: Function that initialize the RFController
    void ReadData();                    // ReadData: Function thats do a Read from the RF-Control and save them into the Struct Data.
  };

  // ---------------------------------------------------------------------------
  // STRUCT TO CONTROL TIMES
  // This Struct serves to make timers without delay funcions, only using 
  // the millins() funtion
  // Methods:
  //      - bool BackgroundTime()
  //      - void SetTimer(int)
  // ---------------------------------------------------------------------------
  typedef struct TIMES
  {
    int TimeInitial = 0;    // This is the time reference when the timer initialize
    int TimeFinal = 0;      // This is the time in microseconds when the timer is going to end
    bool BackgroundTime();  // Backgroundtime: Return a boolean value if the timer is not ended (true if is ended or false if not)
    void SetTimer(int);     // SetTimer: Read a int argument that is the end time, and initialize the counter.
  };

  #ifdef MODULESD == true
    // ---------------------------------------------------------------------------
    // STRUCT FOR RF COMUNICATION
    // Contains the functions to manage and read data from the Desktop application
    // and the control to comunicate with the Module SD
    // Methods:
    //      - 
    // ---------------------------------------------------------------------------
    typedef struct PCDRIVER
    {
      
    };

    // ---------------------------------------------------------------------------
    // STRUCT FOR COMUNICATION TO THE MODULESD
    // Work in progress...
    // ---------------------------------------------------------------------------
    typedef struct ModuleSD
    {

    };
    ModuleSD SDdriver;
  #endif
    
  TIMES Timer;                // Timer: Instance of the Struct TIMES.
  SERVO_DRIVER ServoDriver;   // ServoDriver: Instance of the Struct SERVO_DRIVER
  RF_DRIVER RFdriver;         // RFdriver: Instance of the RF_DRIVER
  
  bool All_Finished = false;  // All_Finished: Flag that indicates if all Servos are in their place

  int AnglesX[6] = {50,50,50,50,50,50};   // AnglesX: is the Setpoints of the Robot in Axis X 
  int AnglesY[6] = {50,50,50,50,50,50};   // AnglesY: is the Setpoints of the Robot in Axis Y
public:
  QURHexapod(int [__SERVOS__][__LEGS__], int[__SERVOS__][__LEGS__]);
  void Routine();
  void SelectMode(bool);
  void SetAnglesLeg(int[], bool);
  void SetAngleServo(int, int, bool);
  bool ServosFinished();
  void Debug(bool, int, int);
};

#endif

// Upcoming updates:
// - Add Gyroscope MPU6050 to control the rotation angle