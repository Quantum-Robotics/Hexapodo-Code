// ---------------------------------------------------------------------------
// RFController Quantum robotics Library - v1.0 - 06/20/2018
//
// AUTHOR:
// Created by Daniel Polanco - Jdanypa@gmail.com
//
// LINKS:
// GitHub Project: https://github.com/Elemeants/Hexapod-QuantumRobotics
//
// BACKGROUND:
// This library was created for the RFController Quantum Robotics, this project is part of my titling so I saw myself in 
// need to create a code to be able to control the robot from a RF-Controler, this library is Open-source so you can change
// add or copy anything you need to use it, everything is documented so that you understand all the code. If you need help
// with something you can send me an email to jdanypa@gmail.com.
//
// FEATURES:
// * This code is made for a Arduino 328p
// * This is only the RFControl for the PCB you need the Code LCDController
// * Convert the Joystick Coord into Positions for the Hexapod Shield
// * Push control for the customized controls from the plataform
//
// CONSTRUCTOR:
//
//
// METHODS:
//
//
// HISTORY:
// 06/20/2018 v1.0 - Initial release.
// ---------------------------------------------------------------------------


#ifndef RFCONTROLLER_H
#define RFCONTROLLER_H

#ifndef RF24_H
	#include "RF24.h"
#endif
#include <Arduino.h>

#define ROTATION false
#define WALKING  true

#define JOYSTICK1_X 	 A0
#define JOYSTICK1_Y 	 A1
#define JOYSTICK2_X 	 A2
#define JOYSTICK2_Y 	 A3

#define PUSH_A 	2
#define PUSH_B	3
#define PUSH_C	4

#define RGB_RED     A5
#define RGB_GREEN   A4
#define RGB_OK   	0
#define RGB_ERROR 	1
#define RGB_WAIT  	2

#define LCDController Serial   // RFController: Instance of the class RF24 to control the antenna.
const byte addresses[][6] = {"0"};  // Addresses: Address of comunication

typedef struct TIMES
  	{
    	int TimeInitial = 0;    // This is the time reference when the timer initialize
    	int TimeFinal = 0;      // This is the time in microseconds when the timer is going to end
    	bool BackgroundTime();  // Backgroundtime: Return a boolean value if the timer is not ended (true if is ended or false if not)
    	void SetTimer(int);     // SetTimer: Read a int argument that is the end time, and initialize the counter.
  	};

class RFControl
{
	typedef struct JOYSTICK_DRIVER
	{
		typedef struct Controler
		{
			int ReadX = 0;
			int ReadY = 0;
			void ReadValues(int, int);
		};
		Controler JoystickRotar;
		Controler JoystickMover;
		int Angle = 0;
		bool Mode = ROTATION;
		void ConvertToVector();
	};

	typedef struct PUSH_DRIVER
	{
		int Buttons[3] = {0, 0, 0};
		int UpdateStatus();
	};

	JOYSTICK_DRIVER Joysticks;
	PUSH_DRIVER PushControl;
	TIMES TimeOutSender;

	// ---------------------------------------------------------------------------
	// STRUCT OF PACKAGE RF
	// This contains all data recived from the RF Controler in mode Vector 
	// or mode Automatic
	// ---------------------------------------------------------------------------
	struct package
	{
	    bool Mode = WALKING;
	    int Angle = 0;
	    int VectorPush[3] = {0, 0, 0};
	};
	typedef struct package Package;
	Package Data;                       // Data: Instance of Struct Package
	void UpdateLCD();
	void StartLCD();
	void StartRF();
	void SendData();
public:
	void Start();
	void Routine();
};

#endif