#include "RFController.h"
#include <Arduino.h>

RF24 RFController(7, 8);

struct RGB_BUILDER
{
	void Start(){    
		pinMode(RGB_RED, OUTPUT); 
		pinMode(RGB_GREEN, OUTPUT);
		WAITING();
	};
	void ERROR(){    
		digitalWrite(RGB_GREEN, LOW); 
		digitalWrite(RGB_RED, HIGH); 
	};
	void OK(){       
		digitalWrite(RGB_GREEN, HIGH);  
		digitalWrite(RGB_RED, LOW); 
	};
	void WAITING(){
		digitalWrite(RGB_GREEN, HIGH);
		digitalWrite(RGB_RED,  HIGH); 
	};
	void ANIMATE(int x){
		if (x == 0) OK();
		if (x == 1) ERROR();
		if (x == 2) WAITING();
		delay(50); OFF(); delay(50);  
	};
	void OFF(){
		digitalWrite(RGB_GREEN, LOW);   digitalWrite(RGB_RED, LOW);
	};
}FLAG;

void RFControl::JOYSTICK_DRIVER::Controler::ReadValues(int pinX, int pinY){
	ReadX = analogRead(pinX);
	ReadY = analogRead(pinY);
	ReadX -= 512;
	ReadY -= 512;
	if (ReadX < 100 && ReadX > -100)
		ReadX = 0;
	if (ReadY < 100 && ReadY > -100)
		ReadY = 0;
}

void RFControl::JOYSTICK_DRIVER::ConvertToVector(){
	JoystickMover.ReadValues(JOYSTICK1_X, JOYSTICK1_Y);
	JoystickRotar.ReadValues(JOYSTICK2_X, JOYSTICK2_Y);
	int _x = Mode == WALKING ? JoystickMover.ReadX : JoystickRotar.ReadX;
	int _y = Mode == WALKING ? JoystickMover.ReadX : JoystickRotar.ReadX;
	int r = sqrt(_x*_x + _y*_y);
	Angle = atan2(_y, _x);
    Angle = (180*Angle)/M_PI;
}

int RFControl::PUSH_DRIVER::UpdateStatus(){
	Buttons[0] = digitalRead(PUSH_A);
	Buttons[1] = digitalRead(PUSH_B);
	Buttons[2] = digitalRead(PUSH_C);
	for(int x = 0; x < 3; x++){
	    if(Buttons[x])
	        return x;
	}
	return -1;
}

// ---------------------------------------------------------------------------
// Methods for Timers controller
//       - BackgroundTime()
//       - SetTimer(int __FINAL__)
// ---------------------------------------------------------------------------

/**
  @Struct RFControl -> TIMES
  @Function BackgroundTime
  @purpuse Checks if the timer ended

  @return Return a boolean value if the timer is not ended (true if is ended or false if not)
*/
bool TIMES::BackgroundTime(){
  // If (<Current Time> less <Initial Time> is less than <Final time>) return true, else return false
  return ((millis() - TimeInitial) < TimeFinal);
}

/**
  @Struct RFControl -> TIMES
  @Function SetTimer
  @purpuse Read a int argument that is the end time, and initialize the counter.

  @param __FINAL__ Variable that indicates the term time
*/
void TIMES::SetTimer(int __FINAL__){
  TimeInitial = millis();     // Sets the <Initial Time> to NOW
  TimeFinal   = __FINAL__;    // Sets the Final Time to the argument __FINAL__
}

void RFControl::UpdateLCD(){
	TIMES SerialOut;
	FLAG.WAITING();
	String *__DATA__; 
	__DATA__[0] = ("Modo: " + String(Joysticks.Mode == true ? "Caminata" : "Rotacion"));
	__DATA__[1] = "Angulo: " + String(Data.Angle);
	__DATA__[2] = ("A:" + String(Data.VectorPush[0]) + " B:" + String(Data.VectorPush[2]) + " C:" + String(Data.VectorPush[2]));
	for (int i = 0; i < 3; ++i)
		Serial.println(__DATA__[i]);
	SerialOut.SetTimer(500);
	while(!SerialOut.BackgroundTime()){
		if(LCDController.available() > 0){
			if(LCDController.read() == '0'){
				FLAG.OK();
				break;
			}
		}
		FLAG.ANIMATE(0);
	}
}

void RFControl::StartLCD(){
	TIMES SerialOut;
	FLAG.WAITING();
	LCDController.begin(38400);
	SerialOut.SetTimer(1000);
	while(!LCDController || !SerialOut.BackgroundTime()){
		for(int x = 0; x < 5; x ++){
	        FLAG.ANIMATE(2);
	    }
	}
	if(!SerialOut.BackgroundTime()){
	    FLAG.ERROR();
	}
	else{
		LCDController.print('0');
		SerialOut.SetTimer(1000);
		while(!SerialOut.BackgroundTime()){
			if(LCDController.available() > 0){
				if(LCDController.read() == '0'){
					FLAG.OK();	
					break;
				}
			}
			FLAG.ANIMATE(0);
		}
	}
}

void RFControl::StartRF(){
	FLAG.WAITING();
	RFController.begin();
  	RFController.setChannel(115); 
  	RFController.setPALevel(RF24_PA_MAX);
  	RFController.setDataRate( RF24_250KBPS ) ; 
  	RFController.openWritingPipe(addresses[0]);
  	FLAG.OK();
}

void RFControl::SendData(){
	FLAG.WAITING();
	bool isSended = RFController.write(&Data, sizeof(Data));
	if (!isSended && !TimeOutSender.BackgroundTime()){
		FLAG.ERROR();
		SendData();
	}
	FLAG.OK();
}

void RFControl::Routine(){
	TIMES RFtimeOut;
	FLAG.OK();
	Joysticks.ConvertToVector();
	int isPushed = PushControl.UpdateStatus();
	UpdateLCD();
	RFtimeOut.SetTimer(500);
	SendData();
}

void RFControl::Start(){
	pinMode(JOYSTICK1_X, INPUT);
	pinMode(JOYSTICK1_Y, INPUT);
	pinMode(JOYSTICK2_X, INPUT);
	pinMode(JOYSTICK2_Y, INPUT);
	pinMode(PUSH_A, INPUT);
	pinMode(PUSH_B, INPUT);
	pinMode(PUSH_C, INPUT);
	FLAG.Start();
	for (int i = 0; i < 20; ++i)
		FLAG.ANIMATE(0);
	StartLCD();
	StartRF();
}