#include <LiquidCrystal.h>

const int rs = 6, en = 7, d4 = 2, d5 = 3, d6 = 4, d7 = 5;   
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);  

#define MASTER Serial

#define RGB_RED         A0
#define RGB_GREEN       A1
#define RGB_BLUE        A2
#define RGB_OK          0
#define RGB_ERROR       1
#define RGB_WAIT        2
#define RGB_EXPECTING   3

#define PUSH_UP     8
#define PUSH_DOWN   9
#define PUSH_ENTER  10

#define MAX_DATA    3

const char loading_Animation[4] = {'|', '/', '-', '\\'};

struct RGB_DRIVER
{
  void Start(){    
    pinMode(RGB_RED, OUTPUT); 
    pinMode(RGB_GREEN, OUTPUT); 
    pinMode(RGB_BLUE, OUTPUT); 
    WAITING();
  };
  void ERROR(){    
    digitalWrite(RGB_GREEN, LOW); 
    digitalWrite(RGB_RED, HIGH); 
    digitalWrite(RGB_BLUE, LOW);
  };
  void OK(){       
    digitalWrite(RGB_GREEN, HIGH);  
    digitalWrite(RGB_RED, LOW); 
    digitalWrite(RGB_BLUE, LOW);
  };
  void WAITING(){
    digitalWrite(RGB_GREEN, LOW);
    digitalWrite(RGB_BLUE,  HIGH); 
    digitalWrite(RGB_RED,   HIGH); 
  };
  void PROCESS(){
    digitalWrite(RGB_GREEN, LOW);
    digitalWrite(RGB_BLUE,  LOW); 
    digitalWrite(RGB_RED,   HIGH); 
  }
  void ANIMATE(int x){
    if (x == 0) OK();
    if (x == 1) ERROR();
    if (x == 2) WAITING();
    if (x == 3) PROCESS(); 
    delay(50); OFF(); delay(50);  
  };
  void OFF(){
    digitalWrite(RGB_GREEN, LOW);   digitalWrite(RGB_RED, LOW);
  };
};
RGB_DRIVER RGB;

struct Timers
{
  int TimeInitial = 0;
  int TimeFinal = 0;
  bool BackgroundTime();
  void SetTimer(int);
};

bool Timers::BackgroundTime(){
  // If (<Current Time> less <Initial Time> is less than <Final time>) return true, else return false
  return ((millis() - TimeInitial) < TimeFinal);
}

void Timers::SetTimer(int __FINAL__){
  TimeInitial = millis();     // Sets the <Initial Time> to NOW
  TimeFinal   = __FINAL__;    // Sets the Final Time to the argument __FINAL__
}

void setup()
{
  pinMode(PUSH_UP,    INPUT);
  pinMode(PUSH_DOWN,  INPUT);
  pinMode(PUSH_ENTER, INPUT);
  RGB.Start();
  for (int i = 0; i < 20; ++i)
    RGB.ANIMATE(3);
  RGB.PROCESS();
  lcd.begin(16, 2);
  lcd.setCursor(0, 0); lcd.print("  CONECTANDO... "); 
  RGB.ANIMATE(0);
  RGB.PROCESS();
  MASTER.begin(38400);
  RGB.ANIMATE(0);
  RGB.PROCESS();
  int ptr = 0;
  while(!MASTER){
    RGB.PROCESS();
    lcd.setCursor(0, 0); lcd.print(loading_Animation[ptr]);
    ptr++;
    if(ptr > 3)
        ptr = 0;
  }
  ptr = 0;
  Timers SerialOut;
  SerialOut.SetTimer(2000);
  while(!(MASTER.available() > 0) || !SerialOut.BackgroundTime()){
      RGB.ANIMATE(3);
  }
  if(!SerialOut.BackgroundTime()){
      lcd.clear(); lcd.setCursor(0, 0); lcd.print(" ERROR ");
      RGB.ERROR();
  }
  else
  {
    RGB.ANIMATE(0);
    RGB.PROCESS();
    while(MASTER.available() > 0){
      RGB.WAITING();
      if(MASTER.read() == '0'){
          MASTER.print('0');
          lcd.clear(); lcd.setCursor(0, 0); lcd.print("S: OK");
          break;
      }
      RGB.OK();
      lcd.setCursor(0, 0); lcd.print(loading_Animation[ptr]);
      ptr++;
      if(ptr > 3)
          ptr = 0;
    }
  }
  MASTER.print('0');
  RGB.OK();
}

String Data[MAX_DATA];
int __POINTER__ = 0;

void loop()
{
  UpdateData();
  UpdateLCD();
}

void UpdateData(){
  int iterator = 0;
  while(MASTER.available() > 0){
    char buf;
    Data[iterator] = "";
    while(buf != '\n'){
      buf = MASTER.read();
      Data[iterator] += buf;
    }
    iterator++;
  }
  MASTER.println('0');
}

void UpdateLCD(){
  ReadPush();
  lcd.setCursor(0, 0); lcd.print(Data[__POINTER__]);
  lcd.setCursor(0, 1); lcd.print(Data[(__POINTER__ + 1)]);
}

void ReadPush(){
  Timers PushTimeout;
  if(digitalRead(PUSH_UP)){
    __POINTER__ --;
    if(__POINTER__ < 0)
      __POINTER__ = 0;
  }
  if(digitalRead(PUSH_DOWN)){
      __POINTER__ ++;
      if(__POINTER__ > (MAX_DATA - 2))
        __POINTER__ = 1;
  }
  PushTimeout.SetTimer(10);
  while(!PushTimeout.BackgroundTime())
      UpdateData();
}