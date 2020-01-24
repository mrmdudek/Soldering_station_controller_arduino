/*
  Soldering Iron Controller
  To change controler mode press UP and DOWN at the same time !!!
  24.4.2014. Milos. 
  Rewrited
  22.01.2020 M.Dudek mr.m.dudek@gmail.com v.1.2 
  // change PWM freq to 135Hz, add lcd clear every 60s, 
 */

//--include the library code:
#include <LiquidCrystal.h>
// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(PB8, PB9 , PB12, PB13, PB14, PB15); // RS,EN,D4,D5,D6,D7

//--define
#define btnMode PB4     // the number of the pushbutton pin
#define btnUp  PB3     // the number of the pushbutton pin
#define btnDown PA5     // the number of the pushbutton pin
#define btnADD  PA6     // the number of the pushbutton pin
#define INFO_build_in_led PC13     // the number of build in led
#define Additional_1 PA3    // the number of..
#define Additional_2 PA4    // the number of..
#define pwmHeater PB0      // the number of the PWM pin
#define BUILD_LED PC13

//-- global variables
float fTemp = 0.0;   // Temperature [C]
float fVoltage; // Voltage from Ain0
float fLSB = 0.0 ; // one LowSiginificantBit [V], (5/1024=0.0048828125) 
float fTime = 0.0; // Time
float fTimeSampling;
float fTimeLast;
int iCycleCounter=0; //Counter of cycles for printing on display
//--ADC info
const float fADResolution = 4096.0; // for 10 bit AD converter
const float fADMax = 3.3; // AD convertor max range
//--PID
const float fKp = 10; //proportional gian (Value tested with HQ Soldering Iron = 10 )
const float fKi = 1; //integral gian (Value tested with HQ Soldering Iron = 1)
const float fKd = 0.0; //derivative gian (Value tested with HQ Soldering Iron = 10)
//--Graphic
const int iHowManyCyclesToRefreshDisplay = 26; //Printing on Display after specified number of sampling cycles
const int iSerialSend = 1; //0-don't send, 1-send to serial
//--Temperature calculation
  // 0 oC - 1,105V
  // 85oC - 1,455
float fTb = -268.35;  // Ambient temperature [C] (b coefficient in y=ax+b, y=temperature,x=input_voltage)
float fTa = 242.85; // [degC/LSB] (a coefficient in y=ax+b, y=temperature,x=input_voltage)
//-- Temperature set
float fTSet = 400.0; // Set point [C]
//-- Type of work
int iPID_or_OnOff = 0; //0=PWM, 1=OnOff  !!!!! HERE THE INITIAL CONTROLER IS SET !!!!
//--PID
float fEk = 0.0; //current error
float fEk_1 = 0.0; //last error
float fPWM; //output PWM
float fIntegralTerm =0.0;

HardwareTimer pwmtimer2(3);

void setup() {
  fLSB = fADMax / fADResolution; // one LowSiginificantBit [V], (3.3/4096=0.000805664) 
  lcd.begin(16, 2);   // set up the LCD's number of columns and rows: 
  pinMode(PB0, PWM); // Heater output
  pinMode(INFO_build_in_led, OUTPUT); // Build in led to show if OnOff regulation work
  pinMode(btnMode, INPUT_PULLUP);   
  pinMode(btnUp, INPUT_PULLUP);   
  pinMode(btnDown, INPUT_PULLUP); 
  pinMode(btnADD, INPUT_PULLUP);  
  pinMode(PA0, INPUT_ANALOG); // Temperature analog input
  Serial.begin(9600);  

pwmtimer2.pause();
pwmtimer2.setPrescaleFactor(16);
pwmtimer2.setOverflow(65535);
pwmtimer2.refresh();
pwmtimer2.resume();
  
  fTimeLast = millis();
}

float fMeasureOversampling() {
  float fSum = 0.0; 
  for (int i=1; i <= 2048; i++){
    fSum = fSum + analogRead(PA0);
  } 
  return fSum / 2048 ;
}

float fLimit (float fPromenljiva, float fMax, float fMin){ //Saturation function
  if (fPromenljiva > fMax)  {  fPromenljiva = fMax ; }
  if (fPromenljiva < fMin)  {  fPromenljiva = fMin ; }
  return fPromenljiva ;
}

float fSimplePID() {
  // calculate PID command, first calculate the coeficients
  float fSimplePID;
  fIntegralTerm = fIntegralTerm + fKi * fEk * fTimeSampling;
  fIntegralTerm = fLimit(fIntegralTerm, 255.0, 0.0);
  fSimplePID = fKp * fEk + fIntegralTerm + fKd * (fEk - fEk_1); // SimplePID
  fSimplePID = fLimit(fSimplePID, 255.0, 0.0);
  if ((fEk > 20)){
    fSimplePID = 255.0;
    }
  return 255-fSimplePID;
}

void RefreshDisplay() {
  lcd.setCursor(0, 0);
  lcd.print("Set:    Temp:   ");
  lcd.setCursor(4, 0);
  lcd.print(fTSet,0);
  lcd.setCursor(13, 0);
  lcd.print(fTemp,0);

  if (iPID_or_OnOff == 0)  {   //PID is active controler

    lcd.setCursor(0, 1);
    lcd.print("%PWM=    ");
    lcd.setCursor(5, 1);
    lcd.print(100-(fPWM/2.55),0);    
    
  } 
  if (iPID_or_OnOff == 1)  {   //OnOff is active controler
    lcd.setCursor(0, 1);
    if (fEk < 0) { lcd.print("Off");     }
    else         { lcd.print("On ");   }
  } 
}
 
void SerialSend() {
  Serial.print("Time:"); 
  Serial.print(fTime,3);   
  Serial.print(",TSet:"); 
  Serial.print(fTSet,1);   
  Serial.print(",Temp:"); 
  Serial.print(fTemp,2);   
  Serial.print(",Voltage:"); 
  Serial.print(fVoltage,3);
  Serial.print(",PWM%:"); 
  Serial.println(100-(fPWM/2.55));
} 

void Buttons() {
  if ((digitalRead(btnUp) == LOW) && (digitalRead(btnDown) == HIGH )) {   
      fTSet = fTSet + 2.0 ;
      if (fTSet > 450.0)  {       fTSet = 450.0 ; }  //LIMIT to 450 degC
  }  
  if ((digitalRead(btnUp) == HIGH) && (digitalRead(btnDown) == LOW )) {   
      fTSet = fTSet - 2.0 ;
      if (fTSet < 0.0)  {    fTSet = 0.0 ; } //LIMIT to 0
  }  
  if ((digitalRead(btnUp) == LOW) && (digitalRead(btnDown) ==LOW )) {   
      iPID_or_OnOff++ ; //increment
      if (iPID_or_OnOff > 1)  {     iPID_or_OnOff = 0; } //then reset it to 0
  } 
  if (digitalRead(btnMode) == LOW) {   
    fTSet = fTSet + 5.0;
    if (fTSet < 150.0) {   
        fTSet = 150.0 ;
      }    
      if ( (fTSet > 150.0) && (fTSet < 280.0 ) ) {  fTSet = 280.0 ;   }    
      else if ( (fTSet > 280.0) && (fTSet < 320.0 ) ) {  fTSet = 320.0 ;   }    
      else if ( (fTSet > 320.0) && (fTSet < 350.0 ) ) {  fTSet = 350.0 ;   } 
      else if ( (fTSet > 350.0) && (fTSet < 380.0 ) ) {  fTSet = 380.0 ;   }    
      else if ( (fTSet > 380.0) && (fTSet < 400.0 ) ) {  fTSet = 400.0 ;   } 
      else if ( (fTSet > 400.0) && (fTSet < 450.0 ) ) {  fTSet = 450.0 ;   }   
      else if (  fTSet > 450.0)                       {  fTSet = 0.0 ;     }    
  }  
} 
  
void loop() {
  fTime = millis() / 1000.0 ;
  fTimeSampling = fTime - fTimeLast;
  fTimeLast = fTime ;

  fVoltage = fMeasureOversampling() * fLSB; // read the input pin and calculate Voltage
  fTemp = fTa * fVoltage + fTb;    // calculate the Temperature

  fEk = fTSet - fTemp; //error for simple PID
  
  if (iPID_or_OnOff == 1 )  { //On-Off regulator  // simple ON-OFF control, works well (about +8 and -2 degC about SetPoint)
    if (fEk < 0) 
    {
      digitalWrite(PB0,HIGH);// inverted logic
      digitalWrite(INFO_build_in_led,HIGH);// if High led is off
    }
    else         { 
      digitalWrite(PB0,LOW);// inverted logic
      digitalWrite(INFO_build_in_led,LOW);  // if low led is on
      }
  }

  if (iPID_or_OnOff == 0 )  { //PID regulator
    fPWM = fSimplePID(); //calculate PID command
    fEk_1 = fEk;  //store the last error 
    pwmWrite(pwmHeater, (fPWM*65535.0/255.0));  //execute the command
  }

  
  if (iSerialSend == 1)  {  SerialSend() ; } //print on UART, serial port
  iCycleCounter++; //increment cycle counter
      if (iPID_or_OnOff == 0 )
    {digitalWrite(PC13,HIGH);
    }
  if ((iCycleCounter%5) == 0)
  {    Buttons(); //read the buttons
  }
  if ((((int)(fTime)) % 60) == 0)
  {
    lcd.clear();
  }
  if (iCycleCounter >= iHowManyCyclesToRefreshDisplay )  { //print od dislplay
    RefreshDisplay() ;  
    if (iPID_or_OnOff == 0 )
    {digitalWrite(PC13,LOW);
    }
    iCycleCounter = 0; //reset counter
  }
//  delay(100); //omitted - goes on approx 0.144 seconds per cycle
}
