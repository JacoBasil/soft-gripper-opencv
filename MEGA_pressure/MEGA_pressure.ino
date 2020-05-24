/*
 * SOFT ROBOTICS
 * THIS CODE IS FOR AIR SUPPLY SYSTEM OF OPERATING SOFT ROBOTIC GRIPPER
 * DEVELOPED BY IAKOV VASILEV
 * CONTACTS: +7(926)077-83-19
 * MAIL: iakov.vasilev@skoltech.ru
 * SKOLKOVO INSTITUTE OF SCIENCE AND TECHNOLOGY 
  */
//----------- PINS OF ELECTROMAGNETIC VALVES RELAY -----------
#define PIN_Valve_Pump              15      //1st of nearby row 
#define PIN_Valve_Receiver          17      //2nd of nearby row 
#define PIN_Valve_Drop              19      //3d  of nearby row
#define PIN_Valve_Vacuum            2       //4th of nearby row
#define PIN_Valve_FirstActuator     4       //1st of distant row 
#define PIN_Valve_SecondActuator    6       //2nd of distant row
#define PIN_Valve_ThirdActuator     8       //3d  of distand row

//----------- PINS OF PRESSURE SENSOR -----------
#define PIN_PressureSensor_Receiver         A8
#define PIN_PressureSensor_FirstActuator    A9
#define PIN_PressureSensor_SecondActuator   A10
#define PIN_PressureSensor_ThirdActuator    A11

//----------- PINS OF MOTOR DRIVER BB-VNH3SP30 Sparkfun (COMPRESSOR) -----------
#define InA         34
#define InB         35
#define PIN_PWM     10
#define EnA         36
#define EnB         37

//----------- PINS OF MOTOR DRIVER L298N (VACUUM PUMP) -----------
#define In1         40
#define In2         41

//--------------- PINS OF POTENTIOMETERS --------------
#define PIN_Potentiometer_1     A2
#define PIN_Potentiometer_2     A1
#define PIN_Potentiometer_3     A0

//----------------- LIBRARIES --------------------
#include <Wire.h>               // библиотека для соединения
#include <LiquidCrystal_I2C.h>  // библиотека дисплея
#include <avr/io.h>
#include <avr/interrupt.h>

//----------------- OBJECTS, CONSTANCES, VARIABLES, PARAMETERS --------------------
LiquidCrystal_I2C lcd(0x27, 20, 4);

String Code;
boolean ReceiverPumped = false;
int ValveOpenTime = 7;
int ValveCloseTime = 10;

int Reading_Potentiometer_1;
int Reading_Potentiometer_2;
int Reading_Potentiometer_3;
int Reading_PressureSensor_Receiver;
int Reading_PressureSensor_FirstActuator;
int Reading_PressureSensor_SecondActuator;
int Reading_PressureSensor_ThirdActuator;
float Pressure_Receiver;
float Pressure_FirstActuator;
float Pressure_SecondActuator;
float Pressure_ThirdActuator;
float Set_Pressure_1;
float Set_Pressure_2;
float Set_Pressure_3;
float Max_Pressure;
const float P_atm = 101.0;     //athmosphere pressure 101 kPa
const float P_trig = 10.0;     //Triggering pressure 10 kPa when 
const float P_targ = 70.0;     //Desired absolute vacuum pressure 70 kPa

//----------------- SETUP --------------------
void setup() {
//----------------- SETUP TIMER1 -------------------- 
  cli();
    TCCR1A = 0;

    TCNT1 = 0;

    OCR1A = 19999;            //f = 100 Hz. OCR1A = (16*10^6) / (100*8) - 1 (must be <65536)

    TIMSK1 = 0;
    TIMSK1 |= (1<<OCIE1A);
  
    TCCR1B = 0;

    TCCR1B |= (1<<WGM12)|(1<<CS11);     //CTC mode with prescaler 8
    
    TCCR1C = 0;
  sei();
  pinMode(PIN_Valve_Pump, OUTPUT);
  pinMode(PIN_Valve_Receiver, OUTPUT);
  pinMode(PIN_Valve_Drop, OUTPUT);
  pinMode(PIN_Valve_Vacuum, OUTPUT);
  pinMode(PIN_Valve_FirstActuator, OUTPUT);
  pinMode(PIN_Valve_SecondActuator, OUTPUT);
  pinMode(PIN_Valve_ThirdActuator, OUTPUT);
  pinMode(InA, OUTPUT);
  pinMode(InB, OUTPUT);
  pinMode(In1, OUTPUT);
  pinMode(In2, OUTPUT);
  pinMode(EnA, INPUT);
  pinMode(EnB, INPUT);
  pinMode(PIN_PWM, OUTPUT);
  Serial.begin(9600);       //startup serial connection with BAUD parameter 9600
  lcd.init();               //initialize lcd screen 
  lcd.backlight();          //turn on the backlight
  lcd.clear();              //clean lcd screen
  DropAir(0);
  DropAir(4);
}

//---------------- CALLBACK OF INTERRUPT ---------------------
void SensorsReading(){
//------------ READINGS OF SET PRESSURE ----------------------------
  Reading_Potentiometer_1 = analogRead(PIN_Potentiometer_1);
  Reading_Potentiometer_2 = analogRead(PIN_Potentiometer_2);
  Reading_Potentiometer_3 = analogRead(PIN_Potentiometer_3);
  Reading_PressureSensor_Receiver = analogRead(PIN_PressureSensor_Receiver);
  Reading_PressureSensor_FirstActuator = analogRead(PIN_PressureSensor_FirstActuator);
  Reading_PressureSensor_SecondActuator = analogRead(PIN_PressureSensor_SecondActuator);
  Reading_PressureSensor_ThirdActuator = analogRead(PIN_PressureSensor_ThirdActuator);

//-------------- PRESSURE VALUE OF RECEIVER AND ALL ACTUATORS (kPa) -----------------------
  Pressure_Receiver = (Reading_PressureSensor_Receiver/1024.0 + 0.00842)/0.002421;
  Pressure_FirstActuator = (Reading_PressureSensor_FirstActuator/1024.0 + 0.00842)/0.002421;
  Pressure_SecondActuator = (Reading_PressureSensor_SecondActuator/1024.0 + 0.00842)/0.002421;
  Pressure_ThirdActuator = (Reading_PressureSensor_ThirdActuator/1024.0 + 0.00842)/0.002421;

//-------------- SET PRESSURE OF ALL ACTUATORS (kPa) -----------------------
  Set_Pressure_1 = (Reading_Potentiometer_1/1024.0 + 0.00842)/0.002421;
  Set_Pressure_2 = (Reading_Potentiometer_2/1024.0 + 0.00842)/0.002421;
  Set_Pressure_3 = (Reading_Potentiometer_3/1024.0 + 0.00842)/0.002421;
}

//----------- MOTOR STARTUP FUNCTION WITH INPUT PARAMETER OF PWM LEVEL -----------
void MotorOn(int PWM){
  digitalWrite(InA, HIGH);
  digitalWrite(InB, LOW); 
  //digitalWrite(EnA, HIGH);
  //digitalWrite(EnB, HIGH);  
  analogWrite(PIN_PWM, PWM);
}

//----------- MOTOR FORCED BLOCKING FUNCTION -----------
void MotorOff(){
  digitalWrite(InA, LOW);
  digitalWrite(InB, LOW); 
  analogWrite(PIN_PWM, 0);
}

//----------- FUNCTION OF DROP AIR FROM 1 - 1ST, 2 -2ND,3 - 3D ACTUATOR OR 4 - FROM ALL ACTUATORS OR 0 - FROM RECEIVER  -----------
void DropAir(int i){
  digitalWrite(PIN_Valve_Pump, LOW);
  digitalWrite(PIN_Valve_Vacuum, LOW);
  switch(i) {
    case 1:
      digitalWrite(PIN_Valve_Receiver, LOW);
      digitalWrite(PIN_Valve_FirstActuator, HIGH);
      digitalWrite(PIN_Valve_SecondActuator, LOW);
      digitalWrite(PIN_Valve_ThirdActuator, LOW);
      digitalWrite(PIN_Valve_Drop, HIGH);
      delay(1000);
      digitalWrite(PIN_Valve_Drop, LOW);
      digitalWrite(PIN_Valve_FirstActuator, LOW);
      break;
    case 2:
      digitalWrite(PIN_Valve_Receiver, LOW);
      digitalWrite(PIN_Valve_FirstActuator, LOW);
      digitalWrite(PIN_Valve_SecondActuator, HIGH);
      digitalWrite(PIN_Valve_ThirdActuator, LOW);
      digitalWrite(PIN_Valve_Drop, HIGH);
      delay(1000);
      digitalWrite(PIN_Valve_Drop, LOW);
      digitalWrite(PIN_Valve_SecondActuator, LOW);
      break;    
    case 3:
      digitalWrite(PIN_Valve_Receiver, LOW);
      digitalWrite(PIN_Valve_FirstActuator, LOW);
      digitalWrite(PIN_Valve_SecondActuator, LOW);
      digitalWrite(PIN_Valve_ThirdActuator, HIGH);
      digitalWrite(PIN_Valve_Drop, HIGH);
      delay(1000);
      digitalWrite(PIN_Valve_Drop, LOW);
      digitalWrite(PIN_Valve_ThirdActuator, LOW);
      break; 
    case 4:
      digitalWrite(PIN_Valve_Receiver, LOW);
      digitalWrite(PIN_Valve_FirstActuator, HIGH);
      digitalWrite(PIN_Valve_SecondActuator, HIGH);
      digitalWrite(PIN_Valve_ThirdActuator, HIGH);
      digitalWrite(PIN_Valve_Drop, HIGH);
      delay(1500);
      digitalWrite(PIN_Valve_Drop, LOW);
      digitalWrite(PIN_Valve_FirstActuator, LOW);
      digitalWrite(PIN_Valve_SecondActuator, LOW);
      digitalWrite(PIN_Valve_ThirdActuator, LOW);
      break; 
    case 0:
      digitalWrite(PIN_Valve_Receiver, HIGH);
      digitalWrite(PIN_Valve_FirstActuator, LOW);
      digitalWrite(PIN_Valve_SecondActuator, LOW);
      digitalWrite(PIN_Valve_ThirdActuator, LOW);
      digitalWrite(PIN_Valve_Drop, HIGH);
      delay(2000);
      digitalWrite(PIN_Valve_Drop, LOW);
      digitalWrite(PIN_Valve_Receiver, LOW);
      break;       
  }
}
//----------- FUNCTION OF REDUCING PRESSURE IN ACTUATORS -----------
void ReducePressure(uint8_t i, uint8_t ValveOpenTime){
  digitalWrite(PIN_Valve_Pump, LOW);
  digitalWrite(PIN_Valve_Vacuum, LOW);
  switch(i) {
    case 1:
      digitalWrite(PIN_Valve_Receiver, LOW);
      digitalWrite(PIN_Valve_FirstActuator, HIGH);
      digitalWrite(PIN_Valve_SecondActuator, LOW);
      digitalWrite(PIN_Valve_ThirdActuator, LOW);
      digitalWrite(PIN_Valve_Drop, HIGH);
      delay(ValveOpenTime);
      digitalWrite(PIN_Valve_Drop, LOW);
      digitalWrite(PIN_Valve_FirstActuator, LOW);
      break;
    case 2:
      digitalWrite(PIN_Valve_Receiver, LOW);
      digitalWrite(PIN_Valve_FirstActuator, LOW);
      digitalWrite(PIN_Valve_SecondActuator, HIGH);
      digitalWrite(PIN_Valve_ThirdActuator, LOW);
      digitalWrite(PIN_Valve_Drop, HIGH);
      delay(ValveOpenTime);
      digitalWrite(PIN_Valve_Drop, LOW);
      digitalWrite(PIN_Valve_SecondActuator, LOW);
      break;    
    case 3:
      digitalWrite(PIN_Valve_Receiver, LOW);
      digitalWrite(PIN_Valve_FirstActuator, LOW);
      digitalWrite(PIN_Valve_SecondActuator, LOW);
      digitalWrite(PIN_Valve_ThirdActuator, HIGH);
      digitalWrite(PIN_Valve_Drop, HIGH);
      delay(ValveOpenTime);
      digitalWrite(PIN_Valve_Drop, LOW);
      digitalWrite(PIN_Valve_ThirdActuator, LOW);
      break; 
    case 4:
      digitalWrite(PIN_Valve_Receiver, LOW);
      digitalWrite(PIN_Valve_FirstActuator, HIGH);
      digitalWrite(PIN_Valve_SecondActuator, HIGH);
      digitalWrite(PIN_Valve_ThirdActuator, HIGH);
      digitalWrite(PIN_Valve_Drop, HIGH);
      delay(ValveOpenTime);
      digitalWrite(PIN_Valve_Drop, LOW);
      digitalWrite(PIN_Valve_FirstActuator, LOW);
      digitalWrite(PIN_Valve_SecondActuator, LOW);
      digitalWrite(PIN_Valve_ThirdActuator, LOW);
      break; 
  }
}

//----------- FUNCTION OF VACUUMIZATION OF ACTUATORS --------------------------
void CreateVacuum(int N, int t){
  switch(N){
    case 1:
      digitalWrite(In1, HIGH);
      digitalWrite(In2, LOW);
      digitalWrite(PIN_Valve_Pump, LOW);
      digitalWrite(PIN_Valve_Receiver, LOW);
      digitalWrite(PIN_Valve_Drop, LOW);
      digitalWrite(PIN_Valve_Vacuum, HIGH);
      digitalWrite(PIN_Valve_FirstActuator, HIGH);
      delay(t);
      digitalWrite(In1, LOW);
      digitalWrite(In2, LOW);
      digitalWrite(PIN_Valve_Vacuum, LOW);
      digitalWrite(PIN_Valve_FirstActuator, LOW);
      break;
    case 2:
      digitalWrite(In1, HIGH);
      digitalWrite(In2, LOW);
      digitalWrite(PIN_Valve_Pump, LOW);
      digitalWrite(PIN_Valve_Receiver, LOW);
      digitalWrite(PIN_Valve_Drop, LOW);
      digitalWrite(PIN_Valve_Vacuum, HIGH);
      digitalWrite(PIN_Valve_SecondActuator, HIGH);
      delay(t);
      digitalWrite(In1, LOW);
      digitalWrite(In2, LOW);
      digitalWrite(PIN_Valve_Vacuum, LOW);
      digitalWrite(PIN_Valve_SecondActuator, LOW);
      break;
    case 3:
      digitalWrite(In1, HIGH);
      digitalWrite(In2, LOW);
      digitalWrite(PIN_Valve_Pump, LOW);
      digitalWrite(PIN_Valve_Receiver, LOW);
      digitalWrite(PIN_Valve_Drop, LOW);
      digitalWrite(PIN_Valve_Vacuum, HIGH);
      digitalWrite(PIN_Valve_ThirdActuator, HIGH);
      delay(t);
      digitalWrite(In1, LOW);
      digitalWrite(In2, LOW);
      digitalWrite(PIN_Valve_Vacuum, LOW);
      digitalWrite(PIN_Valve_ThirdActuator, LOW);
      break;
    case 4:
      digitalWrite(In1, HIGH);
      digitalWrite(In2, LOW);
      digitalWrite(PIN_Valve_Pump, LOW);
      digitalWrite(PIN_Valve_Receiver, LOW);
      digitalWrite(PIN_Valve_Drop, LOW);
      digitalWrite(PIN_Valve_Vacuum, HIGH);
      digitalWrite(PIN_Valve_FirstActuator, HIGH);
      digitalWrite(PIN_Valve_SecondActuator, HIGH);
      digitalWrite(PIN_Valve_ThirdActuator, HIGH);
      delay(t);
      digitalWrite(In1, LOW);
      digitalWrite(In2, LOW);
      digitalWrite(PIN_Valve_Vacuum, LOW);
      digitalWrite(PIN_Valve_ThirdActuator, LOW);
      digitalWrite(PIN_Valve_SecondActuator, LOW);
      digitalWrite(PIN_Valve_ThirdActuator, LOW);      
      break;
  }
}

double max3(double a, double b, double c)
{
  double maxguess;

  maxguess = max(a,b);  // biggest of A and B
  maxguess = max(maxguess, c);  // but maybe C is bigger?

  return(maxguess);
}
boolean Receiver_Pumping(boolean pumped, double p, double p_ref)  {
  if(!pumped){
    if(p_ref + 50.0 > p && p_ref > P_atm){
      digitalWrite(PIN_Valve_Pump, HIGH);
      digitalWrite(PIN_Valve_Receiver, LOW);
      MotorOn(150);
      pumped = false;
    }
    else{
      digitalWrite(PIN_Valve_Pump, LOW);
      digitalWrite(PIN_Valve_Receiver, LOW);
      MotorOff();
      pumped = true;
    }
  }
  return !pumped;
}
void InflateActuator(int N, double P_S, double P_A){
    switch(N) {
    case 1:
      if(P_S > P_atm){
        if(P_S - 5.0 > P_A){
          digitalWrite(PIN_Valve_FirstActuator, HIGH);
        }
        else{
          ReducePressure(1, ValveOpenTime);
        }
      }
      else{
        if(P_S < P_trig && P_A > P_targ){
          CreateVacuum(1, ValveOpenTime);
        }
      }
      break;
    case 2:
      if(P_S > P_atm){
        if(P_S - 5.0 > P_A){
          digitalWrite(PIN_Valve_SecondActuator, HIGH);
        }
        else{
          ReducePressure(2, ValveOpenTime);
        }
      }
      else{
        if(P_S < P_trig && P_A > P_targ){
          CreateVacuum(2, ValveOpenTime);
        }
      }
      break;
    case 3:
      if(P_S > P_atm){
        if(P_S - 5.0 > P_A){
          digitalWrite(PIN_Valve_ThirdActuator, HIGH);
        }
        else{
          ReducePressure(3, ValveOpenTime);
        }
      }
      else{
        if(P_S < P_trig && P_A > P_targ){
          CreateVacuum(3, ValveOpenTime);
        }
      }
      break;
}
}
//------------------------------- INTERRUPT HANDLER ---------------------------------------
ISR(TIMER1_COMPA_vect){
  SensorsReading();
  //Maximum set pressure among all 3 actuators 
  //Max_Pressure = max3(Set_Pressure_1, Set_Pressure_2, Set_Pressure_3); 
  //Receiver pumping
  //ReceiverPumped = Receiver_Pumping(ReceiverPumped, Pressure_Receiver, Max_Pressure);
  }

//------------------------------- MAIN LOOP ----------------------------------------------
void loop() {
//InflateActuator(1, Set_Pressure_1, Pressure_FirstActuator);  

//-------------------------- FIRST ACTUATOR INFLATING ----------------------
  if(Set_Pressure_1  >  Pressure_FirstActuator && Set_Pressure_1 > P_atm){
    if(!ReceiverPumped){
      if(Set_Pressure_1 + 50.0 > Pressure_Receiver){
        digitalWrite(PIN_Valve_Pump, HIGH);
        digitalWrite(PIN_Valve_Receiver, LOW);
        MotorOn(150);
      }
      else{
        ReceiverPumped = true;
        digitalWrite(PIN_Valve_Pump, LOW);
        digitalWrite(PIN_Valve_Receiver, LOW);
        MotorOff();
      }
    }
    else{
      if(Set_Pressure_1 + 30.0 < Pressure_Receiver){
        digitalWrite(PIN_Valve_Pump, LOW);
        MotorOff();
        digitalWrite(PIN_Valve_Receiver, HIGH);
        if(Set_Pressure_1 - 5 > Pressure_FirstActuator){
          digitalWrite(PIN_Valve_FirstActuator, HIGH);
          digitalWrite(PIN_Valve_SecondActuator, HIGH);
          digitalWrite(PIN_Valve_ThirdActuator, HIGH);
          //delay(ValveOpenTime);
          //digitalWrite(PIN_Valve_FirstActuator, LOW);
          //delay(ValveCloseTime);
        }
        else{
          digitalWrite(PIN_Valve_Receiver, LOW);
          digitalWrite(PIN_Valve_FirstActuator, LOW);
          digitalWrite(PIN_Valve_SecondActuator, LOW);
          digitalWrite(PIN_Valve_ThirdActuator, LOW);
        }
      }
      else{
        ReceiverPumped = false;
        digitalWrite(PIN_Valve_Pump, HIGH);
        digitalWrite(PIN_Valve_Receiver, LOW);
        MotorOn(150);
      }
    }
  }
  else if(Set_Pressure_1  <  Pressure_FirstActuator && Set_Pressure_1 > 101.0){
    if(Set_Pressure_1 + 5 < Pressure_FirstActuator){
      ReducePressure(4, ValveOpenTime);
    }
  }
/* 
//-------------------------- SECOND ACTUATOR INFLATING ----------------------  
  if(Set_Pressure_2  >  Pressure_SecondActuator && Set_Pressure_2 > 101.0){
    if(!ReceiverPumped){
      if(Set_Pressure_2 + 50.0 > Pressure_Receiver){
        digitalWrite(PIN_Valve_Pump, HIGH);
        digitalWrite(PIN_Valve_Receiver, LOW);
        MotorOn(150);
      }
      else{
        ReceiverPumped = true;
        digitalWrite(PIN_Valve_Pump, LOW);
        digitalWrite(PIN_Valve_Receiver, LOW);
        MotorOff();
      }
    }
    else{
      if(Set_Pressure_2 + 30.0 < Pressure_Receiver){
        digitalWrite(PIN_Valve_Pump, LOW);
        MotorOff();
        digitalWrite(PIN_Valve_Receiver, HIGH);
        if(Set_Pressure_2 - 5 > Pressure_SecondActuator){
          digitalWrite(PIN_Valve_SecondActuator, HIGH);
          delay(ValveOpenTime);
          digitalWrite(PIN_Valve_SecondActuator, LOW);
          delay(ValveCloseTime);
        }
        else{
          digitalWrite(PIN_Valve_Receiver, LOW);
        }
      }
      else{
        ReceiverPumped = false;
        digitalWrite(PIN_Valve_Pump, HIGH);
        digitalWrite(PIN_Valve_Receiver, LOW);
        MotorOn(150);
      }
    }
  }
  else if(Set_Pressure_2  <  Pressure_SecondActuator && Set_Pressure_2 > 101.0){
    if(Set_Pressure_2 + 5 < Pressure_SecondActuator){
      ReducePressure(2, ValveOpenTime);
    }
  }
  
  //-------------------------- THIRD ACTUATOR INFLATING ----------------------  
  if(Set_Pressure_3  >  Pressure_ThirdActuator && Set_Pressure_3 > 101.0){
    if(!ReceiverPumped){
      if(Set_Pressure_3 + 50.0 > Pressure_Receiver){
        digitalWrite(PIN_Valve_Pump, HIGH);
        digitalWrite(PIN_Valve_Receiver, LOW);
        MotorOn(150);
      }
      else{
        ReceiverPumped = true;
        digitalWrite(PIN_Valve_Pump, LOW);
        digitalWrite(PIN_Valve_Receiver, LOW);
        MotorOff();
      }
    }
    else{
      if(Set_Pressure_3 + 30.0 < Pressure_Receiver){
        digitalWrite(PIN_Valve_Pump, LOW);
        MotorOff();
        digitalWrite(PIN_Valve_Receiver, HIGH);
        if(Set_Pressure_3 - 5 > Pressure_ThirdActuator){
          digitalWrite(PIN_Valve_ThirdActuator, HIGH);
          delay(ValveOpenTime);
          digitalWrite(PIN_Valve_ThirdActuator, LOW);
          delay(ValveCloseTime);
        }
        else{
          digitalWrite(PIN_Valve_Receiver, LOW);
        }
      }
      else{
        ReceiverPumped = false;
        digitalWrite(PIN_Valve_Pump, HIGH);
        digitalWrite(PIN_Valve_Receiver, LOW);
        MotorOn(150);
      }
    }
  }
  else if(Set_Pressure_3  <  Pressure_ThirdActuator && Set_Pressure_3 > 101.0){
    if(Set_Pressure_3 + 5 < Pressure_ThirdActuator){
      ReducePressure(3, ValveOpenTime);
    }
  }
  */
  while(Serial.available()){
    char incomingChar = Serial.read();
    if(incomingChar >= '0' && incomingChar <= '6')
      Code +=incomingChar;
    else if(incomingChar == '\n'){
      if(Code >= "0" && Code <= "4"){
        DropAir(Code.toInt());
      }
      else if(Code == "5"){
        CreateVacuum(4,1000);
      }
      else if(Code == "6"){
        digitalWrite(PIN_Valve_Pump, HIGH);
        MotorOn(160);
        delay(5000);
        MotorOff();
      }
      else{
        Serial.println("WRONG COMMAND, PLEASE CHOOSE CORRECT NUMBER:");
        Serial.println("    0 - DROP AIR FROM RECEIVER");
        Serial.println("    1 - DROP AIR FROM THE FIRST ACTUATOR");
        Serial.println("    2 - DROP AIR FROM THE SECOND ACTUATOR");
        Serial.println("    3 - DROP AIR FROM THE THIRD ACTUATOR");
        Serial.println("    4 - DROP AIR FROM ALL ACTUATORS");  
        Serial.println("    5 - CREATE VACUUM IN THE FIRST ACTUATORS");
        Serial.println("    6 - PUMP AIR TO RECEIVER FOR 5 SEC");                              
      }
    Code = "";  
    }
  }

  lcd.setCursor(0,0);
  lcd.print("P_R:");
  lcd.print(Pressure_Receiver);
  lcd.print(" kPa");
  lcd.setCursor(0,1);
  lcd.print("P1,Ps1:");
  lcd.print(Pressure_FirstActuator);
  lcd.print(", ");
  lcd.print(Set_Pressure_1);
  lcd.setCursor(0,2);
  lcd.print("P2,Ps2:");
  lcd.print(Pressure_SecondActuator);
  lcd.print(", ");
  lcd.print(Set_Pressure_2);
  lcd.setCursor(0,3);
  lcd.print("P2,Ps2:");
  lcd.print(Pressure_ThirdActuator);
  lcd.print(", ");
  lcd.print(Set_Pressure_3);
  
  Serial.print("P_R = ");
  Serial.print(Pressure_Receiver);
  Serial.print("\t");
  Serial.print("P_1 = ");
  Serial.print(Pressure_FirstActuator);
  Serial.print("\t");
  Serial.print("P_S1 = ");
  Serial.print(Set_Pressure_1);
  Serial.print("\t");
  Serial.print("P_2 = ");
  Serial.print(Pressure_SecondActuator);
  Serial.print("\t");
  Serial.print("P_S2 = ");
  Serial.print(Set_Pressure_2);
  Serial.print("\t");
  Serial.print("P_3 = ");
  Serial.print(Pressure_ThirdActuator);
  Serial.print("\t");
  Serial.print("P_S3 = ");
  Serial.print(Set_Pressure_3);
  Serial.print("\t");
  Serial.print(Max_Pressure);
  Serial.println("\t");

}
