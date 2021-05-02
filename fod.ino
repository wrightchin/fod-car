#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Wire.h>

TinyGPSPlus gps;
//SoftwareSerial GPSserial(10, 11);
//SoftwareSerial BTserial(8, 9);

//Black Line
#define blackLinePinAnalogTR A12   //Top Right
#define blackLinePinTR 42         //Top Right
#define blackLinePinAnalogTL A13   //Top Left
#define blackLinePinTL 43         //Top Left
int  blackLineAnalogTR = 0; //top right
int  blackLineAnalogTL = 0; //top left

///////////////////////////////////////////////////////
//ultrasonic
#define echoPin 7
#define trigPin 6
#define echoPin2 4
#define trigPin2 3
unsigned long start_time = 0;
int done = 1;
long distance_in_cm;

unsigned long start_time2 = 0;
int done2 = 1;
long distance_in_cm2;

///////////////////////////////////////////////////////
//motor driver LM298 control pin
int pin1R = 8;
int pin2R = 9;
int pin1L = 10;
int pin2L = 11;

//interrupt pin
int interruptL1 = A5; // 2
int interruptL2 = A4; // 3

int interruptR1 = A0; // 18
int interruptR2 = A1; // 19
//////////////////////////////////////////////////////
// encoder parameter
//encoderValue
long encoderValueR = 0;
long encoderValueL = 0;

//Reset encoderValue
int encoderValueR_Reset = 0;
int encoderValueL_Reset = 0;

//encoderValue Diff
long encoderValueR_Diff = 0;
long encoderValueL_Diff = 0;

int fod_check = 0;

void setup()
{
Serial.begin(9600);
Serial1.begin(9600); // gps 
Serial2.begin(9600); // bluetooth

// ultra
pinMode(echoPin, INPUT);
pinMode(trigPin, OUTPUT);
pinMode(echoPin2, INPUT);
pinMode(trigPin2, OUTPUT);

// blackline
pinMode(blackLinePinTR, INPUT);
pinMode(blackLinePinTL, INPUT);

// motor
pinMode(pin1L, OUTPUT);
pinMode(pin2L, OUTPUT);
pinMode(pin1R, OUTPUT);
pinMode(pin2R, OUTPUT);

  //Encoder
  //https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/
  pinMode(interruptL1, INPUT_PULLUP);
  pinMode(interruptL2, INPUT_PULLUP);
  pinMode(interruptR1, INPUT_PULLUP);
  pinMode(interruptR2, INPUT_PULLUP);
  //setup interrupt
  attachInterrupt(digitalPinToInterrupt(interruptL1), countL, FALLING);
  attachInterrupt(digitalPinToInterrupt(interruptR1), countR, FALLING);

}

void loop() {
  
  InputCapture();
  measure_distance();
  measure_distance2();
  power();
  Serial.print(distance_in_cm);
  Serial.print(" ");
  Serial.print(distance_in_cm2);
  Serial.println("");
//  delay(500);

  Serial.print("|TR(blackline) = ");
  Serial.print(blackLineAnalogTR);
  Serial.print("|TL(blackline) = ");
  Serial.print(blackLineAnalogTL);
  Serial.print(" ");
  

  while (Serial1.available() > 0) {
    if (gps.encode(Serial1.read())) {
      displayInfo();
      
      if (millis() > 5000 && gps.charsProcessed() < 10) {
        Serial.println(F("No GPS detected: check wiring."));
        while(true);
      }
    }
  }
}

void displayInfo() {
  if (gps.location.isValid()) {
    Serial.print(gps.location.lat(), 6);
    Serial.print(",");
    Serial.print(gps.location.lng(), 6);
    Serial.println("");
    
//    Serial2.print(gps.location.lat(), 6);
//    Serial2.print(",");
//    Serial2.print(gps.location.lng(), 6);
//    Serial2.print(",");
    
  }
  else {
    Serial.println(F("INVALID"));
  }

  if (gps.location.isValid()) {
    Serial2.print(gps.location.lat(), 6);
    Serial2.print(",");
    Serial2.print(gps.location.lng(), 6);
    Serial2.print(",");
  } 
  else {
    Serial2.print("22.328491");
    Serial2.print(",");
    Serial2.print("114.155647");
    Serial2.print(",");
  }
  
  if ((distance_in_cm < 20) || (distance_in_cm2 < 20)) {
      fod_check = 1;
      stopCar();
  }
  else {
    fod_check = 0;
  }
    Serial.print("fod_check ");
    Serial.print(fod_check);
    
    Serial2.print(fod_check);
    Serial2.print(";");
}

void measure_distance() {
  long duration;
  
  if (done) {     
    // reset start_time only if the distance has been measured 
    // in the last invocation of the method
    done = 0;
    start_time = millis();
    digitalWrite(trigPin, LOW);
  }
  
  if (millis() > start_time + 2) { 
    digitalWrite(trigPin, HIGH);
  }
  
  if (millis() > start_time + 10) {
    digitalWrite(trigPin, LOW);
    duration = pulseIn(echoPin, HIGH);
    distance_in_cm = (duration / 2.0) / 29.1;
    done = 1;
  }

}

  void measure_distance2() {
  long duration2;
  
  if (done2) {     
    // reset start_time only if the distance has been measured 
    // in the last invocation of the method
    done2 = 0;
    start_time2 = millis();
    digitalWrite(trigPin2, LOW);
  }
  
  if (millis() > start_time2 + 2) { 
    digitalWrite(trigPin2, HIGH);
  }
  
  if (millis() > start_time2 + 10) {
    digitalWrite(trigPin2, LOW);
    duration2 = pulseIn(echoPin2, HIGH);
    distance_in_cm2 = (duration2 / 2.0) / 29.1;
    done2 = 1;
  }
  
}

void turnCarOnsiteL() {
  digitalWrite(pin1R, 1);
  digitalWrite(pin2R, 0);
  digitalWrite(pin1L, 0);
  digitalWrite(pin2L, 1);
//  analogWrite(pin1R, 255);
//  analogWrite(pin2R, 0);
//  analogWrite(pin1L, 0);
//  analogWrite(pin2L, 255);
}

void turnCarOnsiteR() {
  digitalWrite(pin1R, 0);
  digitalWrite(pin2R, 1);
  digitalWrite(pin1L, 1);
  digitalWrite(pin2L, 0);
//  analogWrite(pin1R, 0);
//  analogWrite(pin2R, 255);
//  analogWrite(pin1L, 255);
//  analogWrite(pin2L, 0);
}
void forwardCar() {
//  digitalWrite(pin1R, 1);
//  digitalWrite(pin2R, 0);
//  digitalWrite(pin1L, 1);
//  digitalWrite(pin2L, 0);
  analogWrite(pin1R, 200);
  analogWrite(pin2R, 0);
  analogWrite(pin1L, 200);
  analogWrite(pin2L, 0);
}

void stopCar() {
//  digitalWrite(pin1R, 0);
//  digitalWrite(pin2R, 0);
//  digitalWrite(pin1L, 0);
//  digitalWrite(pin2L, 0);
  analogWrite(pin1R, 0);
  analogWrite(pin2R, 0);
  analogWrite(pin1L, 0);
  analogWrite(pin2L, 0);
}

void power() {
          if ((distance_in_cm < 20) || (distance_in_cm2 <20)) {
            stopCar();
          }
          
          if (!(blackLineAnalogTR < 100) && !(blackLineAnalogTL < 100)) {
            forwardCar();
          }
      
          // left adjust
          else if ((blackLineAnalogTR < 100) && !(blackLineAnalogTL < 100)) {
             turnCarOnsiteL();
          }
      
          // right adjust 
          else if (!(blackLineAnalogTR < 100) && !(blackLineAnalogTL < 100)) {
             turnCarOnsiteR();
          }
        
}

//Interrupt subroutine
void countL() {
  if (digitalRead(interruptL2)) {
    encoderValueL--;
  }
  else {
    encoderValueL++;
  }
}
void countR() {
  if (digitalRead(interruptR2)) {
    encoderValueR++;
  }
  else {
    encoderValueR--;
  }
}

void InputCapture() {
  //black
  blackLineAnalogTR = analogRead(blackLinePinAnalogTR);
  blackLineAnalogTL = analogRead(blackLinePinAnalogTL);
  
}