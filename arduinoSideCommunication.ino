#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

int positionFlag=0;
int photoFlag=0;
int computer = 0;
int ledPin = 13;            // choose the pin for the LED

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_StepperMotor *myMotor = AFMS.getStepper(200, 2);

void setup() { 
  Serial.begin(9600);
  AFMS.begin();  // create with the default frequency 1.6KHz 
  myMotor->setSpeed(10);
  pinMode(ledPin, OUTPUT);      // declare LED as output
  digitalWrite(ledPin, LOW); // turn LED OFF
}
 
void loop() {  
  if (Serial.available() > 0) {
    // read the incoming byte into an int:
    computer = Serial.read()-'0';
    digitalWrite(ledPin, HIGH);  // turn LED ON
  }
  
  if(computer == 1){ 
    myMotor->step(1, FORWARD, SINGLE);
    computer=0;
    Serial.write('1');
  }
  
}

