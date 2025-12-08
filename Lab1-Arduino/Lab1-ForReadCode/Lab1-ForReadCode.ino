/*
ADD approptiate heading


*/

//includew all necessary libraries
#include <Arduino.h>//include for PlatformIO Ide
#include <AccelStepper.h>//include the stepper motor library
#include <MultiStepper.h>//include multiple stepper motor library

//state LEDs connections
#define redLED 5            //red LED for displaying states
#define grnLED 6            //green LED for displaying states
#define ylwLED 7            //yellow LED for displaying states
#define enableLED 13        //stepper enabled LED
int leds[3] = {5,6,7};      //array of LED pin numbers

//define motor pin numbers
#define stepperEnable 48    //stepper enable pin on stepStick 
#define rtStepPin 50 //right stepper motor step pin 
#define rtDirPin 51  // right stepper motor direction pin 
#define ltStepPin 52 //left stepper motor step pin 
#define ltDirPin 53  //left stepper motor direction pin 

AccelStepper stepperRight(AccelStepper::DRIVER, rtStepPin, rtDirPin);//create instance of right stepper motor object (2 driver pins, low to high transition step pin 52, direction input pin 53 (high means forward)
AccelStepper stepperLeft(AccelStepper::DRIVER, ltStepPin, ltDirPin);//create instance of left stepper motor object (2 driver pins, step pin 50, direction input pin 51)
MultiStepper steppers;//create instance to control multiple steppers at the same time

#define stepperEnTrue false //variable for enabling stepper motor
#define stepperEnFalse true //variable for disabling stepper motor
#define max_speed 1000 //maximum stepper motor speed
#define max_accel 10000 //maximum motor acceleration

int pauseTime = 2500;   //time before robot moves
int stepTime = 500;     //delay time between high and low on step pin
int wait_time = 1000;   //delay for printing data

//define encoder pins
#define LEFT 0        //left encoder
#define RIGHT 1       //right encoder
const int ltEncoder = 18;        //left encoder pin (Mega Interrupt pins 2,3 18,19,20,21)
const int rtEncoder = 19;        //right encoder pin (Mega Interrupt pins 2,3 18,19,20,21)
volatile long encoder[2] = {0, 0};  //interrupt variable to hold number of encoder counts (left, right)
int lastSpeed[2] = {0, 0};          //variable to hold encoder speed (left, right)
int accumTicks[2] = {0, 0};         //variable to hold accumulated ticks since last reset

float currentAngle = 0;

//function to set all stepper motor variables, outputs and LEDs
void init_stepper(){
  pinMode(rtStepPin, OUTPUT);//sets pin as output
  pinMode(rtDirPin, OUTPUT);//sets pin as output
  pinMode(ltStepPin, OUTPUT);//sets pin as output
  pinMode(ltDirPin, OUTPUT);//sets pin as output
  pinMode(stepperEnable, OUTPUT);//sets pin as output
  digitalWrite(stepperEnable, stepperEnFalse);//turns off the stepper motor driver
  pinMode(enableLED, OUTPUT);//set enable LED as output
  digitalWrite(enableLED, LOW);//turn off enable LED
  pinMode(redLED, OUTPUT);//set red LED as output
  pinMode(grnLED, OUTPUT);//set green LED as output
  pinMode(ylwLED, OUTPUT);//set yellow LED as output
  digitalWrite(redLED, HIGH);//turn on red LED
  digitalWrite(ylwLED, HIGH);//turn on yellow LED
  digitalWrite(grnLED, HIGH);//turn on green LED
  delay(pauseTime / 5); //wait 0.5 seconds
  digitalWrite(redLED, LOW);//turn off red LED
  digitalWrite(ylwLED, LOW);//turn off yellow LED
  digitalWrite(grnLED, LOW);//turn off green LED

  stepperRight.setMaxSpeed(max_speed);//set the maximum permitted speed limited by processor and clock speed, no greater than 4000 steps/sec on Arduino
  stepperRight.setAcceleration(max_accel);//set desired acceleration in steps/s^2
  stepperLeft.setMaxSpeed(max_speed);//set the maximum permitted speed limited by processor and clock speed, no greater than 4000 steps/sec on Arduino
  stepperLeft.setAcceleration(max_accel);//set desired acceleration in steps/s^2
  steppers.addStepper(stepperLeft);//add left motor to MultiStepper
  steppers.addStepper(stepperRight);//add right motor to MultiStepper
  digitalWrite(stepperEnable, stepperEnTrue);//turns on the stepper motor driver
  digitalWrite(enableLED, HIGH);//turn on enable LED
}

void setup() {
  // put your setup code here, to run once:
  init_stepper();
  Serial.begin(9600);

  //Initial Demo

  // reverse();
  // delay(1000);
  // forward();
  // delay(1000);
  // stop();
  // pivot(1); //left
  // delay(1000);
  // pivot(0); //right
  // delay(1000);
  // turn(1); //turn left
  // delay(1000);
  // turn(0); //turn right
  // delay(1000);
  // spin(true); //left
  // delay(1000);
  // spin(false);
  // delay(1000);
  // stop();

  //Advanced Features Demo

  goToAngle(90);
  goToGoalIn(-12, 36);
  squareIn(2);

  // float theta = 0;
  // for (int i = 0; i < 360; i += 45){
  //   goToAngle(i*1.0);
  //   delay(1000);
  // }
  // goToAngle(90.0);
}

void loop() {
  // put your main code here, to run repeatedly:

}

void forward() {
  Serial.println("forward function");
  digitalWrite(redLED, HIGH);//turn on red LED
  digitalWrite(grnLED, LOW);//turn off green LED
  digitalWrite(ylwLED, LOW);//turn off yellow LED
  digitalWrite(ltDirPin, HIGH); // Enables the motor to move in a particular direction
  digitalWrite(rtDirPin, HIGH); // Enables the motor to move in a particular direction
  // Makes 800 pulses for making one full cycle rotation
  for (int x = 0; x < 1500; x++) {
    digitalWrite(rtStepPin, HIGH);
    digitalWrite(ltStepPin, HIGH);
    delayMicroseconds(stepTime);
    digitalWrite(rtStepPin, LOW);
    digitalWrite(ltStepPin, LOW);
    delayMicroseconds(stepTime);
  }
}

void reverse(){

  Serial.println("forward function");
  digitalWrite(redLED, HIGH);//turn on red LED
  digitalWrite(grnLED, LOW);//turn off green LED
  digitalWrite(ylwLED, LOW);//turn off yellow LED
  digitalWrite(ltDirPin, LOW); // Enables the motor to move in a particular direction
  digitalWrite(rtDirPin, LOW); // Enables the motor to move in a particular direction
  // Makes 800 pulses for making one full cycle rotation
  for (int x = 0; x < 1500; x++) {
    digitalWrite(rtStepPin, HIGH);
    digitalWrite(ltStepPin, HIGH);
    delayMicroseconds(stepTime);
    digitalWrite(rtStepPin, LOW);
    digitalWrite(ltStepPin, LOW);
    delayMicroseconds(stepTime);
  }

}

// 1 for pivot left, 0 for pivot right
void pivot(int direction){
  int stepCount = 1012;
  if (direction ==  1){
    for (int x = 0; x < stepCount; x++) {
      digitalWrite(rtStepPin, HIGH);
      delayMicroseconds(stepTime);
      digitalWrite(rtStepPin, LOW);
      delayMicroseconds(stepTime);
      delayMicroseconds(1000);
    }
  } else {
    for (int x = 0; x < stepCount; x++) {
      digitalWrite(ltStepPin, HIGH);
      delayMicroseconds(stepTime);
      digitalWrite(ltStepPin, LOW);
      delayMicroseconds(stepTime);
    }

  }

}

void turn(int direction){
  Serial.println("turn function");
  digitalWrite(redLED, HIGH);//turn on red LED
  digitalWrite(grnLED, LOW);//turn off green LED
  digitalWrite(ylwLED, LOW);//turn off yellow LED

  digitalWrite(ltDirPin, HIGH); // Enables the motor to move in a particular direction
  digitalWrite(rtDirPin, HIGH); // Enables the motor to move in a particular direction
 
  if (direction == 1){
    for (int x = 0; x < 1212; x++) {
      digitalWrite(rtStepPin, HIGH);
      digitalWrite(ltStepPin, HIGH);
      delayMicroseconds(stepTime);
      if (x % 8 ==0){
      digitalWrite(rtStepPin, LOW);
      }
      digitalWrite(ltStepPin, LOW);
      delayMicroseconds(stepTime);
    } 
    // Makes 800 pulses for making one full cycle rotation
  } else {
    for (int x = 0; x < 1212; x++) {
      digitalWrite(rtStepPin, HIGH);
      digitalWrite(ltStepPin, HIGH);
      delayMicroseconds(stepTime);
      if (x % 8 ==0){
      digitalWrite(ltStepPin, LOW);
      }
      digitalWrite(rtStepPin, LOW);
      delayMicroseconds(stepTime);
    }
  }
  delay(1000); // One second delay
}

void spin(bool CW) {
 
  Serial.println("spin function");
  digitalWrite(redLED, HIGH);//turn on red LED
  digitalWrite(grnLED, LOW);//turn off green LED
  digitalWrite(ylwLED, LOW);//turn off yellow LED
  digitalWrite(ltDirPin, HIGH); // Enables the motor to move in a particular direction
  digitalWrite(rtDirPin, HIGH); // Enables the motor to move in a particular direction
 
    if (CW) {
  digitalWrite(rtDirPin, LOW); // Enables the motor to move in opposite direction
 } else {
 digitalWrite(ltDirPin, LOW); // Enables the motor to move in opposite direction
 }
 // Makes 800 pulses for making one full cycle rotation
  for (int x = 0; x < 800; x++) {
    digitalWrite(rtStepPin, HIGH);
    digitalWrite(ltStepPin, HIGH);
    delayMicroseconds(stepTime);
    digitalWrite(rtStepPin, LOW);
    digitalWrite(ltStepPin, LOW);
    delayMicroseconds(stepTime);
  }
  if (CW) {
  digitalWrite(rtDirPin, HIGH); // Enables the motor to move in opposite direction
  } else {
 digitalWrite(ltDirPin, HIGH); // Enables the motor to move in opposite direction
 }
  delay(1000); // One second delay
}

void stop(){
  digitalWrite(rtStepPin, LOW);
  digitalWrite(ltStepPin, LOW);

}

// void goToAngle(float thetag){ // degrees
//   digitalWrite(rtDirPin, HIGH);
//   digitalWrite(ltDirPin, HIGH);
//   float thetac = 0;
//   if (thetag>180){
//     digitalWrite(rtDirPin, LOW);
//     thetag=thetag-180;
//   } else {
//     digitalWrite(ltDirPin, LOW);
//   }
//   while (thetac < thetag){
//       digitalWrite(rtStepPin, HIGH);
//       digitalWrite(ltStepPin, HIGH);
//       delayMicroseconds(stepTime);
//       digitalWrite(rtStepPin, LOW);
//       digitalWrite(ltStepPin, LOW);
//       delayMicroseconds(stepTime);
//       thetac = thetac + .08895;
//     }
// }

void goToAngle(float thetag) { // degrees
  float thetac = currentAngle;

  if (thetag > 180) {
    digitalWrite(rtDirPin, LOW);  // reverse right wheel
    digitalWrite(ltDirPin, HIGH); // left wheel forward
    thetag -= 180;
  } else {
    digitalWrite(ltDirPin, LOW);  // reverse left wheel
    digitalWrite(rtDirPin, HIGH); // right wheel forward
  }

  // Now start turning
  int step = 0;
  int stepSpeed = 1000;
  while (thetac < thetag){
      digitalWrite(rtStepPin, HIGH);
      digitalWrite(ltStepPin, HIGH);
      delayMicroseconds(stepTime);
      digitalWrite(rtStepPin, LOW);
      digitalWrite(ltStepPin, LOW);
      delayMicroseconds(stepTime);
      thetac += 1530.0/8600.0;
      step++;
      Serial.println(step);
      Serial.println(thetac);
      delayMicroseconds(stepSpeed); //way to control speed of motors
  }
  currentAngle = thetac;

}


void goToGoalCm(float xg, float yg){ // cm
  digitalWrite(rtDirPin, HIGH);
  digitalWrite(ltDirPin, HIGH);
  float xc = 0;
  float yc = 0;
  float dc=0;
  float thetag = atan(xg/yg);
  goToAngle(thetag);
  float dg = sqrt((xc-xg)*(xc-xg)+(yc-yg)*(yc-yg));
  while (dg>dc){
    digitalWrite(rtStepPin, HIGH);
    digitalWrite(ltStepPin, HIGH);
    delayMicroseconds(stepTime);
    digitalWrite(rtStepPin, LOW);
    digitalWrite(ltStepPin, LOW);
    delayMicroseconds(stepTime);
    dc = dc + (8.5*PI)/800; // cm
  }
}

void goToGoalIn(float xg, float yg){ // in
  float xgcm=xg/2.54;
  float ygcm=yg/2.54;
  goToGoalCm(xgcm,ygcm);
}

void squareCm(int L){ // cm
  goToGoalCm(L,0);
  goToGoalCm(0,L);
  goToGoalCm(0,L);
  goToGoalCm(0,L);
}

void squareIn(float L){ // in
  goToGoalIn(L,0);
  goToGoalIn(0,L);
  goToGoalIn(0,L);
  goToGoalIn(0,L);
}
