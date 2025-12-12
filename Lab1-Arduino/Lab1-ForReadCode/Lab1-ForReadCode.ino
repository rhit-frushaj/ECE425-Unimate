/************************************
   Lab1-ForReadCode.ino
   Andrew Frush, Val Rumzis, 12.11.25
   ***********************************
   This program will introduce using the stepper motor library to create motion algorithms for the robot.
   The motions will be forward, reverse, pivot, spin, turn, stop, go to angle, go to goal, move in a circle, square, and figure eight.
   
   The primary functions created are:
   forward   - Moves the robot forwards by rotating the wheels forwards for 1500 pulses.
   reverse   - Moves the robot backwards by rotating the wheels backwards for 1500 pulses.
   pivot     - Pivots robots by rotating the corresponding wheel 1012 pulses. For a pivot CCW, the right wheel is powered and for a pivot CW, the left wheel is powered.
   spin      - Spins robots by rotating both wheels in opposite directions. For CW, right wheel is reverse and left wheel is forwards. For CCW, left wheel is reverse and right wheel is forwards.
   turn      - Turns robots by rotating the outside wheel 1600 pulses. The inside wheel moves every third pulse, therefore moving inner wheel 1/3rd the speed.
   stop      - Stops the motors from running by using the built in stop() method from the AccelStepper library
   goToAngle - Moves the robot to face in an angle relative to it's starting position. Robot will turn CCW or CW based on which requires the shortest time. See function for visual example.
   goToGoalCm- Moves the robot to a position based on it's current position in centimeters. First rotates the robot then moves in a straight line directly to the goal. See function for visual example.
   squareCm  - Moves the robot in a square of a programable side length. Robot starts in the bottom of the square. See function for visual example.
   circleCm  - Moves the robot in a CW or CCW circle of some diameter in centimeters. Diameter is based on the centerline of the robot (between two wheeles).
   figure8   - Moves the robot in a figure eight starting at the center of the figure eight.
   
   NOTE: All functions in cm have an inches counterpart. For example, goToGoalCm for cm or goToGoalIn for in.
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
  Serial.begin(115200);
  delay(1000);
  
  //Initial Demo

  // forward();
  // delay(1000);
  // reverse();
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
  // delay(5000);
  // goToGoalIn(36, 48);
  // delay(5000);
  // goToGoalIn(-24,-24);
  // squareIn(36);
  figure8(36);

}

void loop() {
  // put your main code here, to run repeatedly:
}
/*
  Ensures: Moves the robot forwards by rotating the wheels forwards for 1500 pulses.
*/
void forward() {
  Serial.println("forward function");
  digitalWrite(redLED, HIGH);//turn on red LED
  digitalWrite(grnLED, LOW);//turn off green LED
  digitalWrite(ylwLED, LOW);//turn off yellow LED
  digitalWrite(ltDirPin, HIGH); // Enables the motor to move in a particular direction
  digitalWrite(rtDirPin, HIGH); // Enables the motor to move in a particular direction
  // Makes 1500 pulses 
  for (int x = 0; x < 1500; x++) {
    digitalWrite(rtStepPin, HIGH);
    digitalWrite(ltStepPin, HIGH);
    delayMicroseconds(stepTime);
    digitalWrite(rtStepPin, LOW);
    digitalWrite(ltStepPin, LOW);
    delayMicroseconds(stepTime);
  }
}

/*
  Ensures: Moves the robot backwards by rotating the wheels backwards for 1500 pulses.
*/
void reverse(){

  Serial.println("forward function");
  digitalWrite(redLED, HIGH);//turn on red LED
  digitalWrite(grnLED, LOW);//turn off green LED
  digitalWrite(ylwLED, LOW);//turn off yellow LED
  digitalWrite(ltDirPin, LOW); // Enables the motor to move in a particular direction
  digitalWrite(rtDirPin, LOW); // Enables the motor to move in a particular direction
  // Makes 1500 pulses 
  for (int x = 0; x < 1500; x++) {
    digitalWrite(rtStepPin, HIGH);
    digitalWrite(ltStepPin, HIGH);
    delayMicroseconds(stepTime);
    digitalWrite(rtStepPin, LOW);
    digitalWrite(ltStepPin, LOW);
    delayMicroseconds(stepTime);
  }

}

/*
  Ensures: Pivots robots by rotating the corresponding wheel 1012 pulses. For a pivot CCW, the right wheel is powered and for a pivot CW, the left wheel is powered.

  int direction: For a CCW, direction = 1, for a CW direction, direction = 0
*/
void pivot(int direction){
  digitalWrite(ltDirPin, HIGH); // Enables the motor to move in a particular direction
  digitalWrite(rtDirPin, HIGH); // Enables the motor to move in a particular direction
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
      delayMicroseconds(1000);
    }

  }

}

/*
  Ensures: Turns robots by rotating the outside wheel 1600 pulses. The inside wheel moves every third pulse, therefore moving inner wheel 1/3rd the speed.

  int direction: For a CW, direction = 1, for a CCW direction, direction = 0
*/
void turn(int direction){
  Serial.println("turn function");
  digitalWrite(redLED, HIGH);//turn on red LED
  digitalWrite(grnLED, LOW);//turn off green LED
  digitalWrite(ylwLED, LOW);//turn off yellow LED

  digitalWrite(ltDirPin, HIGH); // Enables the motor to move in a particular direction
  digitalWrite(rtDirPin, HIGH); // Enables the motor to move in a particular direction
 
  if (direction == 1){ // if CW
    for (int x = 0; x < 1600; x++) { // 1600 pulses on outside wheel
      digitalWrite(rtStepPin, HIGH);
      digitalWrite(ltStepPin, HIGH);
      delayMicroseconds(stepTime);
      if (x % 3 ==0){ // every third pulse
      digitalWrite(rtStepPin, LOW);   //makes the right wheel reset its pins 1/3rd rate of left
      }
      digitalWrite(ltStepPin, LOW);
      delayMicroseconds(stepTime);
    } 
  } else { //if CCW
    for (int x = 0; x < 1600; x++) { // 1600 pulses on outside wheel
      digitalWrite(rtStepPin, HIGH);
      digitalWrite(ltStepPin, HIGH);
      delayMicroseconds(stepTime);
      if (x % 3 ==0){   // every third pulse
      digitalWrite(ltStepPin, LOW); //makes the left wheel reset its pins 1/3rd rate of right
      }
      digitalWrite(rtStepPin, LOW);
      delayMicroseconds(stepTime);
    }
  }
  delay(1000); // One second delay
}
/*
   Ensures: Spins robots by rotating both wheels in opposite directions. For CW, right wheel is reverse and left wheel is forwards. For CCW, left wheel is reverse and right wheel is forwards.

   bool CW: If true, ropot spins CW. If false, spins CCW.

*/
void spin(bool CW) {
 
  Serial.println("spin function");
  digitalWrite(redLED, HIGH);//turn on red LED
  digitalWrite(grnLED, LOW);//turn off green LED
  digitalWrite(ylwLED, LOW);//turn off yellow LED
  digitalWrite(ltDirPin, HIGH); // Enables the motor to move in a particular direction
  digitalWrite(rtDirPin, HIGH); // Enables the motor to move in a particular direction
 
    if (CW) { // if CW, reverses right wheel
  digitalWrite(rtDirPin, LOW); // Enables the motor to move in opposite direction
 } else {// if CCW, reverses left wheel
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
  // reset pin direction
  if (CW) {
  digitalWrite(rtDirPin, HIGH); // Enables the motor to move in opposite direction
  } else {
 digitalWrite(ltDirPin, HIGH); // Enables the motor to move in opposite direction
 }
  delay(1000); // One second delay
}

/*
  Ensures: stops the motors from running by using the built in stop() method from the AccelStepper library.
*/
void stop(){
  stepperRight.stop();
  stepperLeft.stop();

}

/*
  Ensures: Moves the robot to face in an angle relative to it's starting position. Robot will turn CCW or CW based on which requires the shortest time.

  Example rotation:

   (30 deg)\    ^ X
            \   |
             \  |
              \ |
      <---------O (robot origin - facing in positive x direction)       
      Y

  float thetag: Angle goal to turn to, positive only between 0 -> 360 degrees.
*/
void goToAngle(float thetag){ // degrees
  float drift = 1.02; //added to artificially correct for the overshoot/ compounding error during square
  thetag *= drift;
  digitalWrite(rtDirPin, HIGH); //sets both motors forward
  digitalWrite(ltDirPin, HIGH);
  float thetac = 0;
  if (thetag>180){ //if angle is greater than 180 want to rotate CCW, right motor reverses
    digitalWrite(rtDirPin, LOW);
    thetag = abs(thetag-360); //calculates angle to rotate in opposite direction
  } else { 
    digitalWrite(ltDirPin, LOW); //otherwise rotates CW, left motor reverses
  }
  while (thetac < thetag){ //main stepping loop, stops when current angle is equal to or greater than target angle
      digitalWrite(rtStepPin, HIGH);
      digitalWrite(ltStepPin, HIGH);
      delayMicroseconds(stepTime);
      digitalWrite(rtStepPin, LOW);
      digitalWrite(ltStepPin, LOW);
      delayMicroseconds(stepTime);
      thetac += 1530.0/8600.0; //calculated amount the robot turns per one step of both motors in opposite directions
      delayMicroseconds(1000); //artificially added delay to help reduce overshoot/momentum
    }
}

/*
  Ensures: Moves the robot to a position based on it's current position in centimeters. First rotates the robot then moves in a straight line directly to the goal. 
           The coordinate system of the robot can be visualized below:

                ^ X
                |
                |
                |
      <---------|        
      Y
    
  float xg: X coordinate of the desired ending location in centimeters.
  float yg: Y coordinate of the desired ending location in centimeters.
*/
void goToGoalCm(float xg, float yg){ // cm
  Serial.println("Going to goal");
  digitalWrite(rtDirPin, HIGH); //sets to drive forward
  digitalWrite(ltDirPin, HIGH);
  float xc = 0; //variable for tracking current position
  float yc = 0;
  float dc=0;
  float thetag = atan(yg/xg); //calculate desired angle
  if ((xg <0 && yg < 0) || (xg > 0 && yg < 0)){ // if point lies in 3 or 4th quadrant must add 180 degrees to angle
    thetag = thetag+ PI;
  }
  goToAngle(thetag*(180/PI)); //rotate robot to proper angle in degrees
  float dg = sqrt((xc-xg)*(xc-xg)+(yc-yg)*(yc-yg)); //calcule distance in cm for robot to travel

  digitalWrite(rtDirPin, HIGH); //set both motors back to drive forward after rotation occured
  digitalWrite(ltDirPin, HIGH);

  delay(100);
  while (dg >= dc){ //main loop advancing robot until it's current position equales or exceedes the target distance
    digitalWrite(rtStepPin, HIGH);
    digitalWrite(ltStepPin, HIGH);
    delayMicroseconds(stepTime);
    digitalWrite(rtStepPin, LOW);
    digitalWrite(ltStepPin, LOW);
    delayMicroseconds(stepTime);
    dc += (8.5*PI)/800; // every loop is a step of the motor, this line adds the proper amount of distance traveled in one step to the current position in cm
    delayMicroseconds(1000); //artificially added delay to slow down speed further

  }
}

/*
  Ensures: Moves the robot to a position based on it's current position in inches. First rotates the robot then moves in a straight line directly to the goal. 
           The coordinate system of the robot can be visualized below:

                ^ X
                |
                |
                |
      <---------|        
      Y
    
  float xg: X coordinate of the desired ending location in inches.
  float yg: Y coordinate of the desired ending location in inches.
*/
void goToGoalIn(float xg, float yg){ // in
  float xgcm=xg*2.54; //conversion to cm
  float ygcm=yg*2.54;
  goToGoalCm(xgcm,ygcm);
}

/*
 Ensures: Moves the robot in a square of a programable side length. Robot starts in the bottom of the square and would follow the path shown below:

  visits (Start (1) -> 2 -> 3 -> 4 -> End (1))
  
    3----<----2
    |         |
    v         ^
    |         |
    4---->----1 (Start, End)
    
 float L: Side length of the square in centimeters. Only takes in a positive value for side length.
*/
void squareCm(float L){
  L = abs(L);
  goToGoalCm(L,0);
  for (int index = 0; index <3; index++)
  goToGoalCm(0,L);
}

/*
 Ensures: Moves the robot in a square of a programable side length. Robot starts in the bottom of the square and would follow the path shown below:

  visits (Start (1) -> 2 -> 3 -> 4 -> End (1))
  
    3----<----2
    |         |
    v         ^
    |         |
    4---->----1 (Start, End)
    
 float L: Side length of the square in Inches. Only takes in a positive value for side length.
*/
void squareIn(float L){
  L = abs(L);
  goToGoalIn(L,0);
  for (int index = 0; index <3; index++)
  goToGoalIn(0,L);
}

/*
  Ensures: Moves the robot in a CW or CCW circle of some diameter in centimeters. Diameter is based on the centerline of the robot (between two wheeles).

  float D: Represents the diameter of the circle to traverse in centimeters. 
           Postive Input:  CW Rotation
           Negative Input: CCW Rotation
*/
void circleCm(float D){
  Serial.println("circle function");
  digitalWrite(redLED, HIGH);//turn on red LED
  digitalWrite(grnLED, LOW);//turn off green LED
  digitalWrite(ylwLED, LOW);//turn off yellow LED

  digitalWrite(ltDirPin, HIGH); // Enables the motor to move forward
  digitalWrite(rtDirPin, HIGH);

  float absD = abs(D); // Use absolute value for calculations
  float circouter = PI * (absD + 21.5); //circumference in centimeters for outer wheel
  float circinner = PI * (absD - 21.5); //circumference in centimeters for inner wheel
  
  float correctionFactor = 0.99; //correction factor added to adjust, tuned so that start and stop at same position

  float outersteps = correctionFactor * circouter / (8.5 * PI / 800); //converts circumfrence to motor steps from cm
  float innersteps = correctionFactor * circinner / (8.5 * PI / 800);

  long positions[2];
  
  if(D < 0){ //CCW Case in which case the left wheel becomes the inner wheel
    positions[0] = (long) innersteps;  
    positions[1] = (long) outersteps;  
  } else {
    positions[0] = (long) outersteps;  
    positions[1] = (long) innersteps;  
  }

  steppers.moveTo(positions);
  bool running = true;
  while(running){
    running = steppers.run();
  }

  stepperRight.setCurrentPosition(0); //have to reset the ending position back to 0 since the moveTo command is not relative.
  stepperLeft.setCurrentPosition(0);
}

/*
  Ensures: Moves the robot in a CW or CCW circle of some diameter in inches. Diameter is based on the centerline of the robot (between two wheeles).

  float D: Represents the diameter of the circle to traverse in Inches. 
           Postive Input:  CW Rotation
           Negative Input: CCW Rotation
*/
void circleIn(float D){// in
  D = D*2.54;
  circleCm(D);
}

/*
 Ensures: Moves the robot in a figure eight starting at the center of the figure eight.

 float D: Diameter of the circles made in the figure eight in inches. 
          Positive Input: Performs a CW circle then a CCW circle
          Negative Input: Performs a CCW circle then a CW circle
*/
void figure8(float D){
  digitalWrite(ylwLED, HIGH);//turn off yellow LED
  circleIn(D);
  circleIn(-1*D);

}

