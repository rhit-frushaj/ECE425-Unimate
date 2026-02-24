/************************************
   LightFollowing_M7.ino
   Andrew Frush, Val Rumzis, 2/24/26
   ***********************************
   The following program is an implementation of a light following state machine with runaway/obstacle avoidance behavior. 
   The primary functions created are:

   checkLightReadings(Bool: print): This method reads the photoresistor voltages. Serial printing is togelable through printing
   moveToLight(): Based on the current light readings, moved the robot motors at a proportional speed, or no movement at all

   Other function were created as helper methods to make more complex behavior like inner and outer wall following.

  ************************************/

//include all necessary libraries
#include <Arduino.h>
#include <RPC.h>
#include <AccelStepper.h>  //include the stepper motor library
#include <MultiStepper.h>  //include multiple stepper motor library

//state LEDs connections
#define redLED 5            //red LED for displaying states
#define grnLED 6            //green LED for displaying states
#define ylwLED 7            //yellow LED for displaying states
#define enableLED 13        //stepper enabled LED
int leds[3] = { 5, 6, 7 };  //array of LED pin numbers

//define motor pin numbers
#define stepperEnable 48  //stepper enable pin on stepStick
#define rtStepPin 50      //right stepper motor step pin
#define rtDirPin 51       // right stepper motor direction pin
#define ltStepPin 52      //left stepper motor step pin
#define ltDirPin 53       //left stepper motor direction pin

AccelStepper stepperRight(AccelStepper::DRIVER, rtStepPin, rtDirPin);  //create instance of right stepper motor object (2 driver pins, low to high transition step pin 52, direction input pin 53 (high means forward)
AccelStepper stepperLeft(AccelStepper::DRIVER, ltStepPin, ltDirPin);   //create instance of left stepper motor object (2 driver pins, step pin 50, direction input pin 51)
MultiStepper steppers;                                                 //create instance to control multiple steppers at the same time

#define stepperEnTrue false  //variable for enabling stepper motor
#define stepperEnFalse true  //variable for disabling stepper motor
#define max_speed 1636       //maximum stepper motor speed
#define max_accel 10000      //maximum motor acceleration

int pauseTime = 2500;  //time before robot moves
int stepTime = 500;    //delay time between high and low on step pin
int wait_time = 1000;  //delay for printing data

float lightLeftVolt = 0; //Light voltages, updated periodically
float lightRightVolt = 0;

// Data structure for sensor readings
struct sensors {
  int front;
  int back;
  int left;
  int right;

  MSGPACK_DEFINE_ARRAY(front, back, left, right);
} dist{};

void setup() {

  Serial.begin(115200);
  init_stepper();                   //Initialize all motor control
  init_sensors();                   //Initialize the Lidar Sensors
  if (RPC.cpu_id() == CM7_CPUID) {  //Sets up RPC for multicore
    blink(ylwLED, 100);
  } else {
    blink(redLED, 100);
  }
  delay(500);
}

void loop() {


  //Update the sensor Readings (Lidar and Light)
  checkLightReadings(true);                    //true Print Results, false no print
  dist = RPC.call("lidarRead").as<sensors>();  //get sensor data
  stateMachine(); //Main Logic
  
}

/*
  Ensures: Controls the state of the robot. Transitioning between randomWander, RunAway, and followLight.
*/
void stateMachine(){
  //Checking for objects nearby 
  float x_result = -1.0 * x_vector();
  float y_result = -1.0 * y_vector();
  float magnitude = sqrt(x_result * x_result + y_result * y_result);


  //All State Logic based on stimuli
  if(magnitude >= 3.0){ //if object to close it runs away in opposite direction proporional to the object closeness
    runAway();
  }
  if(lightLeftVolt > 0.15 || lightRightVolt > 0.15){ //checks for light, if light it breaks into following
    moveToLight();
  } else{ //if object not too close and no light then random wanders
    randomWander();
  }
}

/*
  Ensures: Reads light readings from two photoresistors. Output is voltages. Saved to global variables
  Input Bool: True if want light voltages printed to terminal, False no printing
*/
void checkLightReadings(bool print) {
  lightLeftVolt = analogRead(A0) * (5.0 / 1023.0);
  lightRightVolt = analogRead(A1) * (5.0 / 1023.0);

  if (print) {
    Serial.print("Left: ");
    Serial.print(lightLeftVolt);
    Serial.print(" Right: ");
    Serial.println(lightRightVolt);
  }
}


//function to set all stepper motor variables, outputs and LEDs
void init_stepper() {
  pinMode(rtStepPin, OUTPUT);                   //sets pin as output
  pinMode(rtDirPin, OUTPUT);                    //sets pin as output
  pinMode(ltStepPin, OUTPUT);                   //sets pin as output
  pinMode(ltDirPin, OUTPUT);                    //sets pin as output
  pinMode(stepperEnable, OUTPUT);               //sets pin as output
  digitalWrite(stepperEnable, stepperEnFalse);  //turns off the stepper motor driver
  pinMode(enableLED, OUTPUT);                   //set enable LED as output
  digitalWrite(enableLED, LOW);                 //turn off enable LED
  pinMode(redLED, OUTPUT);                      //set red LED as output
  pinMode(grnLED, OUTPUT);                      //set green LED as output
  pinMode(ylwLED, OUTPUT);                      //set yellow LED as output
  digitalWrite(redLED, HIGH);                   //turn on red LED
  digitalWrite(ylwLED, HIGH);                   //turn on yellow LED
  digitalWrite(grnLED, HIGH);                   //turn on green LED
  delay(pauseTime / 5);                         //wait 0.5 seconds
  digitalWrite(redLED, LOW);                    //turn off red LED
  digitalWrite(ylwLED, LOW);                    //turn off yellow LED
  digitalWrite(grnLED, LOW);                    //turn off green LED

  stepperRight.setMaxSpeed(max_speed);         //set the maximum permitted speed limited by processor and clock speed, no greater than 4000 steps/sec on Arduino
  stepperRight.setAcceleration(max_accel);     //set desired acceleration in steps/s^2
  stepperLeft.setMaxSpeed(max_speed);          //set the maximum permitted speed limited by processor and clock speed, no greater than 4000 steps/sec on Arduino
  stepperLeft.setAcceleration(max_accel);      //set desired acceleration in steps/s^2
  steppers.addStepper(stepperLeft);            //add left motor to MultiStepper
  steppers.addStepper(stepperRight);           //add right motor to MultiStepper
  digitalWrite(stepperEnable, stepperEnTrue);  //turns on the stepper motor driver
  digitalWrite(enableLED, HIGH);               //turn on enable LED
}

/*
  Ensures: Checks light readings and then sets the proportional speed on the corresponding motor. Moved robot by 10 steps before recalculating. 
*/
void moveToLight() {
  digitalWrite(grnLED, LOW);
  digitalWrite(redLED, LOW);
  digitalWrite(ylwLED, HIGH);

  stepperLeft.setSpeed(0);
  stepperRight.setSpeed(0);

  // int rightSpeed = 500;
  // int leftSpeed = 500;
  int kp = 250;

  if (lightLeftVolt > 0.15) {
    stepperLeft.setSpeed(lightLeftVolt * kp);
  }
  if (lightRightVolt > 0.15) {
    stepperRight.setSpeed(lightRightVolt * kp);
  }

  for (int i = 0; i < 10; i++) {
    stepperRight.runSpeed();
    stepperLeft.runSpeed();
  }
}

/*
  Ensures: This method is universal for M4 and M7, blinks LED to know which core is initializing and calls RPC.Begin
           Useful for troubleshooting purposes.
*/
void blink(int led, int delaySeconds) {
  pinMode(led, OUTPUT);  //Commented OUT to speed up startup
  for (int i = 0; i < 10; i++) {
    digitalWrite(led, LOW);
    delay(delaySeconds);
    digitalWrite(led, HIGH);
    delay(delaySeconds);
  }
  RPC.begin();
  digitalWrite(led, LOW);
}

/*
  Ensures: Impliments a potential field protocol that uses the four lidar sensors to be repulsed from external objects. Uses lidar and calculated vector to turn an angle and move away.

  Output: Robot motion based on external stimuli. 
*/
void runAway() {
  digitalWrite(redLED, 1);
  digitalWrite(ylwLED, 0);
  digitalWrite(grnLED, 0);

  if (dist.front == 0 && dist.back == 0 && dist.left != 0 && dist.right != 0) {  //special hard coded case for when there are two walls on the left and right of robot only
    forward(10);
    return;
  }

  if (dist.front != 0 && dist.back != 0 && dist.left == 0 && dist.right == 0) {  //special case for when two wall front and back only
    goToAngle(90);
    forward(10);
    return;
  }

  if (dist.front != 0 || dist.back != 0 || dist.left != 0 || dist.right != 0) {  //this if statement keeps it still if there is no reading
    // multiplies direction vector by -1 to ensure it is repelled
    float x_result = -1.0 * x_vector();
    float y_result = -1.0 * y_vector();

    float theta = atan2(y_result, x_result) * (180 / PI);
    if (theta < 0) {
      theta = 360 + theta;  //this converts the angle back to 0 -> 360
    }

    goToAngle(theta);
    forward(10);
  }
}

/*
  Ensures: Moves the robot forwards by rotating the wheels forwards for for some distance in cm.
  float distance: Distance to move forwards in cm.
*/
void forward(float distance) {
  digitalWrite(ltDirPin, HIGH);  // Enables the motor to move in a particular direction
  digitalWrite(rtDirPin, HIGH);  // Enables the motor to move in a particular direction

  int steps = (int)ceil((distance * 800.0) / (8.5 * PI));  //this converts based on wheel diameter to steps from distance in cm
  //Steps both motors forward for the num steps calculated for the input distance
  for (int x = 0; x < steps; x++) {
    digitalWrite(rtStepPin, HIGH);
    digitalWrite(ltStepPin, HIGH);
    delayMicroseconds(stepTime);
    digitalWrite(rtStepPin, LOW);
    digitalWrite(ltStepPin, LOW);
    delayMicroseconds(stepTime);
  }
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
void goToAngle(float thetag) {  // degrees
  // Serial.println("Made it to angle");
  float drift = 1.02;  //added to artificially correct for the overshoot/ compounding error during square
  thetag *= drift;
  digitalWrite(rtDirPin, HIGH);  //sets both motors forward
  digitalWrite(ltDirPin, HIGH);
  float thetac = 0;
  if (thetag > 180) {  //if angle is greater than 180 want to rotate CCW, right motor reverses
    digitalWrite(rtDirPin, LOW);
    thetag = abs(thetag - 360);  //calculates angle to rotate in opposite direction
  } else {
    digitalWrite(ltDirPin, LOW);  //otherwise rotates CW, left motor reverses
  }
  while (thetac < thetag) {  //main stepping loop, stops when current angle is equal to or greater than target angle
    // prevents this from being blocking, uses global variable to break free if necessary
    
    digitalWrite(rtStepPin, HIGH);
    digitalWrite(ltStepPin, HIGH);
    delayMicroseconds(stepTime);
    digitalWrite(rtStepPin, LOW);
    digitalWrite(ltStepPin, LOW);
    delayMicroseconds(stepTime);
    thetac += 1530.0 / 8600.0;  //calculated amount the robot turns per one step of both motors in opposite directions
    delayMicroseconds(1000);    //artificially added delay to help reduce overshoot/momentum
  }
}

/*
  Ensures: Returns the X component of the vector that points twoards the object it sees

  Returns: Float representing the force 
*/
float x_vector() {
  float kp = 100;                                                   //scaler gain
  float front_force = (dist.front > 0) ? (1.0 / dist.front) : 0.0;  //stops dividing by 0 which would be an error
  float back_force = (dist.back > 0) ? (1.0 / dist.back) : 0.0;
  return kp * (front_force - back_force);
}

/*
  Ensures: Returns the Y component of the vector that points twoards the object it sees
*/
float y_vector() {
  float kp = 100;                                                //scaler gain
  float left_force = (dist.left > 0) ? (1.0 / dist.left) : 0.0;  //stops dividing by 0 which would be an error
  float right_force = (dist.right > 0) ? (1.0 / dist.right) : 0.0;
  return kp * (left_force - right_force);
}

/*
  Ensures: Moves the robot in an apparently random fashion when viewed. Non-Blocking. It runs between random angles, and random forwards
*/
void randomWander() {
  //sets lights to green only on
  digitalWrite(grnLED, HIGH);
  digitalWrite(redLED, LOW);
  digitalWrite(ylwLED, LOW);

  int randomBehavior = random(0, 4);  // chooses random behavior 0, 1, 2, 3
  if (randomBehavior <= 1) {
    double randomAngle = 1.0 * random(0, 361);  // chooses random angle to go to from 0-360 deg
    goToAngle(randomAngle);                     // goes to random angle
  } else {
    forward(25);
  }
}
