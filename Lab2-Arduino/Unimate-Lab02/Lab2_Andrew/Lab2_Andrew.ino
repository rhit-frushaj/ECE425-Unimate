/************************************
   Unimate-Lab02.ino
   Andrew Frush, Val Rumzis, 1.9.25
   ***********************************
   The following program is an implimentation of two basic mobile robot behavior models. The first introduces a fleeing mechanical (potential fields) and the other a 
   follow mechanic when with an object in front of the robot. Both have a collide mechanic stopping the robot when an object appears too close in front of the robot. 
   
   The primary functions created are:
   randomWander(): The robot randomly moves forward, backward, spins, and turns turns at random amounts. Motor speed varies slightly.  
   collide(): Stops the robot when a random object appears in front of the robot. Based on a static "too close" distance parameter.
   runAway(): Impliments potential fields to repulse the robot away from walls and go in free directions. Utilizes goToGoal and goToAngel + lidar sensors.
   follow():
   smartWander():
   smartFollow:

************************************/

//includew all necessary libraries
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

uint8_t currentState = 3;  //Tracks the state robot is currently in
// State Table
// 0 = randomWander
// 1 = angryKid
// 2 = shyKid
// 3 = curiousKid
// 4 = ...

bool running = true;  // global variable to keep whether the robot should be running or not, controls blocking
// // Data to keep the lidar sensor data - taken from example code

struct lidar {
  int front;
  int back;
  int left;
  int right;
  int sonarLeft;
  int sonarRight;

  MSGPACK_DEFINE_ARRAY(front, back, left, right, sonarLeft, sonarRight);
} dist{};


void setup() {
  Serial.begin(115200);
  delay(1000);

  init_stepper();

  if (RPC.cpu_id() == CM7_CPUID) {
    blink(ylwLED, 100);
  } else {
    blink(redLED, 100);
  }
  RPC.bind("collide", collide);
  // RPC.bind("runAway", runAway);

  delay(500);
}

void loop() {

  // Read lidar data from M4
  dist = RPC.call("lidarRead").as<lidar>();
  running = true;
  // delay(50); //delay on sensor reading, can be changed if needbe
  // smartWander();
  // smartFollow();
  follow();
  // Serial.println(
  //   String("Sensor Values (Front, Back, Left, Right): ") + dist.front + ", " + dist.back + ", " + dist.left + ", " + dist.right);
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
  Ensures: Moves the robot forwards by rotating the wheels forwards for for some distance in cm.
  float distance: Distance to move forwards in cm.
*/
void forward(float distance) {
  // Serial.println("forward function");
  // digitalWrite(redLED, HIGH);//turn on red LED
  // digitalWrite(grnLED, LOW);//turn off green LED
  // digitalWrite(ylwLED, LOW);//turn off yellow LED
  digitalWrite(ltDirPin, HIGH);  // Enables the motor to move in a particular direction
  digitalWrite(rtDirPin, HIGH);  // Enables the motor to move in a particular direction

  int steps = (int)ceil((distance * 800.0) / (8.5 * PI));  //this converts based on wheel diameter to steps from distance in cm
  //Steps both motors forward for the num steps calculated for the input distance
  for (int x = 0; x < steps; x++) {
    if (!running) {
      break;
    }
    digitalWrite(rtStepPin, HIGH);
    digitalWrite(ltStepPin, HIGH);
    delayMicroseconds(stepTime);
    digitalWrite(rtStepPin, LOW);
    digitalWrite(ltStepPin, LOW);
    delayMicroseconds(stepTime);
  }
}

/*
  Ensures: Moves the robot backwards by rotating the wheels backwards for some distance in cm. 
  floast distance: Distance in cm to move backwards. Input should be positive.
*/
void reverse(float distance) {

  Serial.println("forward function");
  digitalWrite(redLED, HIGH);   //turn on red LED
  digitalWrite(grnLED, LOW);    //turn off green LED
  digitalWrite(ylwLED, LOW);    //turn off yellow LED
  digitalWrite(ltDirPin, LOW);  // Enables the motor to move in a particular direction
  digitalWrite(rtDirPin, LOW);  // Enables the motor to move in a particular direction

  int steps = (int)ceil((distance * 800.0) / (8.5 * PI));  //this converts based on wheel diameter to steps from distance in cm
  //Steps both motors backwards for the num steps calculated for the input distance
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
  Ensures: Pivots robots by rotating the corresponding wheel 1012 pulses. For a pivot CCW, the right wheel is powered and for a pivot CW, the left wheel is powered.

  int direction: For a CCW, direction = 1, for a CW direction, direction = 0
*/
void pivot(int direction) {
  digitalWrite(ltDirPin, HIGH);  // Enables the motor to move in a particular direction
  digitalWrite(rtDirPin, HIGH);  // Enables the motor to move in a particular direction
  int stepCount = 1012;
  if (direction == 1) {
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
void turn(int direction, int amount) {
  // Serial.println("turn function");
  // digitalWrite(redLED, HIGH);//turn on red LED
  // digitalWrite(grnLED, LOW);//turn off green LED
  // digitalWrite(ylwLED, LOW);//turn off yellow LED

  digitalWrite(ltDirPin, HIGH);  // Enables the motor to move in a particular direction
  digitalWrite(rtDirPin, HIGH);  // Enables the motor to move in a particular direction

  if (direction == 1) {                 // if CW
    for (int x = 0; x < amount; x++) {  // 1600 pulses on outside wheel
      if (running == false) {
        break;
      }
      digitalWrite(rtStepPin, HIGH);
      digitalWrite(ltStepPin, HIGH);
      delayMicroseconds(stepTime);
      if (x % 3 == 0) {                // every third pulse
        digitalWrite(rtStepPin, LOW);  //makes the right wheel reset its pins 1/3rd rate of left
      }
      digitalWrite(ltStepPin, LOW);
      delayMicroseconds(stepTime);
    }
  } else {                              //if CCW
    for (int x = 0; x < amount; x++) {  // 1600 pulses on outside wheel
      if (running == false) {
        break;
      }
      digitalWrite(rtStepPin, HIGH);
      digitalWrite(ltStepPin, HIGH);
      delayMicroseconds(stepTime);
      if (x % 3 == 0) {                // every third pulse
        digitalWrite(ltStepPin, LOW);  //makes the left wheel reset its pins 1/3rd rate of right
      }
      digitalWrite(rtStepPin, LOW);
      delayMicroseconds(stepTime);
    }
  }
  delay(1000);  // One second delay
}
/*
   Ensures: Spins robots by rotating both wheels in opposite directions. For CW, right wheel is reverse and left wheel is forwards. For CCW, left wheel is reverse and right wheel is forwards.

   bool CW: If true, ropot spins CW. If false, spins CCW.

*/
void spin(bool CW) {

  Serial.println("spin function");
  digitalWrite(redLED, HIGH);    //turn on red LED
  digitalWrite(grnLED, LOW);     //turn off green LED
  digitalWrite(ylwLED, LOW);     //turn off yellow LED
  digitalWrite(ltDirPin, HIGH);  // Enables the motor to move in a particular direction
  digitalWrite(rtDirPin, HIGH);  // Enables the motor to move in a particular direction

  if (CW) {                       // if CW, reverses right wheel
    digitalWrite(rtDirPin, LOW);  // Enables the motor to move in opposite direction
  } else {                        // if CCW, reverses left wheel
    digitalWrite(ltDirPin, LOW);  // Enables the motor to move in opposite direction
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
    digitalWrite(rtDirPin, HIGH);  // Enables the motor to move in opposite direction
  } else {
    digitalWrite(ltDirPin, HIGH);  // Enables the motor to move in opposite direction
  }
  delay(1000);  // One second delay
}

/*
  Ensures: stops the motors from running by using the built in stop() method from the AccelStepper library.
*/
void stop() {
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
  running = true;
  while (thetac < thetag) {  //main stepping loop, stops when current angle is equal to or greater than target angle
    // prevents this from being blocking, uses global variable to break free if necessary
    if (running == false) {
      Serial.println("breaking");
      break;
    }
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
void goToGoalCm(float xg, float yg) {  // cm
  // Serial.println("Going to goal");
  digitalWrite(rtDirPin, HIGH);  //sets to drive forward
  digitalWrite(ltDirPin, HIGH);
  float xc = 0;  //variable for tracking current position
  float yc = 0;
  float dc = 0;
  float thetag = atan(yg / xg);                     //calculate desired angle
  if ((xg < 0 && yg < 0) || (xg < 0 && yg >= 0)) {  // if point lies in 2nd or 3rd quadrant must add 180 degrees to angle
    thetag = thetag + PI;
  } else if (xg > 0 && yg < 0) {  // Corrects angle into the correct 360 degree representation if in the 4th quadrant
    thetag += 2 * PI;
  } else if (xg == 0 && yg < 0) {  //special case for 90 degree right turn
    thetag = 3 * PI / 2;
  }
  goToAngle(thetag * (180 / PI));                                  //rotate robot to proper angle in degrees
  float dg = sqrt((xc - xg) * (xc - xg) + (yc - yg) * (yc - yg));  //calcule distance in cm for robot to travel

  digitalWrite(rtDirPin, HIGH);  //set both motors back to drive forward after rotation occured
  digitalWrite(ltDirPin, HIGH);

  delay(100);
  while (dg >= dc) {  //main loop advancing robot until it's current position equales or exceedes the target distance
    if (!running) {
      break;
    }
    digitalWrite(rtStepPin, HIGH);
    digitalWrite(ltStepPin, HIGH);
    delayMicroseconds(stepTime);
    digitalWrite(rtStepPin, LOW);
    digitalWrite(ltStepPin, LOW);
    delayMicroseconds(stepTime);
    dc += (8.5 * PI) / 800;   // every loop is a step of the motor, this line adds the proper amount of distance traveled in one step to the current position in cm
    delayMicroseconds(1000);  //artificially added delay to slow down speed further
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
void goToGoalIn(float xg, float yg) {  // in
  float xgcm = xg * 2.54;              //conversion to cm
  float ygcm = yg * 2.54;
  goToGoalCm(xgcm, ygcm);
}

/*
  Ensures: Moves the robot in an apparently random fashion when viewed. Non-Blocking. It runs between random angles, random spins, and random forward and backwards

*/
void randomWander() {

  // int movementStep = random(0, 501);  // sets random pulse for the forward and turn behaviors to use.
  // //sets lights to green only on
  // digitalWrite(grnLED, HIGH);
  // digitalWrite(redLED, LOW);
  // digitalWrite(ylwLED, LOW);
  // long randomBehavior = random(0, 4);  // chooses random behavior 0, 1, 2, 3
  // if (randomBehavior == 0) {
  //   forward(movementStep);
  // } else if (randomBehavior == 1) {
  //   int dir = random(0, 2);
  //   turn(dir, movementStep);
  // } else {
  //   int randX = random(0, 31);  // sets a random x distance in cm to 0 to 30 cm (approx 1 ft)
  //   int directionXModifier = random(0, 2);
  //   if (directionXModifier == 1) {
  //     randX *= -1;
  //   }
  //   int randY = random(0, 31);  // sets a random x distance in cm to 0 to 30 cm (approx 1 ft)
  //   int directionYModifier = random(0, 2);
  //   if (directionYModifier == 1) {
  //     randY *= -1;
  //   }
  //   goToGoalCm(randX, randY);
  // }

  int randomBehavior = random(0, 4);  // chooses random behavior 0, 1, 2, 3
  if (randomBehavior <= 1) {
    double randomAngel = 1.0 * random(0, 361);
    goToAngle(randomAngel);
  } else {
    forward(50);
  }
}

/*
  Ensures: Impliments a potential field protocol that uses the four lidar sensors to be repulsed from external objects

  Output: Robot motion based on external stimuli. 
*/
void runAway() {
  digitalWrite(redLED, 0);
  digitalWrite(ylwLED, 1);
  digitalWrite(grnLED, 0);
  // Serial.println(
  //   String("Sensor Values (Front, Back, Left, Right): ") + dist.front + ", " + dist.back + ", " + dist.left + ", " + dist.right);

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

    float x_result = -1.0 * x_vector();
    float y_result = -1.0 * y_vector();

    float theta = atan2(y_result, x_result) * (180 / PI);
    if (theta < 0) {
      theta = 360 + theta;  //this converts the angle back to 0 -> 360
    }
    Serial.println(theta);

    float magnitude = sqrt(x_result * x_result + y_result * y_result);
    if (magnitude < 3.0) {  // threshold
      return;               // forces too weak, don't move
    }
    goToAngle(theta);
    forward(10);
  }
}


/*
  Ensures: This method is the follow protocol for the robot
*/
void follow() {  //curious kid
  digitalWrite(redLED, 0);
  digitalWrite(ylwLED, 1);
  digitalWrite(grnLED, 0);
  Serial.println(
    String("Sensor Values (Front, Back, Left, Right): ") + dist.front + ", " + dist.back + ", " + dist.left + ", " + dist.right);

  // if (dist.front == 0 && dist.back == 0 && dist.left != 0 && dist.right != 0) {  //special hard coded case for when there are two walls on the left and right of robot only
  //   forward(10);
  //   return;
  // }

  // if (dist.front != 0 && dist.back != 0 && dist.left == 0 && dist.right == 0) {  //special case for when two wall front and back only
  //   goToAngle(90);
  //   forward(10);
  //   return;
  // }

  if (dist.front != 0 || dist.back != 0 || dist.left != 0 || dist.right != 0) {  //this if statement keeps it still if there is no reading

    float x_result = 1.0 * x_vector();
    float y_result = 1.0 * y_vector();

    float theta = atan2(y_result, x_result) * (180 / PI);
    if (theta < 0) {
      theta = 360 + theta;  //this converts the angle back to 0 -> 360
    }
    Serial.print("Following: ");
    Serial.println(theta);

    float magnitude = sqrt(x_result * x_result + y_result * y_result);
    if (magnitude > 15.0) {  // threshold
      return;               // forces too weak, don't move
    }
    goToAngle(theta);
    forward(10);
  }
}

/*
  Ensures: Implementation of the smart wander routine as per project guidelines. Method contains the main switching logic 

*/
void smartWander() {  //

  //logic for controlling in and out of the randomWander to runAway states
  if (currentState != 2 && (dist.left != 0 || dist.right != 0 || dist.front != 0 || dist.back != 0)) {
    currentState = 2;
  } else {
    currentState = 0;
  }

  //Switch case to call the correct method based on the current state variable value
  switch (currentState) {
    case (0):  //This is the random wander state
      running = true;
      randomWander();
      break;
    case (1):  //This is collide state
      break;
    case (2):  //This is run away
      running = true;
      runAway();
      break;
  }
}

/*
  Ensures: Implementation of the smart follow routine as per project guidelines. Method contains the main switching logic 

*/
void smartFollow() {  //

    //Switch case to call the correct method based on the current state variable value
  switch (currentState) {
    case (0):  //This is the random wander state
      running = true;
      randomWander();
      break;
    case (1):  //This is collide state
      break;
    case (3):  //This is run away
      running = true;
      follow();
      break;
  }
}

/*
  Ensures: Used in setup to begin the RPC communication and show that the seperate Cores are booted with visual cues.
*/
void blink(int led, int delaySeconds) {
  // for (int i = 0; i < 10; i++) {  // FIXED: Initialize i to 0
  //   digitalWrite(led, LOW);
  //   delay(delaySeconds);
  //   digitalWrite(led, HIGH);
  //   delay(delaySeconds);
  // }
  RPC.begin();
  digitalWrite(led, LOW);
}


void collide() {
  digitalWrite(redLED, 1);
  digitalWrite(ylwLED, 0);
  digitalWrite(grnLED, 0);
  Serial.println("Too Close");
  running = false;   //allows to break out of any loops
  currentState = 1;  //Changes the current state to 1 (angry kid)

  delay(500);  //just set a constant stall time, this + the sensor refresh rate is the time it takes to notice the object disapears
}

/*
  Ensures: Returns the X component of the vector that points the opposite direction of obstacles

  Returns: Float representing the force 
*/
float x_vector() {
  float kp = 100;
  float front_force = (dist.front > 0) ? (1.0 / dist.front) : 0.0;  //stops dividing by 0
  float back_force = (dist.back > 0) ? (1.0 / dist.back) : 0.0;
  return kp * (front_force - back_force);
}

/*
  Ensures: Returns the Y component of the vector that points the opposite direction of obstacles
*/
float y_vector() {
  float kp = 100;
  float left_force = (dist.left > 0) ? (1.0 / dist.left) : 0.0;
  float right_force = (dist.right > 0) ? (1.0 / dist.right) : 0.0;
  return kp * (left_force - right_force);
}
