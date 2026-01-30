/************************************
   Unimate-Lab3-M7.ino
   Andrew Frush, Val Rumzis, 1.30.25
   ***********************************
   The following program is an implimentation of wall following behavior to avoid and go around obstacles to go to a goal. 
   The primary functions created are:

   followWall(): Utilizes proportional control and derivative control to follow a wall on the left and right (right is proportional only for easy comparison)
   followCenter(): Maintained equal distance between two walls flanking the robot to the left and right. Utilizes proportional control only.
   followCenterLogic(): State machine logic for switching between FollowCenter, FollowWall, and RandomWander
   goToGoalAvoidance(): Takes robot to relative goal position in the shortest possible distance (line). Deviates from path if obstacle in path.

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

#define centerDeadband 3  //Tolerance for followCenter, cm distance in both directions from the true center in between two walls.

int pauseTime = 2500;  //time before robot moves
int stepTime = 500;    //delay time between high and low on step pin
int wait_time = 1000;  //delay for printing data

bool running = true;  // global variable to keep whether the robot should be running or not, controls blocking

bool lastSeenWall = true;  // global variable for wall follow to track what wall was lost to ensure proper corner following. true = right & false = left

// Variables to keep track of robot global position
float xc = 0;
float yc = 0;
float tc = 0;

int preError;  //error info for derivative control on follow wall
int dE;

// Data to keep the lidar sensor data - taken from example code
struct sensors {
  int front;
  int back;
  int left;
  int right;
  MSGPACK_DEFINE_ARRAY(front, back, left, right);
} dist{};

void setup() {
  Serial.begin(115200);
  init_stepper();  //Initialize all motor control

  if (RPC.cpu_id() == CM7_CPUID) {  //Sets up RPC for multicore
    blink(ylwLED, 100);
  } else {
    blink(redLED, 100);
  }
  delay(500);
}

void loop() {
  // Read lidar data from M4
  dist = RPC.call("lidarRead").as<sensors>();  //get sensor data
  running = RPC.call("isRunning").as<bool>();  //check if robot should be running
  if (running) {
    digitalWrite(redLED, 0);
    digitalWrite(ylwLED, 1);
    digitalWrite(grnLED, 0);
  } else {
    digitalWrite(redLED, 1);
    digitalWrite(ylwLED, 0);
    digitalWrite(grnLED, 0);
  }
  // followWall();
  // followWallInnerCorner();
  goToGoalAvoidance(72 * 2.54, 0);
  //followCenterLogic();
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
  digitalWrite(ltDirPin, HIGH);  // Enables the motor to move in a particular direction
  digitalWrite(rtDirPin, HIGH);  // Enables the motor to move in a particular direction

  int steps = (int)ceil((distance * 800.0) / (8.5 * PI));  //this converts based on wheel diameter to steps from distance in cm
  //Steps both motors forward for the num steps calculated for the input distance
  for (int x = 0; x < steps; x++) {
    running = RPC.call("isRunning").as<bool>();
    if (!running) {
      Serial.println("collide forward");
      return;
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
  Ensures: Turns the robot though a difference in wheel speeds. Direction is left or right turn, Amount is number of steps for outer wheel.
*/
void turn(int direction, int amount) {
  digitalWrite(ltDirPin, HIGH);  // Enables the motor to move in a particular direction
  digitalWrite(rtDirPin, HIGH);  // Enables the motor to move in a particular direction

  if (direction == 1) {                 // if CW
    for (int x = 0; x < amount; x++) {  // 1600 pulses on outside wheel
      if (running == false) {
        return;
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
      if (!running) {
        return;
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
  float drift = 1.02;           //added to artificially correct for the overshoot/ compounding error during square
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
    running = RPC.call("isRunning").as<bool>();
    if (!running) {
      Serial.println("collide goToAngle");
      return;
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
    if (!running) { return; }
    digitalWrite(rtStepPin, HIGH);
    digitalWrite(ltStepPin, HIGH);
    delayMicroseconds(stepTime);
    digitalWrite(rtStepPin, LOW);
    digitalWrite(ltStepPin, LOW);
    delayMicroseconds(stepTime);
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
  Serial.println("Going to goal");
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
  Ensures: Tells robot follow a wall on the left or right. Uses a band to keep the robot at a certain distance to the wall. This code runs only when the mobile robot detects it is near a wall.

*/
void followWall() {
  if (!running) {
    Serial.println("collide followWall");
    return;
  }
  Serial.println(
    String("Sensor Values (Front, Back, Left, Right): ") + dist.front + ", " + dist.back + ", " + dist.left + ", " + dist.right);
  float speed = max_speed - 1100;
  float otherSpeed = max_speed - 1100;
  int upperMidBand = 21;
  int lowerMidBand = 15;
  int kp = 15;
  int kd = 5;
  int error = 0;
  // left wall follow
  if ((dist.left > 0) && (dist.right == 0)) {
    if (dist.left > upperMidBand) {  //for robot too far from wall
      error = dist.left - (upperMidBand + lowerMidBand) / 2;
      otherSpeed = speed - kp * error + kd * dE;  //slowing down wheel proportionally to error and propotional to change in error
      if (otherSpeed < 0) {
        otherSpeed = 0;
      }
      stepperRight.setSpeed(-1.0 * otherSpeed);
      stepperLeft.setSpeed(-1.0 * speed);
      Serial.println("Right Speed" + String(speed));
      Serial.println("Left Speed" + String(otherSpeed));
      preError = error;
    } else if ((dist.left < lowerMidBand) && (dist.left > 0)) {  // robot too close to wall
      error = (upperMidBand + lowerMidBand) / 2 - dist.left;
      otherSpeed = speed - kp * error - kd * dE;
      if (otherSpeed < 0) {
        otherSpeed = 0;
      }
      stepperRight.setSpeed(-1.0 * speed);
      stepperLeft.setSpeed(-1.0 * otherSpeed);
      Serial.println("Left Speed" + String(speed));
      Serial.println("Right Speed" + String(otherSpeed));
      preError = error;
    }
  } else if ((dist.right > 0) && (dist.left == 0)) {  //follow the right wall
    if (dist.right > upperMidBand) {                  //for robot too far from wall
      error = dist.right - (upperMidBand + lowerMidBand) / 2;
      otherSpeed = speed - kp * error;
      if (otherSpeed < 0) {
        otherSpeed = 0;
      }
      stepperLeft.setSpeed(-1.0 * otherSpeed);
      stepperRight.setSpeed(-1.0 * speed);
      Serial.println("Left Speed" + String(speed));
      Serial.println("Right Speed" + String(otherSpeed));
    } else if ((dist.right < lowerMidBand) && (dist.right > 0)) {  // robot too close to wall
      error = (upperMidBand + lowerMidBand) / 2 - dist.right;
      otherSpeed = speed - kp * error;
      if (otherSpeed < 0) {
        otherSpeed = 0;
      }
      stepperLeft.setSpeed(-1.0 * speed);
      stepperRight.setSpeed(-1.0 * otherSpeed);
      Serial.println("Right Speed" + String(speed));
      Serial.println("Left Speed" + String(otherSpeed));
    }
  }
  for (int i = 0; i < 5; i++) {
    if (!running) {
      Serial.println("collide followWall");
      return;
    }
    stepperLeft.runSpeed();
    stepperRight.runSpeed();
  }
}

/*
  Ensures: Tells robot follow a wall on the left or right. Uses a band to keep the robot at a certain distance to the wall. Robot then turns to follow an outide corner turn.

*/
void followWallOuterCorner() {
  if (!running) {
    Serial.println("collide followWallCorner");
    return;
  }
  Serial.println(
    String("Sensor Values (Front, Back, Left, Right): ") + dist.front + ", " + dist.back + ", " + dist.left + ", " + dist.right);
  float speed = max_speed - 1100;
  float otherSpeed = max_speed - 1100;
  int upperMidBand = 21;
  int lowerMidBand = 15;
  int kp = 15;
  int error = 0;
  // left wall follow
  if ((dist.left > 0) && (dist.right == 0)) {
    lastSeenWall = false;
    if (dist.left > upperMidBand) {  //for robot too far from wall
      error = dist.left - (upperMidBand + lowerMidBand) / 2;
      otherSpeed = speed - kp * error;
      if (otherSpeed < 0) {
        otherSpeed = 0;
      }
      stepperRight.setSpeed(-1.0 * otherSpeed);
      stepperLeft.setSpeed(-1.0 * speed);
      Serial.println("Right Speed" + String(speed));
      Serial.println("Left Speed" + String(otherSpeed));
    } else if ((dist.left < lowerMidBand) && (dist.left > 0)) {  // robot too close to wall
      error = (upperMidBand + lowerMidBand) / 2 - dist.left;
      otherSpeed = speed - kp * error;
      if (otherSpeed < 0) {
        otherSpeed = 0;
      }
      stepperRight.setSpeed(-1.0 * speed);
      stepperLeft.setSpeed(-1.0 * otherSpeed);
      Serial.println("Left Speed" + String(speed));
      Serial.println("Right Speed" + String(otherSpeed));
    }
  } else if ((dist.right > 0) && (dist.left == 0)) {
    lastSeenWall = true;
    if (dist.right > upperMidBand) {  //for robot too far from wall
      error = dist.right - (upperMidBand + lowerMidBand) / 2;
      otherSpeed = speed - kp * error;
      if (otherSpeed < 0) {
        otherSpeed = 0;
      }
      stepperLeft.setSpeed(-1.0 * otherSpeed);
      stepperRight.setSpeed(-1.0 * speed);
      Serial.println("Left Speed" + String(speed));
      Serial.println("Right Speed" + String(otherSpeed));
    } else if ((dist.right < lowerMidBand) && (dist.right > 0)) {  // robot too close to wall
      error = (upperMidBand + lowerMidBand) / 2 - dist.right;
      otherSpeed = speed - kp * error;
      if (otherSpeed < 0) {
        otherSpeed = 0;
      }
      stepperLeft.setSpeed(-1.0 * speed);
      stepperRight.setSpeed(-1.0 * otherSpeed);
      Serial.println("Right Speed" + String(speed));
      Serial.println("Left Speed" + String(otherSpeed));
    }
  } else if (lastSeenWall && (dist.right == 0)) {  // lost right wall
    reverse(20);
    goToAngle(270);
    reverse(15);
    return;
  } else if ((!lastSeenWall) && (dist.left == 0)) {  // lost left wall
    reverse(20);
    goToAngle(90);
    reverse(15);
    return;
  } else {  // if detects obstacle in both exits corner follow
    return;
  }
  for (int i = 0; i < 5; i++) {
    if (!running) {
      Serial.println("collide followWall");
      return;
    }
    stepperLeft.runSpeed();
    stepperRight.runSpeed();
  }
}

/*
  Ensures: Tells robot follow a wall on the left or right. Uses a band to keep the robot at a certain distance to the wall. Robot then turns to follow an outide corner turn.

*/
void followWallInnerCorner() {
  if (!running) {
    Serial.println("collide followWallCorner");
    return;
  }
  Serial.println(
    String("Sensor Values (Front, Back, Left, Right): ") + dist.front + ", " + dist.back + ", " + dist.left + ", " + dist.right);
  float speed = max_speed - 1100;
  float otherSpeed = max_speed - 1100;
  int upperMidBand = 21;
  int lowerMidBand = 15;
  int kp = 15;
  int error = 0;
  // left wall follow with no front obstacle
  if ((dist.left > 0) && (dist.right == 0) && (dist.front > 15 || dist.front == 0)) {
    if (dist.left > upperMidBand) {  //for robot too far from wall
      error = dist.left - (upperMidBand + lowerMidBand) / 2;
      otherSpeed = speed - kp * error;
      if (otherSpeed < 0) {
        otherSpeed = 0;
      }
      stepperRight.setSpeed(-1.0 * otherSpeed);
      stepperLeft.setSpeed(-1.0 * speed);
      Serial.println("Right Speed" + String(speed));
      Serial.println("Left Speed" + String(otherSpeed));
    } else if ((dist.left < lowerMidBand) && (dist.left > 0)) {  // robot too close to wall
      error = (upperMidBand + lowerMidBand) / 2 - dist.left;
      otherSpeed = speed - kp * error;
      if (otherSpeed < 0) {
        otherSpeed = 0;
      }
      stepperRight.setSpeed(-1.0 * speed);
      stepperLeft.setSpeed(-1.0 * otherSpeed);
      Serial.println("Left Speed" + String(speed));
      Serial.println("Right Speed" + String(otherSpeed));
    }
  } else if ((dist.right > 0) && (dist.left == 0) && (dist.front > 15 || dist.front == 0)) {  // right wall following no front wall
    if (dist.right > upperMidBand) {                                                          //for robot too far from wall
      error = dist.right - (upperMidBand + lowerMidBand) / 2;
      otherSpeed = speed - kp * error;
      if (otherSpeed < 0) {
        otherSpeed = 0;
      }
      stepperLeft.setSpeed(-1.0 * otherSpeed);
      stepperRight.setSpeed(-1.0 * speed);
      Serial.println("Left Speed" + String(speed));
      Serial.println("Right Speed" + String(otherSpeed));
    } else if ((dist.right < lowerMidBand) && (dist.right > 0)) {  // robot too close to wall
      error = (upperMidBand + lowerMidBand) / 2 - dist.right;
      otherSpeed = speed - kp * error;
      if (otherSpeed < 0) {
        otherSpeed = 0;
      }
      stepperLeft.setSpeed(-1.0 * speed);
      stepperRight.setSpeed(-1.0 * otherSpeed);
      Serial.println("Right Speed" + String(speed));
      Serial.println("Left Speed" + String(otherSpeed));
    }
  } else if ((dist.right > 0) && (dist.left == 0) && (dist.front <= 15) && (dist.front != 0)) {  // right wall follow + front obstacle detected
    goToAngle(90);
    return;
  } else if ((dist.left > 0) && (dist.right == 0) && (dist.front <= 15) && (dist.front != 0)) {  // left wall follow + front obstacle detected
    goToAngle(270);
    return;
  } else {  // if detects obstacle in both exits corner follow
    return;
  }
  for (int i = 0; i < 5; i++) {
    if (!running) {
      Serial.println("collide followWall");
      return;
    }
    stepperLeft.runSpeed();
    stepperRight.runSpeed();
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
  Ensures: Moves the robot in an apparently random fashion when viewed. Non-Blocking. It runs between random angles, and random forwards
*/
void randomWander() {
  if (!running) {
    Serial.println("collideWander");
    return;
  }
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

/*
  Ensures: Keep the robot centered while driving between two walls, uses proportional control
*/
void followCenter() {

  int error = dist.left - dist.right;  // If positive: farther to left, If Negavive farther to right

  int rightSpeed = 500;
  int leftSpeed = 500;
  int kp = 20;

  if (abs(error) > centerDeadband) {

    if (error > 0) {
      leftSpeed -= kp * error;
    } else {
      rightSpeed -= kp * -1 * error;  //this error would be nagative, don't want to make speed faster by subtracting negative so must multiply -1
    }
  }

  stepperRight.setSpeed(-1.0 * leftSpeed);
  stepperLeft.setSpeed(-1.0 * rightSpeed);

  for (int i = 0; i < 10; i++) {
    if (running) {
      stepperRight.runSpeed();
      stepperLeft.runSpeed();
    } else {
      return;
    }
  }
}

/*
  Ensures: This is the main state logic to switch between follow wall, follow center, and random wander
*/
void followCenterLogic() {

  if (dist.left != 0 && dist.right != 0) {
    followCenter();
  } else if (dist.left != 0 || dist.right != 0) {
    followWall();
  } else {
    randomWander();
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
  Ensures: Impliments a potential field protocol that uses the four lidar sensors to be repulsed from external objects. Uses lidar and calculated vector to turn an angle and move away.

  Output: Robot motion based on external stimuli. 
*/
void runAway() {
  digitalWrite(redLED, 0);
  digitalWrite(ylwLED, 1);
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
  Ensures: Takes robot to relative goal position in the shortest possible distance (line). Deviates from path if obstacle in path.A deviation is a scripted event
           where the robot turns, goes forward, then turns back twoards the goal and tries to advance. This is repeated untill the obstacle is reached.
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
void goToGoalAvoidance(float xg, float yg) {

  int detectDist = 15;

  // Calculate initial angle to goal and turn to face it
  float theta_initial = atan2(yg - yc, xg - xc) * (180 / PI);  //in degrees
  if (theta_initial < 0) {
    theta_initial = 360 + theta_initial;  // Convert to 0 -> 360
  }

  // Turn to face goal initially (goToAngle uses RELATIVE angles)
  float angleToTurn = theta_initial - tc;
  // Normalize to 0-360 range
  while (angleToTurn < 0) angleToTurn += 360;
  while (angleToTurn >= 360) angleToTurn -= 360;

  goToAngle(angleToTurn);
  tc = theta_initial;  // Update global orientation

  float error = sqrt((xg - xc) * (xg - xc) + (yg - yc) * (yg - yc));  // Calculate distance from goal (initial error)

  while (error > 4) {  //four is a static distance that we set as close enough to goal (linear distance of 4cm around goal location)
    if (!running) {
      continue;
    }

    dist = RPC.call("lidarRead").as<sensors>();  // Get sensor data

    if (dist.front >= detectDist || dist.front == 0) {  //move forward if nothing is in front or if
      // Path is clear, move forward one step
      digitalWrite(ltDirPin, LOW);  // Reverse direction
      digitalWrite(rtDirPin, LOW);

      // Take one step
      digitalWrite(rtStepPin, HIGH);
      digitalWrite(ltStepPin, HIGH);
      delayMicroseconds(stepTime);
      digitalWrite(rtStepPin, LOW);
      digitalWrite(ltStepPin, LOW);
      delayMicroseconds(stepTime);

      // Update global positions based on current angle for one step
      xc += ((8.5 * PI) / 800) * cos(tc * (PI / 180));
      yc += ((8.5 * PI) / 800) * sin(tc * (PI / 180));

      error = sqrt((xg - xc) * (xg - xc) + (yg - yc) * (yg - yc));  //recalculate error

    } else if ((dist.front < detectDist) && dist.front != 0) {
      // Obstacle detected - avoidance maneuver

      Serial.println(tc);

      // Turn RIGHT 90 degrees
      goToAngle(270);  // goToAngle will turn 270 which is same as CW 90 (right turn)
      tc -= 90;        // Turning right decreases angle
      if (tc < 0) {
        tc += 360;  // Keep in 0-360 range
      }
      // Move forward 30 cm (using reverse function since robot drives backwards), hard coded value to move
      reverse(30);

      // Update position based on direction we just moved
      xc += 30 * cos(tc * (PI / 180));
      yc += 30 * sin(tc * (PI / 180));

      // Calculate new angle to goal from current position
      float theta_to_goal = atan2(yg - yc, xg - xc) * (180 / PI);
      if (theta_to_goal < 0) {
        theta_to_goal = 360 + theta_to_goal;
      }

      // Calculate RELATIVE turn needed from current orientation
      float turnAngle = theta_to_goal - tc;
      // Normalize to 0-360 range for goToAngle
      while (turnAngle < 0) turnAngle += 360;
      while (turnAngle >= 360) turnAngle -= 360;

      // Turn to face goal
      goToAngle(turnAngle);
      tc = theta_to_goal;  // Update global orientation to absolute goal angle

      // Recalculate error
      error = sqrt((xg - xc) * (xg - xc) + (yg - yc) * (yg - yc));
    }
  }

  Serial.println("Goal reached!");
}
