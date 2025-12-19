/************************************
   Lab1-ForReadCode.ino
   Andrew Frush, Val Rumzis, 12.11.25
   ***********************************
   This program will introduce using the stepper motor library to create motion algorithms for the robot.
   The motions will be forward, reverse, pivot, spin, turn, stop, go to angle, go to goal, move in a circle, square, and figure eight.
   
   The primary functions created are:
   forward   - Moves the robot forwards by rotating the wheels forwards for some distance in cm.
   reverse   - Moves the robot backwards by rotating the wheels backwards for some distance in cm. Input should be positive.
   pivot     - Pivots robots by rotating the corresponding wheel 1012 pulses. For a pivot CCW, the right wheel is powered and for a pivot CW, the left wheel is powered.
   spin      - Spins robots by rotating both wheels in opposite directions. For CW, right wheel is reverse and left wheel is forwards. For CCW, left wheel is reverse and right wheel is forwards.
   turn      - Turns robots by rotating the outside wheel 1600 pulses. The inside wheel moves every third pulse, therefore moving inner wheel 1/3rd the speed.
   stop      - Stops the motors from running by using the built in stop() method from the AccelStepper library
   goToAngle - Moves the robot to face in an angle relative to it's starting position. Robot will turn CCW or CW based on which requires the shortest time. See function for visual example.
   goToGoalCm- Moves the robot to a position based on it's current position in centimeters. First rotates the robot then moves in a straight line directly to the goal. See function for visual example.
   squareCm  - Moves the robot in a square of a programable side length. Robot starts in the bottom of the square. See function for visual example.
   circleCm  - Moves the robot in a CW or CCW circle of some diameter in centimeters. Diameter is based on the centerline of the robot (between two wheeles).
   figure8   - Moves the robot in a figure eight starting at the center of the figure eight.
   
   NOTE: Most functions in cm have an inches counterpart. For example, goToGoalCm for cm or goToGoalIn for in.
*/

//includew all necessary libraries
#include <Arduino.h>
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
#define max_speed 1636 //maximum stepper motor speed
#define max_accel 10000 //maximum motor acceleration

int pauseTime = 2500;   //time before robot moves
int stepTime = 500;     //delay time between high and low on step pin
int wait_time = 1000;   //delay for printing data

uint8_t currentState = 0; //Tracks the state robot is currently in
// State Table 
// 0 = randomWander
// 1 = angryKid
// 2 = shyKid
// 3 = curiousKid
// 4 = ...
bool running = true; // global variable to keep whether the robot should be running or not, controls blocking

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
  init_stepper();
  Serial.begin(115200);
  delay(1000);
  forward(10);
}

void loop() {
  // put your main code here, to run repeatedly:
  switch(currentState){
    case(0): //This is the random wander state
      running = true;
      randomWander();
      break;
    case(1):

      break;
    case(2):
      break;
    case(3):
      break;

  }
  // delay(5000);
}
/*
  Ensures: Moves the robot forwards by rotating the wheels forwards for for some distance in cm.
  float distance: Distance to move forwards in cm.
*/
void forward(float distance) {
  Serial.println("forward function");
  digitalWrite(redLED, HIGH);//turn on red LED
  digitalWrite(grnLED, LOW);//turn off green LED
  digitalWrite(ylwLED, LOW);//turn off yellow LED
  digitalWrite(ltDirPin, HIGH); // Enables the motor to move in a particular direction
  digitalWrite(rtDirPin, HIGH); // Enables the motor to move in a particular direction
   
  int steps = (int)ceil((distance*800.0)/(8.5*PI)); //this converts based on wheel diameter to steps from distance in cm
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
  Ensures: Moves the robot backwards by rotating the wheels backwards for some distance in cm. 
  floast distance: Distance in cm to move backwards. Input should be positive.
*/
void reverse(float distance){

  Serial.println("forward function");
  digitalWrite(redLED, HIGH);//turn on red LED
  digitalWrite(grnLED, LOW);//turn off green LED
  digitalWrite(ylwLED, LOW);//turn off yellow LED
  digitalWrite(ltDirPin, LOW); // Enables the motor to move in a particular direction
  digitalWrite(rtDirPin, LOW); // Enables the motor to move in a particular direction
  
  int steps = (int)ceil((distance*800.0)/(8.5*PI)); //this converts based on wheel diameter to steps from distance in cm
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
void turn(int direction, int amount){
  Serial.println("turn function");
  digitalWrite(redLED, HIGH);//turn on red LED
  digitalWrite(grnLED, LOW);//turn off green LED
  digitalWrite(ylwLED, LOW);//turn off yellow LED

  digitalWrite(ltDirPin, HIGH); // Enables the motor to move in a particular direction
  digitalWrite(rtDirPin, HIGH); // Enables the motor to move in a particular direction
 
  if (direction == 1){ // if CW
    for (int x = 0; x < amount; x++) { // 1600 pulses on outside wheel
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
    for (int x = 0; x < amount; x++) { // 1600 pulses on outside wheel
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
  Serial.println("Made it to angle");
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
  running = true;
  while (thetac < thetag){ //main stepping loop, stops when current angle is equal to or greater than target angle
      // prevents this from being blocking, uses global variable to break free if necessary
      if(running == false){
        Serial.println("breaking");
        break;
      }
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
  if ((xg < 0 && yg < 0) || (xg < 0 && yg >= 0)){ // if point lies in 2nd or 3rd quadrant must add 180 degrees to angle
    thetag = thetag+ PI;
  } else if (xg >0 && yg < 0 ){ // Corrects angle into the correct 360 degree representation if in the 4th quadrant
    thetag += 2*PI;
  } else if (xg == 0 && yg < 0){ //special case for 90 degree right turn
    thetag = 3*PI/2;
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
  Ensures: Moves the robot in an apparently random fashion when viewed. Non-Blocking. It runs between random angles, random spins, and random forward and backwards

*/
void randomWander(){
//Andrew Note: I think we should generate a random number between a min and max speed then run the motors at those in a random direction. 
// To spice things if the nunber is divisible by 5 or something like that then we could have it perform moveForward(random distance) and divisible by 3 could be random go to angle? 
    
  //sets lights to green only on
  digitalWrite(grnLED, HIGH);
  digitalWrite(redLED, LOW);
  digitalWrite(ylwLED, LOW);

  long randomDirection = random(0,2); //gives a random number either 0 or 1 (excluses 2)
  digitalWrite(rtDirPin, randomDirection); //set both motors to be same random direction
  digitalWrite(ltDirPin, randomDirection);
  Serial.print("Random Direction: ");
  Serial.println(randomDirection);

  long randomAction = random(0,6); //produces an output of 0, 1, or 2
  Serial.println(randomAction);
  long maxDist = 1201; //max steps it can do
  // if(randomAction == 0){ //if random action is 0
  //   stepperRight.move(random(0,maxDist)); //sets the steps for the left and right motors to randomly go some distance
  //   stepperLeft.move(random(0,maxDist));

  //   //set random speed
  //   stepperRight.setMaxSpeed(1.0*random(100, max_speed); //multiply by 1 to get it back to float
  //   stepperLeft.setMaxSpeed(1.0*random(100, max_speed); //multiply by 1 to get it back to float
    
  // } else if(randomAction == 1){
  //   float theta = 1.0*random(0,361); //random angle between 0-360, excludes 361
  //   goToAngle(theta);
  // }else if(randomAction == 2){
  //   digitalWrite(ltStepPin, !randomDirection); //if gets this we flip the random direction of 
  // }
  float theta;
  long lcount; 
  long rcount;
  long lrandomSpeed; //currently no functionality to make it have different speeds on the wheels
  long rrandomSpeed;
  switch(randomAction){
    case(0): //Random action where the robot turns to some random angle
      theta = random(0, 36001) / 100.0;  // random float between 0.00 → 360.00
      Serial.println(theta);
      goToAngle(theta);
      break;
    
    case(5):
      turn(randomDirection, random(0, maxDist/2));
 
    case(1): //random action where the robot spins some random amount
      digitalWrite(ltDirPin, !randomDirection); //if gets this we flip the random direction of

    case(3):

    case(4):

    case(2): //random action where the robot drives forward or backwards by some random amount
      lcount = random(0, maxDist);
      rcount =random(0, maxDist);
      rrandomSpeed = random(.5, 3);
      lrandomSpeed = rrandomSpeed;

  }

  // if (stepperRight.distanceToGo() == 0) {
  //   // Random change to speed, position and acceleration
  //   // Make sure we dont get 0 speed or accelerations
  //   delay(1000);
  //   stepperRight.moveTo(rand() % 200);
  //   stepperRight.setMaxSpeed((rand() % 200) + 1);
  //   stepperRight.setAcceleration((rand() % 200) + 1);
  // }

  // running = true;
  // while(running){
  //   running = stepperRight.run();
  //   stepperLeft.run();

  //   //CHECK SENSORS HERE ASSUMING WE DON'T DO INTERUPTS (I think interupts would be best for the record, maybe??)
  // }

  int fasterMotor = max(lrandomSpeed, rrandomSpeed);


  while((lcount > 0 || rcount > 0) && running){
    
    digitalWrite(rtStepPin, HIGH);
    digitalWrite(ltStepPin, HIGH);
    // delayMicroseconds(rrandomSpeed);
    delay(rrandomSpeed);
    digitalWrite(rtStepPin, LOW);
    digitalWrite(ltStepPin, LOW);
    
    lcount--;
    rcount--;
  }
}







