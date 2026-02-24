/************************************
   WiFi_MapSolving.ino
   Andrew Frush, Val Rumzis, 2/24/26
   ***********************************
   The following program is an implimentation of server client communication with the intent on recieving cardinal direction instructions to solve a grid structured maze. 
   The primary functions created are:

   connectToWifi(): Connects arduino to WiFi. Prints the ip address of the Arduino Giga to the terminal so it can be used to connect to from client. 
   requestOrders(): Ending command to send back to cliend to signal command has been processed.
   **processCommand: This method interperets the incoming commands and moved the robot accordingly.
 
   Other function were created as helper methods to make more complex behavior like inner and outer wall following.

   ** No an actual seperate method all of this is a part of the loop method

  ************************************/

#include <WiFi.h>
#include <AccelStepper.h>//include the stepper motor library
#include <MultiStepper.h>//include multiple stepper motor library

#define LEDPin 5

const char* password = "**REDACTED**";
const char* ssid = "Government Spy";

WiFiServer server(12345);   // TCP port
WiFiClient client;

int forwardConst = 45.5; //Distance of one of the box sides

const char* cmdFinish = "OrderUp"; //Key word that is sent to the client to signal order completion

int stepTime = 500;     //delay time between high and low on step pin

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

//function to set all stepper motor variables, outputs and LEDs
void init_stepper(){
  pinMode(rtStepPin, OUTPUT);//sets pin as output
  pinMode(rtDirPin, OUTPUT);//sets pin as output
  pinMode(ltStepPin, OUTPUT);//sets pin as output
  pinMode(ltDirPin, OUTPUT);//sets pin as output
  pinMode(stepperEnable, OUTPUT);//sets pin as output
  digitalWrite(stepperEnable, stepperEnFalse);//turns off the stepper motor driver

  stepperRight.setMaxSpeed(max_speed);//set the maximum permitted speed limited by processor and clock speed, no greater than 4000 steps/sec on Arduino
  stepperRight.setAcceleration(max_accel);//set desired acceleration in steps/s^2
  stepperLeft.setMaxSpeed(max_speed);//set the maximum permitted speed limited by processor and clock speed, no greater than 4000 steps/sec on Arduino
  stepperLeft.setAcceleration(max_accel);//set desired acceleration in steps/s^2
  steppers.addStepper(stepperLeft);//add left motor to MultiStepper
  steppers.addStepper(stepperRight);//add right motor to MultiStepper
  digitalWrite(stepperEnable, stepperEnTrue);//turns on the stepper motor driver
 
}

void setup() {
  Serial.begin(115200);
  pinMode(LEDPin, OUTPUT);
  
  connectToWiFi();
  init_stepper();

  //Connect to Computer and signal hook up
  server.begin();
  Serial.println("TCP server started");
  Serial.print("Arduino IP: ");
  Serial.println(WiFi.localIP());
  blink(10);
}

void loop() {
  // Check for new client
  if (!client || !client.connected()) {
    client = server.available();
    if (client) {
      Serial.println("Client connected");
    }
  }


  //Read Lidar
  client.println(read_lidar(10));


  // Catch and interpret data sent from client (computer)
  if (client && client.connected() && client.available()) {
    String cmd = client.readStringUntil('\n');
    cmd.trim();

    Serial.print("Received: ");
    Serial.println(cmd);

    // Simple command handling
    if (cmd == "PING") {
      client.println(cmdFinish);
    } 
    else if (cmd == "STATUS") {
      client.println("Arduino is alive");
      blink(5);
      requestOrders();
    } //Driving Commands for maze movement
    else if(cmd == "f" || cmd == "FORWARD"){ //forward command
      forward(forwardConst);
      requestOrders(); //signal completion after each movement back to client
    }
    else if( cmd == "r"){ //right turn and move command
      goToAngle(270);
      forward(forwardConst);
      requestOrders();
    }
    else if(cmd == "l"){ //left turn and drive command
      goToAngle(90);
      forward(forwardConst);
      requestOrders();
    }
    else if(cmd == "b"){ //rever/backwards turn and move command
      goToAngle(180);
      forward(forwardConst);
      requestOrders();
    }
    else if (cmd == "Stop"){ //stop command
      Serial.print("No More Orders");
    }
    else { //catch all
      client.println("Unknown command");
    }
  }
  delay(100);
}

/*
  Ensures: Sends message back to client that the last order has been completed. Seperate method in case wnat to add other indicators or actions upon completing order.
*/
void requestOrders(){
  client.println(cmdFinish);
}

/*
  Ensures: Simple connect to WiFi command, taken from prior project, standard method.
*/
void connectToWiFi() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
}

/*
  Ensures: Blinks LED for debugging and status purposes.
  int Count: number of times to blink LED
*/
void blink(int count){
  for(int i = 0; i <count; i++){
  digitalWrite(LEDPin, 0);
  delay(100);
  digitalWrite(LEDPin,1);
  delay(100);
  }
  digitalWrite(LEDPin, 0);
}

/*
  Ensures: Moves the robot forwards by rotating the wheels forwards for for some distance in cm.
  float distance: Distance to move forwards in cm.
*/
void forward(float distance) {
  Serial.println("forward function");

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
  Ensures: Given lidar method.
  Return: Int distance from lidar sensor in [cm]
*/
int read_lidar(int pin) {
  int d;
  int16_t t = pulseIn(pin, HIGH);
  d = (t - 1000) * 3 / 40;
  if (t == 0 || t > 1850 || d < 0) {
    d = 0;
  }
  return d;
}


