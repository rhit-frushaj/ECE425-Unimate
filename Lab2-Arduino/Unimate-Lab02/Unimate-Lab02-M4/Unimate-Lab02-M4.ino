/************************************
   Unimate-Lab02-M4.ino
   Andrew Frush, Val Rumzis, 1.16.25
   ***********************************
   The following program is an implimentation of lidar and sensor reading to be used in conjunction with the corresponding M7 code.
   
   The primary functions created are:
   read_lidar: Given a lidar pin location, returns a distance in cm. 
   read_sonar: Given a sonar echo and trig pin location, returns a distance in cm.
   lidarRead: Getter method, callable from M7 to return lidar reading

************************************/

#include <Arduino.h>
#include <RPC.h>

//define sensor pin numbers
#define frontLdr 10
#define backLdr 11
#define leftLdr 12
#define rightLdr 13
#define leftSnrEcho 3
#define leftSnrTrig 2
#define rightSnrEcho 8
#define rightSnrTrig 9
#define numSamples 3

//LED connections
#define ylwLED 7
#define redLED 5

// Data structure for sensor readings
struct sensors {
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

  if (RPC.cpu_id() == CM7_CPUID) {  //Sets up RPC for multicore
    blink(ylwLED, 100);
  } else {
    blink(redLED, 100);
  }

  init_sensors();                    //initialize all sensors
  RPC.bind("lidarRead", lidarRead);  //Bind the lidarRead method to be callable from M7
}

void loop() {
  // Constantly reading sensor values and updating the struct
  int tempRightLidar = 0;
  int tempLeftLidar = 0;
  int tempFrontLidar = 0;
  int tempBackLidar = 0;
  int tempRightSonar = 0;
  int tempLeftSonar = 0;

  //Samples the sensors N times and averages their readings
  for (int i = 0; i < numSamples; i++) {
    tempRightLidar += read_lidar(rightLdr);
    tempLeftLidar += read_lidar(leftLdr);
    tempFrontLidar += read_lidar(frontLdr);
    tempBackLidar += read_lidar(backLdr);
    tempRightSonar += read_sonar(rightSnrTrig, rightSnrEcho);
    tempLeftSonar += read_sonar(leftSnrTrig, leftSnrEcho);
  }

  dist.front = tempFrontLidar / numSamples;
  dist.back = tempBackLidar / numSamples;
  dist.left = tempLeftLidar / numSamples;
  dist.right = tempRightLidar / numSamples;
  dist.sonarLeft = tempLeftSonar / numSamples;
  dist.sonarRight = tempRightSonar / numSamples;

  //IGNORE - This code left for example later

  // if ((dist.front <= tooClose && dist.front != 0) || (dist.back <= tooClose && dist.back != 0) || (dist.left <= tooClose && dist.left != 0) || (dist.right <= tooClose && dist.right != 0)){ //this is smart but I don't think it can stay, this would be active for all states not just when it's colide
  //   RPC.call("collide");
  // } else if( dist.front != 0 || dist.left != 0 || dist.back != 0 || dist.right != 0){
  //   RPC.call("runAway");
  // }

  delay(10);  // Small delay to prevent overwhelming the sensors
}

/*
  Ensures: This method is universal for M4 and M7, blinks LED to know which core is initializing and calls RPC.Begin
           Useful for troubleshooting purposes.
*/
void blink(int led, int delaySeconds) {
  // pinMode(led, OUTPUT); //Commented OUT to speed up startup
  // for (int i = 0; i < 10; i++) {
  //   digitalWrite(led, LOW);
  //   delay(delaySeconds);
  //   digitalWrite(led, HIGH);
  //   delay(delaySeconds);
  // }
  RPC.begin();
  digitalWrite(led, LOW);
}

/*
  Ensures: This is a getter method. Returns the sensor data when called. Callable from M7.
*/
sensors lidarRead() {
  return dist;
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

/*
  Ensures: Initializes all the pins for the sensors (lidar and sonar).
*/
void init_sensors() {
  pinMode(frontLdr, INPUT);
  pinMode(backLdr, INPUT);
  pinMode(leftLdr, INPUT);
  pinMode(rightLdr, INPUT);
  pinMode(rightSnrEcho, INPUT);
  pinMode(leftSnrEcho, INPUT);
  pinMode(leftSnrTrig, OUTPUT);
  pinMode(rightSnrTrig, OUTPUT);
}

/*
  Ensures: Reads sonar given the trig and echo pin
  Return: Int sonar distance reading [cm]
*/
int read_sonar(int trig, int echo) {
  uint16_t distance, pulseWidthUs;
  // pinMode(trig, OUTPUT);
  digitalWrite(trig, LOW);
  digitalWrite(trig, HIGH);  //Set the trig pin High
  delayMicroseconds(10);     //Delay of 10 microseconds
  digitalWrite(trig, LOW);   //Set the trig pin Low
  // pinMode(echo, INPUT);                //Set the pin to input mode
  pulseWidthUs = pulseIn(echo, HIGH);  //Detect the high level time on the echo pin, the output high level time represents the ultrasonic flight time (unit: us)
  distance = pulseWidthUs / 58;
  if (distance < 0 || distance > 30) { distance = 0; }
  return distance;
}
