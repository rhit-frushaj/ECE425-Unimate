/************************************
   Unimate-Lab3-M4.ino
   Andrew Frush, Val Rumzis, 
   ***********************************
   The following program is an implimentation of lidar and sensor reading to be used in conjunction with the corresponding M7 code.
   
   The primary functions created are:
   read_lidar: Given a lidar pin location, returns a distance in cm. 
   lidarRead: Getter method, callable from M7 to return lidar reading

************************************/

#include <Arduino.h>
#include <RPC.h>

//define sensor pin numbers
#define frontLdr 10
#define backLdr 11
#define leftLdr 12
#define rightLdr 13
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

  MSGPACK_DEFINE_ARRAY(front, back, left, right);
} dist{};

#define tooClose 3

bool running = true; // robot starts by running

void setup() {
  Serial.begin(115200);

  if (RPC.cpu_id() == CM7_CPUID) {  //Sets up RPC for multicore
    blink(ylwLED, 100);
  } else {
    blink(redLED, 100);
  }
  RPC.bind("lidarRead", lidarRead);  //Bind the lidarRead method to be callable from M7
  RPC.bind("isRunning", isRunning);  //Bind the isRunning method to be callable from M7
  init_sensors();                    //initialize all sensors
}

void loop() {
  // Constantly reading sensor values and updating the struct
  int tempRightLidar = 0;
  int tempLeftLidar = 0;
  int tempFrontLidar = 0;
  int tempBackLidar = 0;

  //Samples the sensors N times and averages their readings
  for (int i = 0; i < numSamples; i++) {
    tempRightLidar += read_lidar(rightLdr); // !! THESE HAVE BEEN FLIPPED ON PURPOSE SINCE THE ROBOT IS IN REVERSE !!
    tempLeftLidar += read_lidar(leftLdr);
    tempFrontLidar += read_lidar(frontLdr);
    tempBackLidar += read_lidar(backLdr);
  }

  dist.back = tempFrontLidar / numSamples;
  dist.front = tempBackLidar / numSamples;
  dist.right = tempLeftLidar / numSamples;
  dist.left = tempRightLidar / numSamples;

  if ((dist.front <= tooClose && dist.front != 0) || (dist.back <= tooClose && dist.back != 0) || (dist.left <= tooClose && dist.left != 0) || (dist.right <= tooClose && dist.right != 0)){
    //RPC.call("collide");
    running = false;
  } else {
    running = true;
  }
  delay(10);  // Small delay to prevent overwhelming the sensors
}

/*
  Ensures: This method is universal for M4 and M7, blinks LED to know which core is initializing and calls RPC.Begin
           Useful for troubleshooting purposes.
*/
void blink(int led, int delaySeconds) {
  pinMode(led, OUTPUT); //Commented OUT to speed up startup
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
}

/*
  Ensures: This is a getter method for the global running variable
*/
bool isRunning(){
  return running;
}
