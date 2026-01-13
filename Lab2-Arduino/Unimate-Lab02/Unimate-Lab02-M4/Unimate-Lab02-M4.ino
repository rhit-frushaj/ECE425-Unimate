// M4
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
#define numSamples 5

//LED connections
#define ylwLED 7
#define redLED 5

#define tooClose 5 // collide distance in [cm]

// Data structure for lidar
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

  if (RPC.cpu_id() == CM7_CPUID) {
    blink(ylwLED, 100);
  } else {
    blink(redLED, 100);
  }

  init_sensors();
  RPC.bind("lidarRead", lidarRead);
}

void loop() {
  // Constantly reading sensor values and updating the struct
  int tempRightLidar = 0;
  int tempLeftLidar = 0;
  int tempFrontLidar = 0;
  int tempBackLidar = 0;
  int tempRightSonar = 0;
  int tempLeftSonar = 0;

  for (int i = 0; i < 3; i++) {
    tempRightLidar += read_lidar(rightLdr);
    tempLeftLidar += read_lidar(leftLdr);
    tempFrontLidar += read_lidar(frontLdr);
    tempBackLidar += read_lidar(backLdr);
    tempRightSonar += read_sonar(rightSnrTrig, rightSnrEcho);
    tempLeftSonar += read_sonar(leftSnrTrig, leftSnrEcho);
  }


  dist.front = tempFrontLidar/3;
  dist.back = tempBackLidar/3;
  dist.left = tempLeftLidar/3;
  dist.right = tempRightLidar/3;
  dist.sonarLeft = tempLeftSonar/3;
  dist.sonarRight = tempRightSonar/3;

  // if ((dist.front <= tooClose && dist.front != 0) || (dist.back <= tooClose && dist.back != 0) || (dist.left <= tooClose && dist.left != 0) || (dist.right <= tooClose && dist.right != 0)){ //this is smart but I don't think it can stay, this would be active for all states not just when it's colide
  //   RPC.call("collide");
  // }

  delay(10);  // Small delay to prevent overwhelming the sensors
}

void blink(int led, int delaySeconds) {
  pinMode(led, OUTPUT);
  for (int i = 0; i < 10; i++) {
    digitalWrite(led, LOW);
    delay(delaySeconds);
    digitalWrite(led, HIGH);
    delay(delaySeconds);
  }
  RPC.begin();
  digitalWrite(led, LOW);
}

lidar lidarRead() {
  return dist;
}

int read_lidar(int pin) {
  int d;
  int16_t t = pulseIn(pin, HIGH);
  d = (t - 1000) * 3 / 40;
  if (t == 0 || t > 1850 || d < 0) {
    d = 0;
  }
  return d;
}

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

int read_sonar(int trig, int echo) {
  // float velocity((331.5 + 0.6 * (float)(20)) * 100 / 1000000.0);
  uint16_t distance, pulseWidthUs;
  // pinMode(trig, OUTPUT);
  digitalWrite(trig, LOW);
  digitalWrite(trig, HIGH);            //Set the trig pin High
  delayMicroseconds(10);              //Delay of 10 microseconds
  digitalWrite(trig, LOW);             //Set the trig pin Low
  // pinMode(echo, INPUT);                //Set the pin to input mode
  pulseWidthUs = pulseIn(echo, HIGH);  //Detect the high level time on the echo pin, the output high level time represents the ultrasonic flight time (unit: us)
  // distance = pulseWidthUs * velocity / 2.0;
  distance = pulseWidthUs / 58;
  if (distance < 0 || distance > 30) { distance = 0; }
  return distance;
}
