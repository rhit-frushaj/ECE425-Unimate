
#include <Arduino.h>
#include <RPC.h>

//define sensor pin numbers
#define frontLdr 10
#define backLdr 11
#define leftLdr 12
#define rightLdr 13

//LED connections
#define ylwLED 7
#define redLED 5

#define tooClose 15

// Data structure for lidar
struct lidar {
  int front;
  int back;
  int left;
  int right;

  MSGPACK_DEFINE_ARRAY(front, back, left, right);
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
  dist.front = read_lidar(frontLdr);
  dist.back = read_lidar(backLdr);
  dist.left = read_lidar(leftLdr);
  dist.right = read_lidar(rightLdr);
  
  if (dist.front <= tooClose && dist.front != 0){ //this is smart but I don't think it can stay, this would be active for all states not just when it's colide
    RPC.call("collide");
  }

  delay(10); // Small delay to prevent overwhelming the sensors
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

lidar lidarRead(){
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

void init_sensors(){
  pinMode(frontLdr, INPUT);
  pinMode(backLdr, INPUT);
  pinMode(leftLdr, INPUT);
  pinMode(rightLdr, INPUT);
}



