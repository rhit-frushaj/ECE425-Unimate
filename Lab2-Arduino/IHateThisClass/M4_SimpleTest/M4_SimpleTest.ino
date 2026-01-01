
// #include<RPC.h>

// void setup(){

//   RPC.begin();
//   RPC.bind("add", add);
// }

// void loop(){

// }

// int add(int x, int y){
//   return x + y;
// }

#include "Arduino.h"
#include "RPC.h"

using namespace rtos;

Thread sensorThread;

void setup() {
  RPC.begin();
  // Serial.begin(115200);

  /*
  Starts a new thread that loops the requestReading() function
  */
  sensorThread.start(requestReading);
}

void loop() {

}

/*
This thread calls the sensorThread() function remotely
every second. Result is printed to the RPC1 stream.
*/
void requestReading() {
  while (true) {
    delay(1000);
    auto result = RPC.call("sensorRead", 5, 4).as<int>();
    RPC.println("Result is " + String(result));
  }
}