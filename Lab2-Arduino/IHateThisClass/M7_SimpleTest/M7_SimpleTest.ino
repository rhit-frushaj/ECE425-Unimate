
// #include<RPC.h>

// using namespace rtos;

// Thread testThread;

// void setup(){
//   Serial.begin(115200);
//   RPC.begin();
//   testThread.start(printAdd);
// }

// void loop(){


// }

// void printAdd(){
// while(true){
//   Serial.print("Number:");
//   // Serial.println(RPC.call("add", 5, 10));
//   int x = 5;
//   int y = 4;
//   int result = RPC.call("add", x, y).as<int>();
//   // int result = x+y;
//   Serial.println(result);
//   delay(100);
// }
// }

#include "Arduino.h"
#include "RPC.h"

void setup() {
  RPC.begin();
  Serial.begin(115200);

  //Bind the sensorRead() function on the M7
  RPC.bind("sensorRead", sensorRead);
}

void loop() {
  // On M7, let's print everything that is received over the RPC1 stream interface
  // Buffer it, otherwise all characters will be interleaved by other prints
  String buffer = "";
  while (RPC.available()) {
    buffer += (char)RPC.read();  // Fill the buffer with characters
  }
  if (buffer.length() > 0) {
    Serial.print(buffer);
  }
}

/*
Function on the M7 that returns an analog reading (A0)
*/
int sensorRead(int x, int y) {
  int result = x + y;
  return result;
}

