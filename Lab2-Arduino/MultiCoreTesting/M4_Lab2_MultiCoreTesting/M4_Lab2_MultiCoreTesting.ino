
// M4 Sketch - Server side, contained the counting logic

#include<RPC.h>
#include<Arduino.h>

uint8_t counter;

void setup(){
  counter = 0b00000100;
  RPC.bind("getLED", getLED);

  if (RPC.cpu_id() == CM7_CPUID) {
    blink(7, 100); //blink blue LED (M7 core)
  } else {
    blink(5, 100); //blink green LED (M4 core)
  } 

}

void loop(){
  //this loop is validated - ie I know this code produces the output 4, 2, 1
  counter >>= 1; //every second is advances to the next LED

  if (counter == 0b00000000){ //the led only goes from 5 to 7, so if you go past 7 you go back to 5
    counter = 0b00000100;
  }

  delay(1000);

}

uint8_t getLED(){

  return counter;

}

//Useful for setup, should see it blink if setup completes
void blink(int led, int delaySeconds) {
  for (int i; i < 10; i++) {
    digitalWrite(led, LOW);
    delay(delaySeconds);
    digitalWrite(led, HIGH);
    delay(delaySeconds);
  }
  RPC.begin();
}

// #include <RPC.h>
// uint8_t counter;

// void setup() {
//   RPC.begin();
//   counter = 0b00000100;
// }

// void loop() {
  
//   counter >>= 1; //every second is advances to the next LED

//   if (counter == 0b00000000){ //the led only goes from 5 to 7, so if you go past 7 you go back to 5
//     counter = 0b00000100;
//   }

//   RPC.println(counter);

//   delay(1000);
// }