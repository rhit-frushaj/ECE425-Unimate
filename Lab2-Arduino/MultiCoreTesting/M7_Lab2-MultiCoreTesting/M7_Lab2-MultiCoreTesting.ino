
#include<Arduino.h>
#include<RPC.h>


#define redLED 5            //red LED for displaying states
#define grnLED 6            //green LED for displaying states
#define ylwLED 7            //yellow LED for displaying states
uint8_t counter = 1;

void setup(){
  Serial.begin(115200);
  Serial.println("Hello?");


  int ledPins[3] = {redLED, grnLED, ylwLED}; //creates an array of LEDs   
  for(int pin : ledPins){
    pinMode(pin, 1); //sets all pins as outputs
  }

  if (RPC.cpu_id() == CM7_CPUID) {
    blink(ylwLED, 100); //blink blue LED (M7 core)
  } else {
    blink(5, 100); //blink green LED (M4 core)
  }  

}

void loop(){
  
  counter = RPC.call("getLED").as<uint8_t>();
  
  Serial.print("LED: ");
  Serial.println(counter);

  switch (counter){

    case 0b00000100:
    digitalWrite(redLED, 1);
    digitalWrite(grnLED, 0);
    digitalWrite(ylwLED, 0);
    break;
    case 0b00000010:
    digitalWrite(redLED, 0);
    digitalWrite(grnLED, 1);
    digitalWrite(ylwLED, 0);
    break;
    case 0b00000001:
    digitalWrite(redLED, 0);
    digitalWrite(grnLED, 0);
    digitalWrite(ylwLED, 1);
    break;
  }

}

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

// void setup() {
//   Serial.begin(9600);
//   RPC.begin();
// }

// void loop() {
//   String buffer = "";
//   while (RPC.available()) {
//     buffer += (char)RPC.read();  // Fill the buffer with characters
//   }
//   if (buffer.length() > 0) {
//     Serial.print(buffer);
//   }
// }
