

void setup(){

Serial.begin(115200);

}

void loop(){

float voltageLeft  = analogRead(A0) * (5.0/1023.0);
Serial.print("Left: ");
Serial.print(voltageLeft);
float voltageRight = analogRead(A1) * (5.0/1023.0);
Serial.print(" Right: ");
Serial.println(voltageRight);
delay(500);

}
