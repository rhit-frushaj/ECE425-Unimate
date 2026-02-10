
// Testing File for recieving and sending Serial Communication from the Arduino to Computer over WiFi 

#include <WiFi.h>

#define LEDPin 5

// const char* ssid = "SpectrumSetup-37";
// const char* password = "zealhotel947";
const char* password = "Sc00byD00$4";
const char* ssid = "Government Spy";

WiFiServer server(12345);   // TCP port
WiFiClient client;

void setup() {
  Serial.begin(115200);
  pinMode(LEDPin, OUTPUT);
  connectToWiFi();

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

  // If client sent data
  if (client && client.connected() && client.available()) {
    String cmd = client.readStringUntil('\n');
    cmd.trim();

    Serial.print("Received: ");
    Serial.println(cmd);

    // Simple command handling
    if (cmd == "PING") {
      client.println("PONG");
    } 
    else if (cmd == "STATUS") {
      client.println("Arduino is alive");
      blink(5);
    }
    else {
      client.println("Unknown command");
    }
  }
}

void connectToWiFi() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
}

void blink(int count){
  for(int i = 0; i <count; i++){
  digitalWrite(LEDPin, 0);
  delay(100);
  digitalWrite(LEDPin,1);
  delay(100);
  }
  digitalWrite(LEDPin, 0);
}




