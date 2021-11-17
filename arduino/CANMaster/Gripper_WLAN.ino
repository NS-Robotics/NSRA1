#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <Servo.h>

#define S1 D0
#define S2 D1
#define open 900

const char* ssid = "HomeNetworkRS";
const char* password = "Pegasus26061966";

ESP8266WebServer server(80);

Servo Servo1; 
Servo Servo2; 

void handleRoot() {
  server.send(200, "text/plain", "hello from esp8266!");
}

void handleNotFound(){
  String message = "File Not Found\n\n";
  message += "URI: ";
  message += server.uri();
  message += "\nMethod: ";
  message += (server.method() == HTTP_GET)?"GET":"POST";
  message += "\nArguments: ";
  message += server.args();
  message += "\n";
  for (uint8_t i=0; i<server.args(); i++){
    message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
  }
  server.send(404, "text/plain", message);
}


//Setup_________________________________________________________________________
void setup(void){
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);

  Servo1.attach(S1);
  Servo2.attach(S2);
  
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  Serial.println("");

  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  if (MDNS.begin("esp8266")) {
    Serial.println("MDNS responder started");
  }

  server.on("/", handleRoot);

  server.on("/open", []() {
    server.send(200, "text/plain", "Gripper opened!");
    digitalWrite(5, HIGH);
    for (int f = 0; f < open; f++)
    {
      digitalWrite(3, HIGH);
      delayMicroseconds(100);
      digitalWrite(3, LOW);
      delayMicroseconds(100);
    }
  });

  server.on("/close", []() {
    server.send(200, "text/plain", "Gripper closed!");
    digitalWrite(5, LOW);
    for (int f = 0; f < open; f++)
    {
      digitalWrite(3, HIGH);
      delayMicroseconds(100);
      digitalWrite(3, LOW);
      delayMicroseconds(100);
    }
  });

  server.onNotFound(handleNotFound);

  server.begin();
  Serial.println("HTTP server started");

  pinMode(5, OUTPUT);
  pinMode(3, OUTPUT);
}

//Loop_______________________________________________________________________________
void loop(void){
  server.handleClient();
}