#include <Arduino.h>
#include <WiFi.h>
#include <NetworkClient.h>
#include <WiFiAP.h>
#define RXD2 13 //Hardware Serial Pins from esp32 to teensy
#define TXD2 12 //Hardware Serial Pins from esp32 to teensy

// Set wifi name and password
const char *ssid = "Hampter";
const char *password = "Whirligig";

// Begin server listening on TCP port 80
NetworkServer server(80);


void setup() {

  // Begin ESP32 USB serial connection
  Serial.begin(115200);

  // Begin ESP32s hardware serial port
  // baud rate, 8 data bits, no parity, 1 stop bit, GPIO 13 receives data, GPIO 12 transmits data
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2); 

  Serial.println();
  Serial.println("Configuring access point...");

  // Create wifi access point
  // You can remove the password parameter if you want the AP to be open.
  // a valid password must have more than 7 characters
  if (!WiFi.softAP(ssid, password)) {
    Serial.println("Wi-Fi failed");
    while (true) {}
  }

  // Print IP address to use to connect to ESP32 server
  IPAddress myIP = WiFi.softAPIP();
  Serial.print(" ESP32 IP address: ");
  Serial.println(WiFi.softAPIP());

  // Start the server
  server.begin();
  Serial.println("TCP server started");

}

void loop() {
  // Listening for incoming clients
  NetworkClient client = server.accept();  
  
  // Once client is heard
  if (client) {                     // if you get a client,
    Serial.println("Simulink connected!");  // print a message out the serial port
    String currentLine = "";        // make a String to hold incoming data from the client
    while (client.connected()) {    // loop while the client's connected
      if (client.available()) {     // if there's bytes to read from the client,

        // Read cmd from simulink
        char c = client.read();     // read a byte, then

        // Print out command to teensy over hardware serial 
        Serial2.println(c);

        // Cmd from simulink print out to esp32 serial monitor
        Serial.write(c);

        delay(1);

      }
    // close the connection:
    client.stop();
    Serial.println("\nSimulink disconnected");
  }
 }
}
