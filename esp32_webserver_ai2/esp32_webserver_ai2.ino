/* ----------------------------- UART Setup ----------------------------- */
// ESP32 transmits data to KL25Z via UART2 (Serial2.write)
#define RXD2 16
#define TXD2 17

// Assign output variables to GPIO pins
const int output26 = 26;

// Timing variables
unsigned long currentTime = millis();
unsigned long previousTime = 0; 
// Define timeout time in milliseconds (example: 2000ms = 2s)
const long timeoutTime = 2000;
int wait30 = 30000; // time to reconnect when connection is lost.

#define ESP32_LEDRED_ON 0b00000001U
#define ESP32_LEDRED_OFF 0b00000010U
#define ESP32_LEDGREEN_ON 0b00000011U
#define ESP32_LEDGREEN_OFF 0b00000100U 

#define ESP32_MOVE_STOP 0b00110000U
#define ESP32_MOVE_FORWARD 0b00110001U
#define ESP32_MOVE_BACK 0b00110010U
#define ESP32_MOVE_LEFT 0b00110011U
#define ESP32_MOVE_RIGHT 0b00110100U

#define ESP32_MODE_MANUAL 0b11110000U
#define ESP32_MODE_AUTO 0b11110001U

#define ESP32_MISC_RESERVED 0b11000000U
#define ESP32_MISC_CONNECTED 0b11000001U
#define ESP32_MISC_TESTING_ON 0b11000010U
#define ESP32_MISC_TESTING_OFF 0b11000011U

/* ----------------------------- Wifi Setup ----------------------------- */
#include <WiFi.h>
// Replace with your network credentials, change to 2.4Ghz if using Hotspot
const char* ssid = "BigTits";
const char* password = "rtpy4269";

// Set web server port number to 80
WiFiServer server(80);

// Variable to store the HTTP request
String response, ip_address;

// Auxiliary variables to store the current output state
String output26State = "off";

// check using ipconfig (Windows cmd)
IPAddress local_IP(192,168,151,156);   // ensure no clashing IP 
// Gateway IP address
IPAddress gateway(192,168,151,139);     // 1,1 at the end
IPAddress subnet(255, 255, 255, 0);
IPAddress primaryDNS(8, 8, 8, 8);
IPAddress secondaryDNS(8, 8, 4, 4); 

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  // Initialize the output variables as outputs
  pinMode(output26, OUTPUT);
  // Set outputs to LOW
  digitalWrite(output26, LOW);

  // Connect to Wi-Fi network with SSID and password
  Serial.print("Connecting to ");
  Serial.println(ssid);

  // Configure Static IP
  if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS))
  {
    Serial.println("Static IP failed to configure");
  }
  
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  // Print local IP address and start web server
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  ip_address = WiFi.localIP().toString();
  Serial.println(ip_address);
  server.begin();
}

void loop() {

// If disconnected, try to reconnect every 30 seconds.
  if ((WiFi.status() != WL_CONNECTED) && (millis() > wait30)) {
    Serial.println("Trying to reconnect WiFi...");
    WiFi.disconnect();
    WiFi.begin(ssid, password);
    wait30 = millis() + 30000;
  } 
  
  // Check if a client has connected..
  WiFiClient client = server.available();
  if (!client) {
    return;
  }
   
  Serial.print("New client: ");
  Serial.println(client.remoteIP());
  
  // Read the information sent by the client.
  String req = client.readStringUntil('\r');
  Serial.println(req);

  // Make the client's request.
  
  /* ------------------- Debugging Purposes  ------------------- */
  if(req.indexOf("status") != -1)
  {
    response = "WiFi Connected: " + ip_address;
  }
  
  if(req.indexOf("onRed") != -1)
  {
    digitalWrite(output26, HIGH);
    response = "TESTING RED LED, ON";
    Serial2.write(ESP32_MISC_TESTING_ON);
  }
  
  if(req.indexOf("offRed") != -1)
  {
    digitalWrite(output26, LOW);
    response = "TESTING RED LED, OFF";
    Serial2.write(ESP32_MISC_TESTING_OFF);
  }

  if (req.indexOf("connected") != -1)
  {
    response = "CONNECTED";
    Serial2.write(ESP32_MISC_CONNECTED);
  }

  /* ------------------- Movement  ------------------- */

  if(req.indexOf("moveStop") != -1)
  {
    response = "MOVE STOP";
    Serial2.write(ESP32_MOVE_STOP);
  }

  if(req.indexOf("moveForward") != -1)
  {
    response = "MOVE FORWARD";
    Serial2.write(ESP32_MOVE_FORWARD);
  }

  if(req.indexOf("moveLeft") != -1)
  {
    response = "MOVE LEFT";
    Serial2.write(ESP32_MOVE_LEFT);
  }

  if(req.indexOf("moveRight") != -1)
  {
    response = "MOVE RIGHT";
    Serial2.write(ESP32_MOVE_RIGHT);
  }

  if(req.indexOf("moveBack") != -1)
  {
    response = "MOVE BACK";
    Serial2.write(ESP32_MOVE_BACK);
  }

  if(req.indexOf("selfDriveStart") != -1)
  {
    response = "SELF DRIVE START";
    Serial2.write(ESP32_MODE_AUTO);
  }

  if(req.indexOf("selfDriveStop") != -1)
  {
    response = "SELF DRIVE STOP";
    Serial2.write(ESP32_MODE_MANUAL);
  }

  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/html");
  client.println(""); 
  client.println(response); //  Return status.

  client.flush();
  client.stop();
  Serial.println("Client disconnected.");
}
