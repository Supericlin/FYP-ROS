// Button, Magnetic Sensor, Potentiometer Sensor

#include <WiFi.h> //WiFi Library
#include <PubSubClient.h> // MQTT Library
#include <Adafruit_NeoPixel.h> //RGB Led Library

#define MAX_LED 14 
// WiFi credentials
const char* ssid = "superic-home";         // Wi-Fi SSID
const char* password = "super1c000000";    // Wi-Fi password

// MQTT broker details
const char* mqtt_server = "192.168.0.224";   // MQTT broker IP address
const int mqtt_port = 1883;                  // MQTT broker port (default: 1883)
const char* mqtt_username = "mouser";        // MQTT username
const char* mqtt_password = "m0user";        // MQTT password

// MQTT publish topics
const char* cancel_topic = "robot/control";  // MQTT topic for cancel messages
const char* pause_topic = "robot/control";   // MQTT topic for pause messages
const char* resume_topic = "robot/control";  // MQTT topic for resume messages

// MQTT subscription topic
//const char* cmd_topic = "robot/cmd";
const char* status_topic = "robot/status";

const int rgbPIN = 7;    //Define the pins of the RGB light
const int buttonPin = 14; // GPIO14 (Pin 14) for the button
const int force_sensorPin = 6; //magnetic Sensor Pin
const int analogInPin = 5;  // Analog input pin that the potentiometer is attached to
const int analogOutPin = 4;  // Analog output pin that the LED is attached to

int sensorValue = 0;  // value read from the pot
int outputValue = 0;  // value output to the PWM (analog out)
int lastMagSen = -1; // -1 ensures the first read always triggers
int minBrightness = 20; // Minimum brightness level (0-255)

// Variables for button handling
unsigned long lastPressTime = 0;       // Timestamp of the last button press
unsigned long pressStartTime = 0;      // Timestamp when the button was first pressed
bool buttonPressed = false;            // State variable for the button
bool longPressSent = false;            // Whether a long press action has been sent
bool isPaused = false;                 // Tracks whether the system is in the "paused" state
bool lightSen = true;
bool lastLedState = true;              // true for ON, false for OFF
bool batteryLow = false;               // Battery low status flag
bool nav2Available = false;            // Nav2 availability status flag
int clickCount = 0;                    // Keeps track of clicks for double-click detection

// Timing thresholds (in milliseconds)
const unsigned long debounceDelay = 50;       // Debounce delay
const unsigned long longPressDuration = 3000; // Long press duration (3 seconds)
const unsigned long doubleClickTimeout = 300; // Maximum time between double clicks

Adafruit_NeoPixel strip = Adafruit_NeoPixel( MAX_LED, rgbPIN, NEO_RGB + NEO_KHZ800 );

// WiFi and MQTT clients
WiFiClient espClient;
PubSubClient client(espClient);

// Function to print debug message with timestamp
void debugPrint(const char* message) {
  Serial.print(millis());
  Serial.print(": ");
  Serial.println(message);
  Serial.flush(); // Ensure data is sent immediately
}

// MQTT message callback
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String statusMsg = "";
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("]: ");
  for (unsigned int i = 0; i < length; i++) {
    statusMsg += (char)payload[i];
  }
  Serial.print(statusMsg);
  Serial.println();

  if (statusMsg == "LightSen_on") {
      //digitalWrite(analogOutPin, HIGH);
      lightSen = true;
      debugPrint("CMD: LightSen ON");
  } 
  if (statusMsg == "LightSen_off") {
      //digitalWrite(analogOutPin, LOW);
      lightSen = false;
      debugPrint("CMD: LightSen OFF");
  } 
  if (statusMsg == "batt_red") {
      batteryLow = true;
      debugPrint("Status: Battery Low");
  } 
  if (statusMsg == "nav2_1") {
      debugPrint("Status: NAV2 service available");
  } 
  if (statusMsg == "nav2_0") {
      debugPrint("Status: NAV2 service unavailable");
  }
}

// Function to connect to Wi-Fi
void setupWiFi() {
  delay(10);
  debugPrint("Starting WiFi connection...");
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print("."); // Progress indicator
    Serial.flush();
  }

  Serial.println();
  debugPrint("Wi-Fi connected!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.flush();
}

// Function to connect to the MQTT broker
void connectToMQTT() {
  int attempts = 0;
  
  while (!client.connected() && attempts < 5) {
    debugPrint("Connecting to MQTT broker...");
    
    // Create a unique client ID
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);
    
    if (client.connect(clientId.c_str(), mqtt_username, mqtt_password)) {
      debugPrint("MQTT connected!");
      // Subscribe to the command topic
      if (client.subscribe(status_topic)) {
        debugPrint("Subscribed to robot/status");
      } else {
        debugPrint("Failed to subscribe to robot/status");
      }
    } else {
      Serial.print("MQTT connection failed, rc=");
      Serial.print(client.state());
      Serial.println(" retrying in 2 seconds");
      Serial.flush();
      delay(2000); // Wait before retrying
      attempts++;
    }
  }
  
  if (!client.connected()) {
    debugPrint("Failed to connect to MQTT after multiple attempts. Will retry later.");
  }
}

// Function to publish a message to the given MQTT topic
void publishMessage(const char* topic, const char* message) {
  if (!client.connected()) {
    debugPrint("MQTT disconnected. Reconnecting...");
    connectToMQTT();
  }
  
  if (client.publish(topic, message)) {
    Serial.print("Published to ");
    Serial.print(topic);
    Serial.print(": ");
    Serial.println(message);
    Serial.flush();
  } else {
    debugPrint("Failed to publish message!");
  }
}

// Function to handle button logic
void handleButton() {
  // Read the button state with debouncing
  static unsigned long lastDebounceTime = 0;
  static int lastButtonState = HIGH;
  int buttonState = digitalRead(buttonPin);
  
  // Check if reading is different from last stable reading
  if (buttonState != lastButtonState) {
    lastDebounceTime = millis();
  }
  
  // Only process button if debounce period has passed
  if ((millis() - lastDebounceTime) > debounceDelay) {
    // Update the button state and check if it has changed
    bool isButtonPressed = (buttonState == LOW); // Active LOW button
    
    if (isButtonPressed != buttonPressed) {
      buttonPressed = isButtonPressed;
      
      if (buttonPressed) {
        // Button is pressed
        pressStartTime = millis(); // Record the press start time
        clickCount++;
        debugPrint("Button pressed");
      } else {
        // Button is released
        unsigned long pressDuration = millis() - pressStartTime;
        
        if (pressDuration < longPressDuration && !longPressSent) {
          // Short press detected
          lastPressTime = millis(); // Record the release time
          debugPrint("Button released (short press)");
        } else {
          debugPrint("Button released (after long press)");
        }
        
        // Reset long press flag when button is released
        longPressSent = false;
      }
    }
  }
  
  // Update last button state
  lastButtonState = buttonState;
  
  // Process long press
  if (buttonPressed && !longPressSent && (millis() - pressStartTime >= longPressDuration)) {
    publishMessage(cancel_topic, "navStop"); // Send "stop" message
    debugPrint("Long press detected - Sent 'stop'");
    Serial.println("Long press detected - Sent 'stop'");
    longPressSent = true; // Prevent sending multiple long press messages
    clickCount = 0;       // Reset click count
  }
  
  // Process double-click after button releases
  if (!buttonPressed && clickCount == 2 && (millis() - lastPressTime <= doubleClickTimeout)) {
    if (isPaused) {
      publishMessage(resume_topic, "navCon"); // Send "resume" message 
      debugPrint("Double click - Sent 'resume'");
      Serial.println("Double click - Sent 'resume'");
      isPaused = false;
    } else {
      publishMessage(pause_topic, "navPause"); // Send "pause" message
      debugPrint("Double click - Sent 'pause'");
      Serial.println("Double click - Sent 'pause'");
      isPaused = true;
    }
    
    clickCount = 0; // Reset click count after double click
  }
  
  // Reset click count if timeout occurs (for single clicks or incomplete double clicks)
  if (!buttonPressed && clickCount > 0 && (millis() - lastPressTime > doubleClickTimeout)) {
    if (clickCount == 1) {
      debugPrint("Single click timeout - No action taken");
    }
    clickCount = 0;
  }
}

void RGB_Light(int R, int G, int B) {
  R = map(R, 0, 255, 0, 100);
  G = map(G, 0, 255, 0, 100);
  B = map(B, 0, 255, 0, 100);
  uint32_t color = strip.Color(G, R, B);
  for (uint8_t i = 0; i < MAX_LED; i++) {
    strip.setPixelColor(i, color);
  }
  strip.show();
}

void RGB_Breathing() {
  static uint8_t brightness = 0;
  static int8_t fadeAmount = 1;
  if (batteryLow) {
    // Red breathing when battery is low
    RGB_Light(brightness, 0, 0); // Breathing red
  } else {
    // Normal teal breathing
    RGB_Light(0, brightness, brightness); // Breathing teal
  } // Breathing teal (change for other colors)
  brightness += fadeAmount;
  if (brightness == 0 || brightness == 200) fadeAmount = -fadeAmount;
}

void handlePotentiometer() {
  sensorValue = analogRead(analogInPin);

  if (lightSen == true) {
    outputValue = map(sensorValue, 0, 4095, minBrightness, 255);
    analogWrite(analogOutPin, outputValue);
    //Serial.println(sensorValue);

    if (!lastLedState) { // LED just turned on
      lastLedState = true;
    }
  } else {
    outputValue = 0;
    analogWrite(analogOutPin, outputValue);

    if (lastLedState) { // LED just turned off
      Serial.println("Led Off!");
      lastLedState = false;
    }
  }
}

void handleMagneticSensor() {
  int magSen = digitalRead(force_sensorPin); // Read current value
  // Only act if the value changed:
  if (magSen != lastMagSen) {
    if (magSen == 0) {
      Serial.println("MagSen is 0, handle attached!");
      publishMessage(cancel_topic, "navStop");
    } else {
      Serial.println("MagSen is 1, handle detached!");
      publishMessage(resume_topic, "navCon");
    }
    lastMagSen = magSen; // Update for next time
  }
}

void setup() {
  // Initialize Serial and wait a moment
  Serial.begin(115200);
  delay(500); // Give serial monitor time to start

  strip.begin();           
  strip.show(); 
  
  debugPrint("ESP32 MQTT Button Controller Starting...");

  // Initialize button pin
  pinMode(buttonPin, INPUT_PULLUP); // Use internal pull-up resistor

  // Initialize Magnetic pin
  pinMode(force_sensorPin, INPUT);

  analogReadResolution(12);
  pinMode(analogOutPin, OUTPUT);
  
  // Seed the random number generator
  randomSeed(analogRead(0));
  
  // Connect to Wi-Fi
  setupWiFi();
  
  // Setup MQTT client
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(mqttCallback);
  
  debugPrint("Setup complete!");
}

void loop() {
  RGB_Breathing();
  // Ensure we're connected to the MQTT broker
  if (!client.connected()) {
    connectToMQTT();
  }
  
  // Process MQTT messages and maintain connection
  client.loop();
  
  // Handle the button logic
  handleButton();
  handlePotentiometer();
  handleMagneticSensor();

  // Small delay to prevent excessive CPU usage and help with debouncing
  delay(10);
}