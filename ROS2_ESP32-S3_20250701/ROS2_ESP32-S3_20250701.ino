// Button, Magnetic Sensor, Potentiometer Sensor, PVDF Pressure Sensor

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

// MQTT subscription topics
const char* cmd_topic = "robot/cmd";
const char* status_topic = "robot/status";
const char* pot_topic = "robot/pot";  // MQTT topic for potentiometer values

const int rgbPIN = 7;    //Define the pins of the RGB light
const int buttonPin = 14; // GPIO14 (Pin 14) for the button
const int force_sensorPin = 6; //magnetic Sensor Pin
const int analogInPin = 5;  // Analog input pin that the potentiometer is attached to
const int analogOutPin = 4;  // Analog output pin that the LED is attached to
const int pvdfPin = 2;      // PVDF pressure sensor analog input pin

int sensorValue = 0;  // value read from the pot
int outputValue = 0;  // value output to the PWM (analog out)
int lastMagSen = -1; // -1 ensures the first read always triggers
int minBrightness = 20; // Minimum brightness level (0-255)
int lastPotValue = -1; // Last potentiometer value for change detection

// PVDF pressure sensor variables
// int pvdfValue = 0;           // Raw analog reading from PVDF sensor
// int pvdfThreshold = 100;     // Threshold to detect handle holding (adjust based on your sensor)
// bool isHandleHeld = false;   // Current state of handle holding
// bool lastHandleState = false; // Previous state for change detection
// unsigned long lastPvdfRead = 0; // Last time PVDF was read
// const unsigned long pvdfReadInterval = 50; // Read PVDF every 50ms

// Variables for button handling
unsigned long lastPressTime = 0;       // Timestamp of the last button press
unsigned long pressStartTime = 0;      // Timestamp when the button was first pressed
bool buttonPressed = false;            // State variable for the button
bool longPressSent = false;            // Whether a long press action has been sent
bool isPaused = false;                 // Tracks whether the system is in the "paused" state
bool lightSen = true;
//bool magSen = 0;
bool lastLedState = true; // true for ON, false for OFF
int clickCount = 0;                    // Keeps track of clicks for double-click detection
bool batteryLow = false;               // Battery low status flag
bool nav2Available = false;            // Nav2 availability status flag
unsigned long lastMagneticCheck = 0;   // Rate limiting for magnetic sensor

// Message queue for MQTT publishing
char pendingMessage[50] = "";
bool hasPendingMessage = false;

// Timing thresholds (in milliseconds)
const unsigned long debounceDelay = 50;       // Debounce delay
const unsigned long longPressDuration = 3000; // Long press duration (3 seconds)
const unsigned long doubleClickTimeout = 300; // Maximum time between double clicks

Adafruit_NeoPixel strip = Adafruit_NeoPixel( MAX_LED, rgbPIN, NEO_RGB + NEO_KHZ800 );
//uint8_t i = 0;    
//uint8_t brightness = 0; 
//uint8_t fadeAmount = 1;

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

// MQTT message callback - Add battery status with error handling
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  // Add error checking for parameters
  if (!topic || !payload || length == 0) {
    return;
  }
  
  // Handle nav2 status and battery status
  if (strcmp(topic, status_topic) == 0) {
    if (length == 8 && memcmp(payload, "batt_red", 8) == 0) {
      batteryLow = true;
    } else if (length == 5 && memcmp(payload, "nav2_1", 5) == 0) {
      nav2Available = true;
    } else if (length == 5 && memcmp(payload, "nav2_0", 5) == 0) {
      nav2Available = false;
    }
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
      // Subscribe to the command and status topics
      if (client.subscribe(cmd_topic)) {
        debugPrint("Subscribed to robot/cmd");
      } else {
        debugPrint("Failed to subscribe to robot/cmd");
      }
      
      if (client.subscribe(status_topic)) {
        debugPrint("Subscribed to robot/status");
      } else {
        debugPrint("Failed to subscribe to robot/status");
      }
      
      // Add a small delay after connection to stabilize
      delay(100);
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
  // Check if client is valid and connected
  if (!client.connected()) {
    debugPrint("MQTT disconnected. Reconnecting...");
    connectToMQTT();
    return; // Don't publish if reconnection failed
  }
  
  // Add error handling for publish
  bool publishResult = false;
  try {
    publishResult = client.publish(topic, message);
  } catch (...) {
    debugPrint("Exception during MQTT publish");
    return;
  }
  
  if (publishResult) {
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
  // Only process button if Nav2 is available
  if (!nav2Available) {
    return;
  }
  
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
        Serial.println("Button pressed");
      } else {
        // Button is released
        unsigned long pressDuration = millis() - pressStartTime;
        
        if (pressDuration < longPressDuration && !longPressSent) {
          // Short press detected
          lastPressTime = millis(); // Record the release time
          Serial.println("Button released (short press)");
        } else {
          Serial.println("Button released (after long press)");
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
    Serial.println("Long press detected - Queueing 'stop'");
    strcpy(pendingMessage, "navStop");
    hasPendingMessage = true;
    longPressSent = true; // Prevent sending multiple long press messages
    clickCount = 0;       // Reset click count
  }
  
  // Process double-click after button releases
  if (!buttonPressed && clickCount == 2 && (millis() - lastPressTime <= doubleClickTimeout)) {
    if (isPaused) {
      Serial.println("Double click - Queueing 'resume'");
      strcpy(pendingMessage, "navCon");
      hasPendingMessage = true;
      isPaused = false;
    } else {
      Serial.println("Double click - Queueing 'pause'");
      strcpy(pendingMessage, "navPause");
      hasPendingMessage = true;
      isPaused = true;
    }
    
    clickCount = 0; // Reset click count after double click
  }
  
  // Reset click count if timeout occurs (for single clicks or incomplete double clicks)
  if (!buttonPressed && clickCount > 0 && (millis() - lastPressTime > doubleClickTimeout)) {
    if (clickCount == 1) {
      Serial.println("Single click timeout - No action taken");
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
  }
  
  brightness += fadeAmount;
  if (brightness == 0 || brightness == 200) fadeAmount = -fadeAmount;
}

void handlePotentiometer() {
  // Temporarily disable potentiometer to prevent crash
  return;
}

void handleMagneticSensor() {
  // Only process magnetic sensor if Nav2 is available
  if (!nav2Available) {
    return;
  }
  
  // Rate limiting - check every 1000ms (very conservative)
  unsigned long currentTime = millis();
  if (currentTime - lastMagneticCheck < 1000) {
    return;
  }
  lastMagneticCheck = currentTime;
  
  // Simple digital read with error checking
  int magSen = digitalRead(force_sensorPin);
  
  // Only process if we have a valid reading and it changed
  if (magSen != lastMagSen && lastMagSen != -1) {
    if (magSen == 0) {
      Serial.println("Magnetic: Attached - Queueing 'stop'");
      strcpy(pendingMessage, "navStop");
      hasPendingMessage = true;
    } else if (magSen == 1) {
      Serial.println("Magnetic: Detached - Queueing 'resume'");
      strcpy(pendingMessage, "navCon");
      hasPendingMessage = true;
    }
  }
  
  // Update state
  lastMagSen = magSen;
}

// void handlePVDFPressureSensor() {
//   unsigned long currentTime = millis();
//   if (currentTime - lastPvdfRead >= pvdfReadInterval) {
//     pvdfValue = analogRead(pvdfPin);
//     lastPvdfRead = currentTime;
    
//     // Debug output for calibration
//     Serial.print("PVDF Value: ");
//     Serial.println(pvdfValue);

//     // Check if handle is being held (pressure above threshold)
//     if (pvdfValue > pvdfThreshold) {
//       if (!isHandleHeld) {
//         isHandleHeld = true;
//         debugPrint("PVDF: Handle held detected");
//         Serial.println("PVDF: Handle held detected");
//         //publishMessage(cancel_topic, "navStop");
//       }
//     } else {
//       if (isHandleHeld) {
//         isHandleHeld = false;
//         debugPrint("PVDF: Handle released");
//         Serial.println("PVDF: Handle released");
//         //publishMessage(resume_topic, "navCon");
//       }
//     }
//   }
// }

//void handlePVDFPressureSensor() {
  // Temporarily disable PVDF to reduce memory usage
  // pvdfValue = analogRead(pvdfPin);
  // Serial.print("PVDF Value: ");
  // Serial.println(pvdfValue);
//}



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

  // Initialize PVDF pressure sensor pin
  // Note: PVDF sensor is analog input, no pinMode needed for analogRead
  // but we can set the analog read resolution for better precision
  analogReadResolution(12);
  pinMode(analogOutPin, OUTPUT);
  
  // Seed the random number generator
  randomSeed(analogRead(0));
  
  // Connect to Wi-Fi
  setupWiFi();
  
  // Setup MQTT client with smaller buffer to reduce memory usage
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(mqttCallback);
  client.setBufferSize(256); // Reduced buffer size to save memory
  
  debugPrint("Setup complete!");
}

void loop() {
  // Minimal loop to prevent reboot
  RGB_Breathing();
  
  // Ensure we're connected to the MQTT broker
  if (!client.connected()) {
    connectToMQTT();
  }
  
  // Process MQTT messages and maintain connection
  client.loop();
  
  // Sensor functions (queue messages instead of publishing)
  handleButton();
  // handlePotentiometer(); // Temporarily disabled
  handleMagneticSensor();
  
  // Publish any pending messages
  if (hasPendingMessage && client.connected()) {
    Serial.print("Publishing: ");
    Serial.println(pendingMessage);
    client.publish(cancel_topic, pendingMessage);
    hasPendingMessage = false;
    pendingMessage[0] = '\0'; // Clear the message
  }

  // Longer delay to prevent task conflicts
  delay(500);
}