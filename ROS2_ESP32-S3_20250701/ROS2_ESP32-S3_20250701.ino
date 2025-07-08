// Button, Magnetic Sensor, Potentiometer Sensor, PVDF Pressure Sensor

#include <WiFi.h>
#include <WiFiManager.h>
#include <PubSubClient.h>
#include <Adafruit_NeoPixel.h>

#define MAX_LED 14
#define MQTT_SERVER "192.168.0.224"
#define MQTT_PORT 1883
#define MQTT_USERNAME "mouser"
#define MQTT_PASSWORD "m0user"
#define MQTT_TOPIC "robot/control"
#define STATUS_TOPIC "robot/status"

// Pin definitions
#define RGB_PIN 7
#define BUTTON_PIN 14
#define MAG_SENSOR_PIN 6
#define POT_PIN 5
#define LED_PIN 4
#define PVDF_PIN 2

// Timing constants
#define DEBOUNCE_DELAY 50
#define LONG_PRESS_DURATION 3000
#define DOUBLE_CLICK_TIMEOUT 300
#define BOOT_DELAY 5000

// Global variables
char ssid[32] = "";
char password[64] = "";

int sensorValue = 0, outputValue = 0, lastMagSen = -1, pvdfValue = 0;
int minBrightness = 20, clickCount = 0, navColorMode = 0;

unsigned long lastPressTime = 0, pressStartTime = 0, bootTime = 0;

bool buttonPressed = false, longPressSent = false, isPaused = false;
bool lightSen = true, lastLedState = true, batteryLow = false;
bool nav2Available = false, pressureState = false, firstBoot = true;

Adafruit_NeoPixel strip = Adafruit_NeoPixel(MAX_LED, RGB_PIN, NEO_RGB + NEO_KHZ800);
WiFiClient espClient;
PubSubClient client(espClient);
WiFiManager wifiManager;

// MQTT message callback
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String statusMsg = "";
  for (unsigned int i = 0; i < length; i++) {
    statusMsg += (char)payload[i];
  }
  
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("]: ");
  Serial.println(statusMsg);

  // Handle different status messages
  if (statusMsg == "LightSen_on") lightSen = true;
  else if (statusMsg == "LightSen_off") lightSen = false;
  else if (statusMsg == "batt_red") batteryLow = true;
  else if (statusMsg == "nav2_1") nav2Available = true;
  else if (statusMsg == "nav2_0") nav2Available = false;
  else if (statusMsg == "navStatus_1" || statusMsg == "navStatus_4") navColorMode = 1;
  else if (statusMsg == "navStatus_5") navColorMode = 0;
  else if (statusMsg == "navStatus_6") navColorMode = 2;
}

// Function to connect to Wi-Fi
void setupWiFi() {
  delay(10);
  Serial.println("Starting WiFiManager...");
  
  wifiManager.setConfigPortalTimeout(180);
  
  if (!wifiManager.autoConnect("ESP32_Config")) {
    Serial.println("Failed to connect and hit timeout");
    delay(3000);
    ESP.restart();
  }
  
  Serial.println("Wi-Fi connected!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

// Function to connect to the MQTT broker
void connectToMQTT() {
  int attempts = 0;
  
  while (!client.connected() && attempts < 3) {
    Serial.println("Connecting to MQTT broker...");
    
    char clientId[32];
    sprintf(clientId, "ESP32Client-%d", random(1000, 9999));
    
    if (client.connect(clientId, MQTT_USERNAME, MQTT_PASSWORD)) {
      Serial.println("MQTT connected!");
      client.subscribe(STATUS_TOPIC) ? 
        Serial.println("Subscribed to robot/status") : 
        Serial.println("Failed to subscribe to robot/status");
      break;
    } else {
      Serial.print("MQTT connection failed, rc=");
      Serial.println(client.state());
      delay(2000);
      attempts++;
    }
  }
  
  if (!client.connected()) {
    Serial.println("Failed to connect to MQTT after multiple attempts. Will retry later.");
  }
}

// Function to publish a message to the given MQTT topic
void publishMessage(const char* topic, const char* message) {
  // Don't publish during first 5 seconds after boot
  if (firstBoot && (millis() - bootTime < BOOT_DELAY)) {
    Serial.println("Skipping publish during first boot");
    return;
  }
  
  // After 5 seconds, allow publishing
  if (firstBoot && (millis() - bootTime >= BOOT_DELAY)) {
    firstBoot = false;
    Serial.println("First boot period ended, publishing enabled");
  }
  
  if (!client.connected()) {
    Serial.println("MQTT disconnected. Reconnecting...");
    connectToMQTT();
  }
  
  if (client.publish(topic, message)) {
    Serial.print("Published to ");
    Serial.print(topic);
    Serial.print(": ");
    Serial.println(message);
  } else {
    Serial.println("Failed to publish message!");
  }
}

// Function to handle button logic
void handleButton() {
  static unsigned long lastDebounceTime = 0;
  static int lastButtonState = HIGH;
  int buttonState = digitalRead(BUTTON_PIN);
  
  // Debouncing
  if (buttonState != lastButtonState) {
    lastDebounceTime = millis();
  }
  
  if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY) {
    bool isButtonPressed = (buttonState == LOW);
    
    if (isButtonPressed != buttonPressed) {
      buttonPressed = isButtonPressed;
      
      if (buttonPressed) {
        pressStartTime = millis();
        clickCount++;
        Serial.println("Button pressed");
      } else {
        unsigned long pressDuration = millis() - pressStartTime;
        
        if (pressDuration < LONG_PRESS_DURATION && !longPressSent) {
          lastPressTime = millis();
          Serial.println("Button released (short press)");
        } else {
          Serial.println("Button released (after long press)");
        }
        longPressSent = false;
      }
    }
  }
  
  lastButtonState = buttonState;
  
  // Process long press (only when NAV2 is available)
  if (nav2Available && buttonPressed && !longPressSent && 
      (millis() - pressStartTime >= LONG_PRESS_DURATION)) {
    publishMessage(MQTT_TOPIC, "navStop");
    Serial.println("Long press detected - Sent 'stop'");
    longPressSent = true;
    clickCount = 0;
  }
  
  // Process double-click after button releases (only when NAV2 is available)
  if (nav2Available && !buttonPressed && clickCount == 2 && 
      (millis() - lastPressTime <= DOUBLE_CLICK_TIMEOUT)) {
    if (isPaused) {
      publishMessage(MQTT_TOPIC, "navCon");
      Serial.println("Double click - Sent 'resume'");
      isPaused = false;
    } else {
      publishMessage(MQTT_TOPIC, "navPause");
      Serial.println("Double click - Sent 'pause'");
      isPaused = true;
    }
    clickCount = 0;
  }
  
  // Reset click count if timeout occurs
  if (!buttonPressed && clickCount > 0 && 
      (millis() - lastPressTime > DOUBLE_CLICK_TIMEOUT)) {
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
    RGB_Light(brightness, 0, 0); // Red breathing when battery is low
  } else if (navColorMode == 1) {
    RGB_Light(0, brightness, 0); // Green breathing when navigation is running
  } else if (navColorMode == 2) {
    RGB_Light(brightness, brightness, 0); // Yellow breathing when navigation is stopped
  } else {
    RGB_Light(0, brightness, brightness); // Teal breathing for normal state
  }
  
  brightness += fadeAmount;
  if (brightness == 0 || brightness == 200) fadeAmount = -fadeAmount;
}

void handlePotentiometer() {
  sensorValue = analogRead(POT_PIN);
  
  if (lightSen) {
    outputValue = map(sensorValue, 0, 4095, minBrightness, 255);
    analogWrite(LED_PIN, outputValue);
    
    if (!lastLedState) {
      lastLedState = true;
    }
  } else {
    outputValue = 0;
    analogWrite(LED_PIN, outputValue);
    
    if (lastLedState) {
      Serial.println("Led Off!");
      lastLedState = false;
    }
  }
}

void handleMagneticSensor() {
  int magSen = digitalRead(MAG_SENSOR_PIN);
  
  if (magSen != lastMagSen) {
    if (magSen == 0) {
      Serial.println("MagSen is 0, handle attached!");
      publishMessage(MQTT_TOPIC, "navStop");
    } else {
      Serial.println("MagSen is 1, handle detached!");
      publishMessage(MQTT_TOPIC, "navCon");
    }
    lastMagSen = magSen;
  }
}

void handlePVDFPressureSensor() {
  pvdfValue = analogRead(PVDF_PIN);
  
  // Reduce noise by setting small values to 0
  if (pvdfValue < 250) pvdfValue = 0;
  
  bool shouldPublish = nav2Available;
  
  if (pvdfValue == 0 && pressureState) {
    Serial.print("No pressure: ");
    Serial.println(pvdfValue);
    if (shouldPublish) publishMessage(MQTT_TOPIC, "navStop");
    pressureState = false;
  } else if (pvdfValue > 1000 && !pressureState) {
    Serial.print("Pressure: ");
    Serial.println(pvdfValue);
    if (shouldPublish) publishMessage(MQTT_TOPIC, "navCon");
    pressureState = true;
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);
  
  strip.begin();
  strip.show();
  
  Serial.println("ESP32 MQTT Button Controller Starting...");
  
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(MAG_SENSOR_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  
  analogReadResolution(12);
  randomSeed(analogRead(0));
  
  setupWiFi();
  
  client.setServer(MQTT_SERVER, MQTT_PORT);
  client.setCallback(mqttCallback);
  
  bootTime = millis();
  
  Serial.println("Setup complete!");
}

void loop() {
  RGB_Breathing();
  yield();
  
  if (!client.connected()) {
    connectToMQTT();
  }
  
  client.loop();
  
  handleButton();
  handlePotentiometer();
  handleMagneticSensor();
  handlePVDFPressureSensor();
  
  delay(10);
}