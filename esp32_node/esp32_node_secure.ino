/*
 * Smart Farm IoT Node - ESP32-S3-DevKit-C-1
 * Secure Version with TLS/SSL Encryption
 * 
 * This code implements a smart farm monitoring and control system using:
 * - BMP280 temperature sensor
 * - Capacitive soil moisture sensor
 * - 5V DC fan (controlled via MOSFET transistor)
 * - DC water pump (controlled via MOSFET transistor)
 * 
 * Communication:
 * - WiFi connection to local network
 * - MQTT over TLS/SSL to HiveMQ Cloud broker
 * - Secure authentication with username/password
 */

#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <ArduinoJson.h>
#include <WiFiClientSecure.h>
#include <esp_wifi.h>

// MAC address configuration
// Some ESP32-S3 boards come with invalid MAC (00:00:00:00:00:00)
// Setting a custom MAC address fixes WiFi connection issues
uint8_t customMACAddress[] = {0x02, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E};

// WiFi event handler for connection status monitoring
void WiFiEvent(WiFiEvent_t event) {
  switch(event) {
    case ARDUINO_EVENT_WIFI_STA_START:
      Serial.println("WiFi Started");
      break;
    case ARDUINO_EVENT_WIFI_STA_CONNECTED:
      Serial.println("WiFi Connected to AP");
      break;
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      Serial.print("WiFi Got IP: ");
      Serial.println(WiFi.localIP());
      break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      Serial.print("WiFi Disconnected. Reason: ");
      Serial.println(WiFi.status());
      break;
    default:
      break;
  }
}

// ========== CONFIGURATION - UPDATE THESE VALUES ==========
const char* ssid = "YOUR_WIFI_SSID";                    // Your WiFi network name
const char* password = "YOUR_WIFI_PASSWORD";        // Your WiFi password
const char* mqtt_server = "YOUR_BROKER_URL.hivemq.cloud";  // HiveMQ Cloud broker URL
const int mqtt_port = 8883;                // TLS port (encrypted)
const char* mqtt_username = "YOUR_MQTT_USERNAME";  // HiveMQ Cloud username
const char* mqtt_password = "YOUR_MQTT_PASSWORD";  // HiveMQ Cloud password
const char* node_id = "node001";           // Unique identifier for this node (change for multiple nodes)
// ==========================================================

// MQTT Topic Definitions
// Topics follow pattern: farm/{node_id}/{type}
String topic_sensors = "farm/" + String(node_id) + "/sensors";      // Publish sensor data
String topic_control = "farm/" + String(node_id) + "/control";      // Receive control commands
String topic_thresholds = "farm/" + String(node_id) + "/thresholds"; // Receive threshold updates

// Pin Definitions for ESP32-S3-DevKit-C-1
#define SOIL_MOISTURE_PIN 4     // ADC pin for soil moisture sensor (GPIO 4)
#define FAN_PIN 5               // GPIO for fan control via MOSFET transistor (GPIO 5)
#define PUMP_PIN 6              // GPIO for DC pump control via MOSFET transistor (GPIO 6)
#define BMP280_SDA 39           // I2C SDA pin for BMP280 (GPIO 39)
#define BMP280_SCL 40           // I2C SCL pin for BMP280 (GPIO 40)

// Hardware Objects
WiFiClientSecure espClientSecure;  // Secure WiFi client for TLS/SSL
PubSubClient mqtt(espClientSecure); // MQTT client using secure connection
Adafruit_BMP280 bmp;                // BMP280 temperature sensor

// Sensor Reading Variables
float temperature = 0;              // Temperature in Celsius
int soilMoistureRaw = 0;            // Raw ADC reading from moisture sensor
float soilMoisturePercent = 0;      // Moisture percentage (0-100%)

// Actuator State Variables
bool fanOn = false;                 // Fan state (true = ON, false = OFF)
bool pumpOn = false;                // Pump state (true = ON, false = OFF)
bool autoMode = true;               // Auto control mode (true = enabled, false = manual)

// Control Thresholds
float tempThreshold = 30.0;         // Temperature threshold for fan activation (°C)
float moistureThreshold = 30.0;     // Moisture threshold for pump activation (%)

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("=== Smart Farm Node Starting (SECURE TLS) ===");
  
  // Initialize GPIO pins
  pinMode(FAN_PIN, OUTPUT);
  digitalWrite(FAN_PIN, LOW);
  Serial.print("Fan pin ");
  Serial.print(FAN_PIN);
  Serial.println(" set to OUTPUT, LOW");
  
  // Initialize pump control pin
  pinMode(PUMP_PIN, OUTPUT);
  digitalWrite(PUMP_PIN, LOW);
  pumpOn = false;
  delay(100);  // Ensure pin is stable
  Serial.print("Pump pin ");
  Serial.print(PUMP_PIN);
  Serial.println(" set to OUTPUT, LOW (OFF)");
  
  // Initialize I2C bus for BMP280
  Wire.begin(BMP280_SDA, BMP280_SCL);
  
  // Initialize BMP280 temperature sensor
  // Try both common I2C addresses (0x76 and 0x77)
  if (!bmp.begin(0x76)) {
    Serial.println("BMP280 not found at address 0x76, trying 0x77...");
    if (!bmp.begin(0x77)) {
      Serial.println("ERROR: BMP280 not found! Check wiring.");
      Serial.println("Expected connections:");
      Serial.println("  BMP280 VCC -> ESP32 3.3V");
      Serial.println("  BMP280 GND -> ESP32 GND");
      Serial.println("  BMP280 SDA -> ESP32 GPIO 39");
      Serial.println("  BMP280 SCL -> ESP32 GPIO 40");
      while(1) delay(10);  // Halt execution
    }
  }
  Serial.println("BMP280 initialized successfully");
  
  // Configure TLS/SSL for secure MQTT connection
  // Note: setInsecure() skips certificate validation (for testing)
  // For production, use: espClientSecure.setCACert(root_ca_certificate);
  espClientSecure.setInsecure();
  
  // Set custom MAC address (fixes WiFi issues on some ESP32-S3 boards)
  esp_wifi_set_mac(WIFI_IF_STA, customMACAddress);
  
  // Connect to WiFi network
  connectWiFi();
  
  // Setup MQTT connection if WiFi is connected
  if (WiFi.status() == WL_CONNECTED) {
    mqtt.setServer(mqtt_server, mqtt_port);
    mqtt.setCallback(mqttCallback);
    
    Serial.println("\nSetup complete!");
    Serial.println("Using TLS/SSL encryption (port 8883)");
    Serial.println("Ready to connect to MQTT broker");
  } else {
    Serial.println("\nWarning: WiFi not connected - MQTT setup skipped");
    Serial.println("Will retry WiFi connection in main loop");
  }
}

void loop() {
  // Retry WiFi connection if disconnected
  if (WiFi.status() != WL_CONNECTED) {
    static unsigned long lastWiFiRetry = 0;
    if (millis() - lastWiFiRetry > 10000) {  // Retry every 10 seconds
      Serial.println("\nRetrying WiFi connection...");
      connectWiFi();
      lastWiFiRetry = millis();
    }
    delay(1000);
    return;  // Don't proceed until WiFi is connected
  }
  
  // Maintain MQTT connection
  if (!mqtt.connected()) {
    connectMQTT();
  }
  mqtt.loop();  // Process incoming MQTT messages
  
  // Read sensors and publish data every 5 seconds
  static unsigned long lastRead = 0;
  if (millis() - lastRead > 5000) {
    readSensors();
    publishData();
    lastRead = millis();
  }
  
  // Execute automatic control if enabled
  if (autoMode) {
    autoControl();
  }
}

void connectWiFi() {
  Serial.println("\n=== WiFi Connection ===");
  Serial.print("SSID: '");
  Serial.print(ssid);
  Serial.print("' (length: ");
  Serial.print(strlen(ssid));
  Serial.println(")");
  
  // Reset WiFi state completely
  Serial.println("Resetting WiFi state...");
  WiFi.disconnect(true);  // Disconnect and erase stored credentials
  delay(2000);
  WiFi.mode(WIFI_OFF);
  delay(1000);
  WiFi.mode(WIFI_STA);  // Set to station mode (client mode)
  delay(1000);
  
  // Configure WiFi settings
  WiFi.setAutoReconnect(false);
  WiFi.persistent(false);  // Don't save credentials to flash
  
  // Register event handler for connection monitoring
  WiFi.onEvent(WiFiEvent);
  
  // Set WiFi transmit power to 8.5 dBm (recommended for stability)
  // High power (19.5 dBm) can cause connection issues on some ESP32-S3 boards
  WiFi.setTxPower(WIFI_POWER_8_5dBm);
  Serial.println("WiFi power set to 8.5 dBm");
  
  // Clear any previous connection attempts
  WiFi.disconnect(true);
  delay(1000);
  
  // Use BSSID and channel method for more reliable connection
  Serial.println("Scanning for network...");
  
  String targetSSID = String(ssid);
  uint8_t bestBSSID[6] = {0};
  int32_t bestChannel = 0;
  int32_t bestRSSI = -1000;
  bool bssidFound = false;
  
  // Scan networks and find the best signal
  int n = WiFi.scanNetworks();
  Serial.print("Found ");
  Serial.print(n);
  Serial.println(" networks");
  
  for (int i = 0; i < n; i++) {
    if (WiFi.SSID(i) == targetSSID) {
      int32_t rssi = WiFi.RSSI(i);
      if (rssi > bestRSSI) {
        bestRSSI = rssi;
        bestChannel = WiFi.channel(i);
        memcpy(bestBSSID, WiFi.BSSID(i), 6);
        bssidFound = true;
        Serial.print("  Found target network on channel ");
        Serial.print(bestChannel);
        Serial.print(", RSSI: ");
        Serial.print(bestRSSI);
        Serial.println(" dBm");
      }
    }
  }
  
  // Connect using BSSID/channel method if found, otherwise use standard method
  if (!bssidFound) {
    Serial.println("Network not found in scan, using standard connection method...");
    WiFi.begin(ssid, password);
  } else {
    Serial.print("Connecting using BSSID/channel method...");
    WiFi.begin(ssid, password, bestChannel, bestBSSID);
  }
  
  Serial.println("Waiting for connection...");
  delay(3000);
  
  // Check initial connection status
  int initialStatus = WiFi.status();
  Serial.print("Initial WiFi status: ");
  Serial.print(initialStatus);
  switch(initialStatus) {
    case 0: Serial.println(" (IDLE)"); break;
    case 1: Serial.println(" (NO_SSID_AVAIL)"); break;
    case 3: Serial.println(" (CONNECTED)"); break;
    case 4: Serial.println(" (CONNECTION_FAILED)"); break;
    case 6: Serial.println(" (WRONG_PASSWORD)"); break;
    default: Serial.println(" (UNKNOWN)"); break;
  }
  
  // If already connected, return
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("Connected!");
    return;
  }
  
  // Wait for connection with timeout
  int attempts = 0;
  int maxAttempts = 40;  // 20 seconds total (40 * 500ms)
  
  while (WiFi.status() != WL_CONNECTED && attempts < maxAttempts) {
    delay(500);
    Serial.print(".");
    attempts++;
    
    // Print status every 5 seconds
    if (attempts % 10 == 0) {
      int status = WiFi.status();
      Serial.print(" [");
      Serial.print(status);
      switch(status) {
        case 0: Serial.print(":IDLE"); break;
        case 1: Serial.print(":NO_SSID"); break;
        case 3: Serial.print(":CONNECTED"); break;
        case 4: Serial.print(":FAILED"); break;
        case 6: Serial.print(":WRONG_PASSWORD"); break;
        default: Serial.print(":UNKNOWN"); break;
      }
      Serial.print("] ");
    }
  }
  
  // Check final connection status
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi CONNECTED!");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    Serial.print("Signal strength: ");
    Serial.print(WiFi.RSSI());
    Serial.println(" dBm");
    Serial.print("Gateway: ");
    Serial.println(WiFi.gatewayIP());
  } else {
    Serial.println("\nWiFi connection FAILED!");
    int status = WiFi.status();
    Serial.print("Final status: ");
    Serial.println(status);
    
    Serial.println("\nStatus code meanings:");
    Serial.println("  0 = IDLE");
    Serial.println("  1 = NO_SSID_AVAIL (network not found)");
    Serial.println("  4 = CONNECTION_FAILED");
    Serial.println("  6 = WRONG_PASSWORD");
    
    if (status == 6) {
      Serial.println("\nWRONG PASSWORD ERROR:");
      Serial.println("  - Double-check password in code");
      Serial.println("  - Make sure no extra spaces");
      Serial.println("  - Verify password on router/hotspot");
      Serial.println("  - Try re-typing password in code");
    } else if (status == 1) {
      Serial.println("\nNETWORK NOT FOUND:");
      Serial.print("  Looking for: '");
      Serial.print(ssid);
      Serial.println("'");
      Serial.println("  - Check SSID spelling (case-sensitive)");
      Serial.println("  - Make sure router/hotspot is ON");
      Serial.println("  - Ensure router is broadcasting 2.4GHz WiFi");
    }
    
    Serial.println("\nRetrying in 3 seconds...");
    delay(3000);
  }
}

void connectMQTT() {
  while (!mqtt.connected()) {
    Serial.print("Connecting to HiveMQ Cloud broker (TLS)...");
    Serial.print(mqtt_server);
    Serial.print(":");
    Serial.print(mqtt_port);
    Serial.print("...");
    
    // Generate unique client ID
    String clientId = "ESP32_" + String(node_id) + "_" + String(random(1000, 9999));
    
    // Connect with authentication
    if (mqtt.connect(clientId.c_str(), mqtt_username, mqtt_password)) {
      Serial.println(" connected (SECURE)!");
      
      // Subscribe to control and threshold topics
      mqtt.subscribe(topic_control.c_str());
      mqtt.subscribe(topic_thresholds.c_str());
      
      Serial.print("Subscribed to: ");
      Serial.print(topic_control);
      Serial.print(", ");
      Serial.println(topic_thresholds);
    } else {
      Serial.print(" failed, rc=");
      Serial.print(mqtt.state());
      Serial.println(" retrying in 5 seconds...");
      delay(5000);
    }
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  // Convert payload to string
  char message[length + 1];
  memcpy(message, payload, length);
  message[length] = '\0';
  
  Serial.print("Received MQTT message: ");
  Serial.println(message);
  
  // Parse JSON message
  StaticJsonDocument<200> doc;
  deserializeJson(doc, message);
  
  // Handle fan control command
  if (doc.containsKey("fan")) {
    bool newFanState = doc["fan"];
    Serial.print("Fan command (MANUAL): ");
    Serial.println(newFanState ? "ON" : "OFF");
    
    // Update state
    fanOn = newFanState;
    
    // Control fan via GPIO immediately
    digitalWrite(FAN_PIN, fanOn ? HIGH : LOW);
    delay(10);  // Small delay to ensure pin state is set
    
    // Verify pin state
    int pinState = digitalRead(FAN_PIN);
    Serial.print("Pin ");
    Serial.print(FAN_PIN);
    Serial.print(" set to ");
    Serial.print(fanOn ? "HIGH" : "LOW");
    Serial.print(" (readback: ");
    Serial.print(pinState);
    Serial.print(") - Expected: ");
    Serial.println(fanOn ? "HIGH" : "LOW");
    
    // Check if pin state matches expected
    if ((fanOn && pinState != HIGH) || (!fanOn && pinState != LOW)) {
      Serial.println("WARNING: Pin state mismatch! Retrying...");
      digitalWrite(FAN_PIN, fanOn ? HIGH : LOW);
      delay(10);
      pinState = digitalRead(FAN_PIN);
      Serial.print("After retry - Pin state: ");
      Serial.println(pinState);
    }
    
    // Disable auto mode when manual control is used
    if (autoMode) {
      autoMode = false;
      Serial.println("Auto mode DISABLED (manual control active)");
    }
    
    // Publish updated state immediately
    publishData();
  }
  
  // Handle pump control command
  if (doc.containsKey("pump")) {
    bool newPumpState = doc["pump"];
    Serial.println("========================================");
    Serial.print("PUMP COMMAND RECEIVED: ");
    Serial.println(newPumpState ? "ON" : "OFF");
    Serial.println("========================================");
    
    // Update state
    pumpOn = newPumpState;
    
    // Force pin to OUTPUT mode (in case it was changed)
    pinMode(PUMP_PIN, OUTPUT);
    
    // Control pump via GPIO immediately
    digitalWrite(PUMP_PIN, pumpOn ? HIGH : LOW);
    delay(50);  // Longer delay to ensure pin state is set
    
    // Verify pin state multiple times
    Serial.println("Verifying pin state...");
    for (int i = 0; i < 3; i++) {
      int pinState = digitalRead(PUMP_PIN);
      Serial.print("  Read #");
      Serial.print(i + 1);
      Serial.print(": Pin ");
      Serial.print(PUMP_PIN);
      Serial.print(" = ");
      Serial.print(pinState);
      Serial.print(" (");
      Serial.print(pinState == HIGH ? "HIGH/3.3V" : "LOW/0V");
      Serial.print(") - Expected: ");
      Serial.println(pumpOn ? "HIGH/3.3V" : "LOW/0V");
      delay(10);
    }
    
    // Measure actual voltage if possible (requires multimeter)
    Serial.println("\nTROUBLESHOOTING CHECKLIST:");
    Serial.println("1. Use multimeter to check GPIO 6 voltage:");
    Serial.print("   - Should read ");
    Serial.print(pumpOn ? "~3.3V" : "~0V");
    Serial.println(" when pump command is sent");
    Serial.println("2. Check MOSFET Gate voltage:");
    Serial.print("   - Should read ");
    Serial.print(pumpOn ? "~3.3V" : "~0V");
    Serial.println(" (after 330Ω resistor)");
    Serial.println("3. Check MOSFET connections:");
    Serial.println("   - Gate: Connected to GPIO 6 via 330Ω");
    Serial.println("   - Source: Connected to GND");
    Serial.println("   - Drain: Connected to pump negative");
    Serial.println("4. Check power supply:");
    Serial.println("   - Pump positive: External 5V supply");
    Serial.println("   - Common ground: ESP32 GND = Power supply GND");
    Serial.println("5. IRFZ44N may need >3.3V to fully turn on!");
    Serial.println("   - Vgs(th) = 2-4V, 3.3V might not be enough");
    Serial.println("   - Consider logic-level MOSFET (e.g., IRLZ44N)");
    Serial.println("========================================\n");
    
    // Disable auto mode when manual control is used
    if (autoMode) {
      autoMode = false;
      Serial.println("Auto mode DISABLED (manual control active)");
    }
    
    // Publish updated state immediately
    publishData();
  }
  
  // Handle auto mode toggle
  if (doc.containsKey("auto_mode")) {
    autoMode = doc["auto_mode"];
    Serial.print("Auto mode: ");
    Serial.println(autoMode ? "ENABLED" : "DISABLED");
  }
  
  // Handle threshold updates
  bool thresholdChanged = false;
  
  if (doc.containsKey("temp_threshold")) {
    float oldThreshold = tempThreshold;
    float newThreshold = doc["temp_threshold"];
    // Only update if value actually changed (avoid unnecessary logging)
    if (abs(oldThreshold - newThreshold) > 0.01) {
      tempThreshold = newThreshold;
      Serial.print("Temperature threshold updated: ");
      Serial.print(oldThreshold);
      Serial.print("°C → ");
      Serial.print(tempThreshold);
      Serial.println("°C");
      thresholdChanged = true;
    }
  }
  
  if (doc.containsKey("moisture_threshold")) {
    float oldThreshold = moistureThreshold;
    float newThreshold = doc["moisture_threshold"];
    // Only update if value actually changed
    if (abs(oldThreshold - newThreshold) > 0.01) {
      moistureThreshold = newThreshold;
      Serial.print("Moisture threshold updated: ");
      Serial.print(oldThreshold);
      Serial.print("% → ");
      Serial.print(moistureThreshold);
      Serial.println("%");
      thresholdChanged = true;
    }
  }
  
  // Re-evaluate automatic control if thresholds changed
  if (thresholdChanged && autoMode) {
    Serial.println("Re-evaluating auto control with new thresholds...");
    autoControl();
    
    Serial.print("Re-evaluation complete - Fan: ");
    Serial.print(fanOn ? "ON" : "OFF");
    Serial.print(", Pump: ");
    Serial.println(pumpOn ? "ON" : "OFF");
  }
}

void readSensors() {
  // Read temperature from BMP280
  temperature = bmp.readTemperature();
  
  // Read raw ADC value from soil moisture sensor
  soilMoistureRaw = analogRead(SOIL_MOISTURE_PIN);
  
  // Calibration values for soil moisture sensor
  // IMPORTANT: These values must be calibrated for your specific sensor!
  // 
  // Calibration procedure:
  // 1. Upload code and open Serial Monitor
  // 2. Hold sensor in AIR (completely dry) - note the raw value
  // 3. Dip sensor in WATER (completely wet) - note the raw value
  // 4. Update dryValue and wetValue below with your measured values
  //
  // The sensor reading will be mapped from [dryValue, wetValue] to [0%, 100%]
  int dryValue = 2000;   // Raw ADC value when sensor is dry (in air)
  int wetValue = 3500;   // Raw ADC value when sensor is wet (in water)
  
  // Convert raw ADC reading to percentage (0-100%)
  soilMoisturePercent = map(soilMoistureRaw, dryValue, wetValue, 0, 100);
  soilMoisturePercent = constrain(soilMoisturePercent, 0, 100);
  
  // Print sensor readings
  Serial.print("Temp: ");
  Serial.print(temperature);
  Serial.print("°C, Moisture: ");
  Serial.print(soilMoisturePercent);
  Serial.print("% (raw: ");
  Serial.print(soilMoistureRaw);
  Serial.println(")");
}

void publishData() {
  // Create JSON document with sensor data and actuator states
  StaticJsonDocument<200> doc;
  doc["node_id"] = node_id;
  doc["temperature"] = temperature;
  doc["soil_moisture"] = soilMoisturePercent;
  doc["soil_moisture_raw"] = soilMoistureRaw;
  doc["fan"] = fanOn;
  doc["pump"] = pumpOn;
  doc["auto_mode"] = autoMode;
  
  // Serialize to JSON string and publish to MQTT
  char buffer[200];
  serializeJson(doc, buffer);
  mqtt.publish(topic_sensors.c_str(), buffer);
}

void autoControl() {
  // Automatic fan control based on temperature threshold
  bool shouldFanBeOn = (temperature > tempThreshold);
  
  if (shouldFanBeOn && !fanOn) {
    // Turn fan ON when temperature exceeds threshold
    fanOn = true;
    digitalWrite(FAN_PIN, HIGH);
    Serial.print("Fan ON (auto) - Temp ");
    Serial.print(temperature);
    Serial.print("°C > threshold ");
    Serial.print(tempThreshold);
    Serial.println("°C");
  } else if (!shouldFanBeOn && fanOn) {
    // Turn fan OFF when temperature drops to or below threshold
    if (temperature <= tempThreshold) {
      fanOn = false;
      digitalWrite(FAN_PIN, LOW);
      Serial.print("Fan OFF (auto) - Temp ");
      Serial.print(temperature);
      Serial.print("°C <= threshold ");
      Serial.print(tempThreshold);
      Serial.println("°C");
    }
  }
  
  // Automatic pump control based on moisture threshold
  // Uses hysteresis to prevent rapid switching (fluttering)
  
  // Define the "turn off" point: 5% above the turn on threshold
  // This creates a hysteresis band to prevent rapid on/off switching
  float turnOffMoisture = moistureThreshold + 5.0;
  
  // Special case: If threshold is 0% and moisture is 0%, turn pump off
  // This handles the edge case where threshold equals current reading
  if (moistureThreshold == 0.0 && soilMoisturePercent == 0.0) {
    if (pumpOn) {
      pumpOn = false;
      pinMode(PUMP_PIN, OUTPUT);  // Ensure pin is OUTPUT
      digitalWrite(PUMP_PIN, LOW);
      delay(10);  // Small delay for stability
      Serial.print("Pump OFF (auto) - 0% threshold met with 0% moisture - Pin ");
      Serial.print(PUMP_PIN);
      Serial.println(" set to LOW");
    }
  }
  // Rule 1: Turn pump ON if moisture is below threshold
  else if (soilMoisturePercent < moistureThreshold) {
    if (!pumpOn) {
      pumpOn = true;
      pinMode(PUMP_PIN, OUTPUT);  // Ensure pin is OUTPUT
      digitalWrite(PUMP_PIN, HIGH);
      delay(10);  // Small delay for stability
      Serial.print("Pump ON (auto) - Moisture ");
      Serial.print(soilMoisturePercent);
      Serial.print("% < threshold ");
      Serial.print(moistureThreshold);
      Serial.print("% - Pin ");
      Serial.print(PUMP_PIN);
      Serial.println(" set to HIGH");
    }
  }
  // Rule 2: Turn pump OFF if moisture is at or above turn-off point
  else if (soilMoisturePercent >= turnOffMoisture) {
    if (pumpOn) {
      pumpOn = false;
      pinMode(PUMP_PIN, OUTPUT);  // Ensure pin is OUTPUT
      digitalWrite(PUMP_PIN, LOW);
      delay(10);  // Small delay for stability
      Serial.print("Pump OFF (auto) - Moisture ");
      Serial.print(soilMoisturePercent);
      Serial.print("% >= turn-off point ");
      Serial.print(turnOffMoisture);
      Serial.print("% - Pin ");
      Serial.print(PUMP_PIN);
      Serial.println(" set to LOW");
    }
  }
  // Rule 3: Hysteresis band - do nothing if moisture is between threshold and turn-off point
  // This prevents rapid switching when moisture is near the threshold
  else {
    // Moisture is between threshold and turnOffMoisture (e.g., 30% to 35%)
    // Leave pump in current state to prevent fluttering
  }
}
