#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

// Pin configuration
#define IR_SENSOR_PIN 15
#define NOMINAL_DISTANCE 1.3  // meters between marks
#define LED_PIN 2

// CALIBRATION - ADJUST THIS!
float CALIBRATION_FACTOR = 1.05;  // Variable, not #define

// Running Speed and Cadence Service UUIDs
#define RSC_SERVICE_UUID        "1814"
#define RSC_MEASUREMENT_UUID    "2A53"
#define RSC_FEATURE_UUID        "2A54"

// Global variables
unsigned long lastMarkTime = 0;
unsigned long currentMarkTime = 0;
float filteredSpeed = 0.0;
float rawSpeed = 0.0;
uint32_t totalDistance = 0;
unsigned long lastDetectionTime = 0;

// Improved filtering
#define SPEED_BUFFER_SIZE 7
float speedBuffer[SPEED_BUFFER_SIZE];
int bufferIndex = 0;
float lastStableSpeed = 0.0;

// Timing validation
unsigned long lastValidInterval = 0;
#define MIN_INTERVAL 200    // 200ms = ~23 km/h max
#define MAX_INTERVAL 3000   // 3000ms = ~1.5 km/h min
#define MAX_VARIATION 0.3   // 30% max variation from average

// Debounce settings
unsigned long debounceDelay = 150;  // Start with 150ms
unsigned long lastSensorRead = 0;
int lastSensorState = HIGH;  // Assuming HIGH = white

// BLE
BLECharacteristic *pRSCMeasurement;
BLEServer *pServer;
bool deviceConnected = false;
bool oldDeviceConnected = false;

// Running Speed and Cadence data structure
#pragma pack(push, 1)
struct RSCMeasurement {
    uint8_t flags;
    uint16_t instantaneousSpeed;
    uint8_t instantaneousCadence;
    uint16_t instantaneousStrideLength;
    uint32_t totalDistance;
};
#pragma pack(pop)

// BLE connection callbacks
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        deviceConnected = true;
        Serial.println("*** Device CONNECTED ***");
        digitalWrite(LED_PIN, HIGH);
        delay(200);
        digitalWrite(LED_PIN, LOW);
    };

    void onDisconnect(BLEServer* pServer) {
        deviceConnected = false;
        Serial.println("*** Device DISCONNECTED ***");
        digitalWrite(LED_PIN, LOW);
        delay(500);
        BLEDevice::startAdvertising();
        Serial.println("Advertising restarted");
    }
};

// Calculate speed
float calculateSpeed(unsigned long timeDiff) {
    if (timeDiff == 0) return 0.0;
    
    float speed_ms = NOMINAL_DISTANCE / (timeDiff / 1000.0);
    return speed_ms * 3.6 * CALIBRATION_FACTOR;
}

// Check if interval is reasonable
bool isValidInterval(unsigned long interval) {
    // Check absolute limits
    if (interval < MIN_INTERVAL || interval > MAX_INTERVAL) {
        return false;
    }
    
    // Check consistency with previous intervals
    if (lastValidInterval > 0) {
        float variation = abs((int)interval - (int)lastValidInterval) / (float)lastValidInterval;
        if (variation > MAX_VARIATION) {
            return false;
        }
    }
    
    return true;
}

// Update debounce based on speed
void updateDebounceDelay(float speed) {
    if (speed > 12.0) {
        debounceDelay = 80;    // Fast speeds
    } else if (speed > 8.0) {
        debounceDelay = 100;   // Medium-high speeds
    } else if (speed > 4.0) {
        debounceDelay = 120;   // Medium speeds
    } else {
        debounceDelay = 150;   // Slow speeds
    }
}

// Edge detection - detect BLACK (LOW) state
void checkForMark() {
    int sensorValue = digitalRead(IR_SENSOR_PIN);
    unsigned long now = millis();
    
    // DEBUG: Show sensor state occasionally
    static unsigned long lastDebug = 0;
    if (now - lastDebug > 2000) {
        Serial.printf("[SENSOR] State: %d | Debounce: %lums\n", sensorValue, debounceDelay);
        lastDebug = now;
    }
    
    // Detect FALLING edge (WHITE -> BLACK transition)
    if (sensorValue == LOW && lastSensorState == HIGH) {
        // Apply debounce
        if (now - lastDetectionTime > debounceDelay) {
            lastDetectionTime = now;
            
            // Store times
            lastMarkTime = currentMarkTime;
            currentMarkTime = now;
            
            // Calculate interval if we have previous mark
            if (lastMarkTime > 0) {
                unsigned long interval = currentMarkTime - lastMarkTime;
                
                // Validate interval
                if (isValidInterval(interval)) {
                    // Calculate raw speed
                    rawSpeed = calculateSpeed(interval);
                    
                    // Update last valid interval (weighted average)
                    if (lastValidInterval == 0) {
                        lastValidInterval = interval;
                    } else {
                        lastValidInterval = (lastValidInterval * 3 + interval) / 4;
                    }
                    
                    // Update speed buffer
                    speedBuffer[bufferIndex] = rawSpeed;
                    bufferIndex = (bufferIndex + 1) % SPEED_BUFFER_SIZE;
                    
                    // Calculate median (more stable than average)
                    float tempBuffer[SPEED_BUFFER_SIZE];
                    int validCount = 0;
                    
                    for (int i = 0; i < SPEED_BUFFER_SIZE; i++) {
                        if (speedBuffer[i] > 0.5) { // Ignore very low speeds
                            tempBuffer[validCount++] = speedBuffer[i];
                        }
                    }
                    
                    if (validCount >= 3) {
                        // Sort for median
                        for (int i = 0; i < validCount - 1; i++) {
                            for (int j = i + 1; j < validCount; j++) {
                                if (tempBuffer[i] > tempBuffer[j]) {
                                    float temp = tempBuffer[i];
                                    tempBuffer[i] = tempBuffer[j];
                                    tempBuffer[j] = temp;
                                }
                            }
                        }
                        
                        // Take median
                        filteredSpeed = tempBuffer[validCount / 2];
                        lastStableSpeed = filteredSpeed;
                        
                        // Update total distance
                        totalDistance += (uint32_t)(NOMINAL_DISTANCE * CALIBRATION_FACTOR * 100);
                        
                        // Update debounce based on speed
                        updateDebounceDelay(filteredSpeed);
                        
                        // LED feedback
                        digitalWrite(LED_PIN, HIGH);
                        delay(20);
                        digitalWrite(LED_PIN, LOW);
                        
                        // Debug output
                        Serial.printf("✅ MARK | Speed: %.2f km/h | Interval: %lu ms | Buffer: %d/%d\n",
                                     filteredSpeed, interval, validCount, SPEED_BUFFER_SIZE);
                    }
                } else {
                    Serial.printf("⚠️ SKIP | Invalid interval: %lu ms\n", interval);
                }
            } else {
                // First mark
                Serial.println("📍 First mark detected");
            }
        }
    }
    
    lastSensorState = sensorValue;
}

// Gradual speed decay when no marks detected
void updateSpeedDecay() {
    unsigned long now = millis();
    unsigned long timeSinceLastMark = now - currentMarkTime;
    
    if (filteredSpeed > 0 && timeSinceLastMark > 1000) {
        if (timeSinceLastMark > 3000) {
            // Reset after 3 seconds
            filteredSpeed = 0.0;
            rawSpeed = 0.0;
            lastValidInterval = 0;
            
            // Clear buffer
            for (int i = 0; i < SPEED_BUFFER_SIZE; i++) {
                speedBuffer[i] = 0.0;
            }
            
            Serial.println("⏰ Speed reset (timeout)");
        } else if (timeSinceLastMark > 1500) {
            // Gradual decay
            filteredSpeed *= 0.95;
            if (filteredSpeed < 0.5) filteredSpeed = 0.0;
        }
    }
}

// Setup BLE
void setupBLEFootpod() {
    BLEDevice::init("Stable_Footpod");
    
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());
    
    BLEService *pRSCService = pServer->createService(RSC_SERVICE_UUID);
    
    pRSCMeasurement = pRSCService->createCharacteristic(
        RSC_MEASUREMENT_UUID,
        BLECharacteristic::PROPERTY_READ | 
        BLECharacteristic::PROPERTY_NOTIFY
    );
    
    pRSCMeasurement->addDescriptor(new BLE2902());
    
    BLECharacteristic *pRSCFeature = pRSCService->createCharacteristic(
        RSC_FEATURE_UUID,
        BLECharacteristic::PROPERTY_READ
    );
    
    uint16_t features = 0x0003;
    pRSCFeature->setValue((uint8_t*)&features, 2);
    
    pRSCService->start();
    
    BLEAdvertising *pAdvertising = pServer->getAdvertising();
    pAdvertising->addServiceUUID(pRSCService->getUUID());
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);
    pAdvertising->setMaxPreferred(0x12);
    pAdvertising->start();
    
    Serial.println("✅ BLE initialized");
}

// Send BLE data
void sendRunningData() {
    if (!deviceConnected) return;
    
    static unsigned long lastSend = 0;
    unsigned long now = millis();
    
    // Send at 1Hz when moving, 0.5Hz when stationary
    unsigned long sendInterval = (filteredSpeed > 0) ? 1000 : 2000;
    
    if (now - lastSend > sendInterval) {
        RSCMeasurement data;
        data.flags = 0x03;
        
        float speed_ms = filteredSpeed / 3.6;
        if (speed_ms < 0) speed_ms = 0;
        data.instantaneousSpeed = (uint16_t)(speed_ms * 256);
        
        data.instantaneousCadence = 0;
        data.instantaneousStrideLength = 0;
        data.totalDistance = totalDistance;
        
        pRSCMeasurement->setValue((uint8_t*)&data, sizeof(data));
        pRSCMeasurement->notify();
        
        lastSend = now;
    }
}

// Calibration helper
void calibrateForSpeed(float actualSpeed) {
    if (filteredSpeed > 0.5 && actualSpeed > 0.5) {
        float newFactor = CALIBRATION_FACTOR * (actualSpeed / filteredSpeed);
        
        // Limit adjustments
        if (newFactor > 0.8 && newFactor < 1.2) {
            CALIBRATION_FACTOR = newFactor;
            Serial.printf("📐 Calibration updated: %.3f (was %.3f)\n", 
                         CALIBRATION_FACTOR, CALIBRATION_FACTOR / (actualSpeed / filteredSpeed));
        }
    }
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    pinMode(LED_PIN, OUTPUT);
    pinMode(IR_SENSOR_PIN, INPUT_PULLUP);
    
    Serial.println("\n\n========================================");
    Serial.println("      STABLE FOOTPOD V2.0");
    Serial.println("========================================");
    
    // LED test
    for(int i = 0; i < 3; i++) {
        digitalWrite(LED_PIN, HIGH);
        delay(150);
        digitalWrite(LED_PIN, LOW);
        delay(150);
    }
    
    Serial.println("\n⚙️  SETTINGS:");
    Serial.printf("Calibration: %.3f\n", CALIBRATION_FACTOR);
    Serial.printf("Distance: %.2f m\n", NOMINAL_DISTANCE);
    Serial.printf("Speed buffer: %d samples\n", SPEED_BUFFER_SIZE);
    Serial.printf("Min interval: %d ms (%.1f km/h max)\n", 
                  MIN_INTERVAL, NOMINAL_DISTANCE / (MIN_INTERVAL / 1000.0) * 3.6);
    Serial.printf("Max interval: %d ms (%.1f km/h min)\n", 
                  MAX_INTERVAL, NOMINAL_DISTANCE / (MAX_INTERVAL / 1000.0) * 3.6);
    
    Serial.println("\n📟 COMMANDS:");
    Serial.println("  'c X.XX' - Set calibration (e.g., 'c 1.05')");
    Serial.println("  'a X.XX' - Auto-calibrate to speed (e.g., 'a 9.0')");
    Serial.println("  'r' - Reset counters");
    Serial.println("  's' - Show statistics");
    Serial.println("\n========================================\n");
    
    // Initialize buffer
    for (int i = 0; i < SPEED_BUFFER_SIZE; i++) {
        speedBuffer[i] = 0.0;
    }
    
    setupBLEFootpod();
    
    Serial.println("🚀 System ready - waiting for consistent marks...");
}

void loop() {
    // Handle serial commands
    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');
        input.trim();
        
        if (input.startsWith("c ")) {
            float newCal = input.substring(2).toFloat();
            if (newCal >= 0.8 && newCal <= 1.2) {
                CALIBRATION_FACTOR = newCal;
                Serial.printf("📏 Calibration set to: %.3f\n", CALIBRATION_FACTOR);
            } else {
                Serial.println("❌ Invalid calibration (0.8 - 1.2)");
            }
        } 
        else if (input.startsWith("a ")) {
            float targetSpeed = input.substring(2).toFloat();
            calibrateForSpeed(targetSpeed);
        }
        else if (input == "r" || input == "R") {
            totalDistance = 0;
            filteredSpeed = 0.0;
            rawSpeed = 0.0;
            Serial.println("📊 Counters reset");
        }
        else if (input == "s" || input == "S") {
            Serial.println("\n📈 STATISTICS:");
            Serial.printf("Filtered speed: %.2f km/h\n", filteredSpeed);
            Serial.printf("Raw speed: %.2f km/h\n", rawSpeed);
            Serial.printf("Last interval: %lu ms\n", currentMarkTime - lastMarkTime);
            Serial.printf("Calibration: %.3f\n", CALIBRATION_FACTOR);
            Serial.printf("Total distance: %.1f m\n", totalDistance / 100.0);
            Serial.printf("Buffer fill: %d/%d\n", bufferIndex, SPEED_BUFFER_SIZE);
            Serial.println();
        }
    }
    
    // Core functions
    checkForMark();
    updateSpeedDecay();
    
    // Send BLE data
    if (deviceConnected) {
        sendRunningData();
    }
    
    // BLE connection management
    if (!deviceConnected && oldDeviceConnected) {
        delay(500);
        oldDeviceConnected = deviceConnected;
    }
    if (deviceConnected && !oldDeviceConnected) {
        oldDeviceConnected = deviceConnected;
    }
    
    // Status update
    static unsigned long lastStatus = 0;
    if (millis() - lastStatus > 2000) {
        if (filteredSpeed > 0) {
            Serial.printf("📊 Current: %.2f km/h | Distance: %.1f m | Cal: %.3f\n",
                         filteredSpeed, totalDistance / 100.0, CALIBRATION_FACTOR);
        }
        lastStatus = millis();
    }
    
    delay(5);
}
