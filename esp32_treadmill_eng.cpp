#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

// Pin configuration
#define IR_SENSOR_PIN 32
#define NOMINAL_DISTANCE 1.5  // meters between marks
#define LED_PIN 2

// FINE CALIBRATION - MODIFY THIS VALUE
#define CALIBRATION_FACTOR 1.053  // Increase by 5% to correct underestimation

// Running Speed and Cadence Service UUIDs
#define RSC_SERVICE_UUID        "1814"
#define RSC_MEASUREMENT_UUID    "2A53"
#define RSC_FEATURE_UUID        "2A54"

// Global variables
unsigned long lastMarkTime = 0;
unsigned long currentMarkTime = 0;
float filteredSpeed = 0.0;
uint32_t totalDistance = 0;

// Dynamic debouncing
unsigned long lastDetectionTime = 0;
unsigned long dynamicDebounceDelay = 100; // Start with 100ms

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
        Serial.println("*** Garmin CONNECTED ***");
        digitalWrite(LED_PIN, HIGH);
    };

    void onDisconnect(BLEServer* pServer) {
        deviceConnected = false;
        Serial.println("*** Garmin DISCONNECTED ***");
        digitalWrite(LED_PIN, LOW);
        delay(500);
        BLEDevice::startAdvertising();
        Serial.println("Advertising restarted");
    }
};

// Calculate speed with calibration factor
float calculateCalibratedSpeed(unsigned long timeDiff) {
    // Effective distance = nominal distance * calibration factor
    float effectiveDistance = NOMINAL_DISTANCE * CALIBRATION_FACTOR;
    float speed_ms = effectiveDistance / (timeDiff / 1000.0);
    return speed_ms * 3.6; // Convert to km/h
}

// Update debouncing based on current speed
void updateDynamicDebounce() {
    if (filteredSpeed < 3.0) {
        dynamicDebounceDelay = 150; // Slow: longer debouncing
    } else if (filteredSpeed < 8.0) {
        dynamicDebounceDelay = 80;  // Medium: medium debouncing
    } else {
        dynamicDebounceDelay = 40;  // Fast: short debouncing
    }
}

// Mark detection with dynamic debouncing
void checkForMark() {
    int sensorValue = digitalRead(IR_SENSOR_PIN);
    unsigned long now = millis();
    
    // AZDelivery sensor typically gives:
    // 0 = object detected (black)
    // 1 = no object (white)
    bool seesBlack = (sensorValue == 0);
    
    if (seesBlack) {
        // USE DYNAMIC DEBOUNCING
        if (now - lastDetectionTime > dynamicDebounceDelay) {
            lastDetectionTime = now;
            
            if (currentMarkTime > 0 && lastMarkTime > 0) {
                unsigned long timeDiff = currentMarkTime - lastMarkTime;
                
                // WIDER TIME RANGE FOR HIGH SPEEDS
                if (timeDiff >= 100 && timeDiff <= 15000) {
                    float speed_kmh = calculateCalibratedSpeed(timeDiff);
                    
                    // WIDER SPEED RANGE
                    if (speed_kmh >= 0.5 && speed_kmh <= 30.0) {
                        filteredSpeed = speed_kmh;
                        totalDistance += (uint32_t)(NOMINAL_DISTANCE * CALIBRATION_FACTOR * 100);
                        
                        Serial.printf("🎯 Speed: %.2f km/h (time: %lu ms) | Debounce: %lums\n", 
                                     speed_kmh, timeDiff, dynamicDebounceDelay);
                        
                        // Shorter LED flash at high speeds
                        digitalWrite(LED_PIN, HIGH);
                        delay(filteredSpeed > 10 ? 10 : 20);
                        digitalWrite(LED_PIN, LOW);
                    }
                }
            }
            
            lastMarkTime = currentMarkTime;
            currentMarkTime = now;
            
            // Update debouncing for next detection
            updateDynamicDebounce();
        }
    }
}

// Speed timeout
void checkSpeedTimeout() {
    unsigned long now = millis();
    
    if (filteredSpeed > 0 && (now - currentMarkTime > 5000)) {
        if (now - currentMarkTime > 10000) {
            filteredSpeed = 0.0;
            // Reset debounce to default value
            dynamicDebounceDelay = 100;
            Serial.println("⏰ Timeout: speed reset to zero");
        } else {
            filteredSpeed *= 0.95; // Gradual reduction
        }
    }
}

// Improved smoothing filter for high speeds
void updateFilteredSpeed() {
    static float previousSpeed = 0.0;
    
    if (filteredSpeed > 0) {
        // Dynamic smoothing factor based on speed
        float smoothingFactor;
        if (filteredSpeed < 5.0) {
            smoothingFactor = 0.8; // Heavy smoothing at low speeds
        } else if (filteredSpeed < 12.0) {
            smoothingFactor = 0.6; // Medium smoothing
        } else {
            smoothingFactor = 0.4; // Light smoothing at high speeds
        }
        
        filteredSpeed = (smoothingFactor * previousSpeed) + ((1 - smoothingFactor) * filteredSpeed);
        previousSpeed = filteredSpeed;
    }
}

// BLE Footpod configuration
void setupBLEFootpod() {
    BLEDevice::init("Garmin_Footpod");
    
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
    
    uint16_t features = 0x0003; // Instantaneous speed + total distance
    pRSCFeature->setValue((uint8_t*)&features, 2);
    
    pRSCService->start();
    
    BLEAdvertising *pAdvertising = pServer->getAdvertising();
    pAdvertising->addServiceUUID(pRSCService->getUUID());
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);
    pAdvertising->setMaxPreferred(0x12);
    pAdvertising->start();
    
    Serial.println("BLE Footpod ready");
}

// Send running data to Garmin
void sendRunningData() {
    if (!deviceConnected) return;
    
    RSCMeasurement data;
    data.flags = 0x03; // Instantaneous speed + total distance present
    
    float speed_ms = filteredSpeed / 3.6;
    if (speed_ms < 0) speed_ms = 0;
    data.instantaneousSpeed = (uint16_t)(speed_ms * 256); // Convert to BLE units
    
    data.instantaneousCadence = 0;
    data.instantaneousStrideLength = 0;
    data.totalDistance = totalDistance;
    
    pRSCMeasurement->setValue((uint8_t*)&data, sizeof(data));
    pRSCMeasurement->notify();
}

// High speed performance test
void testHighSpeedPerformance() {
    Serial.println("\n⚡ HIGH SPEED TEST");
    Serial.println("Set treadmill to different speeds:");
    Serial.println("6, 8, 10, 12 km/h and verify readings");
    Serial.println("Press ENTER to start test...");
    
    while (!Serial.available()) {
        delay(100);
    }
    Serial.read();
    
    Serial.println("Test running for 30 seconds...");
    Serial.println("Speed  | Debounce | Time between marks");
    Serial.println("-------|----------|-------------------");
    
    unsigned long testStart = millis();
    while (millis() - testStart < 30000) {
        if (filteredSpeed > 0) {
            Serial.printf("%6.1f | %5lu    | %6lu ms\n", 
                         filteredSpeed, dynamicDebounceDelay, 
                         currentMarkTime - lastMarkTime);
        }
        delay(1000);
    }
    
    Serial.println("Test completed!");
}

// Initial setup
void setup() {
    Serial.begin(115200);
    pinMode(LED_PIN, OUTPUT);
    pinMode(IR_SENSOR_PIN, INPUT);
    
    // Initial flash sequence
    for(int i = 0; i < 3; i++) {
        digitalWrite(LED_PIN, HIGH);
        delay(200);
        digitalWrite(LED_PIN, LOW);
        delay(200);
    }
    
    Serial.println("\n=== FOOTPOD SYSTEM - HIGH SPEED ===");
    Serial.printf("Calibration factor: %.3f\n", CALIBRATION_FACTOR);
    Serial.println("Dynamic debouncing: 40-150ms");
    Serial.println("Send 't' for high speed test");
    Serial.println("====================================\n");
    
    // Initialization
    filteredSpeed = 0.0;
    totalDistance = 0;
    dynamicDebounceDelay = 100;
    
    setupBLEFootpod();
    
    Serial.println("✅ System ready for high speeds");
}

// Main loop
void loop() {
    // Serial commands
    if (Serial.available()) {
        char cmd = Serial.read();
        if (cmd == 't' || cmd == 'T') {
            testHighSpeedPerformance();
        }
    }
    
    checkForMark();
    updateFilteredSpeed();
    checkSpeedTimeout();
    
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
    
    // Debug every 2 seconds
    static unsigned long lastDebug = 0;
    if (millis() - lastDebug > 2000) {
        Serial.printf("Speed: %5.1f km/h | Debounce: %lums | Distance: %5.1f m\n",
                     filteredSpeed, dynamicDebounceDelay, totalDistance / 100.0);
        lastDebug = millis();
    }
    
    delay(5); // Faster loop
}