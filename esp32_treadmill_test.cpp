#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

// Pin configuration
#define IR_SENSOR_PIN 15
#define NOMINAL_DISTANCE 1.3  // meters between marks
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

// New constants for high speed optimization
#define MIN_VALID_TIME_DIFF 50    // Reduced from 100ms for high speeds
#define MAX_VALID_TIME_DIFF 8000  // Reduced from 15000ms
#define MAX_SPEED_KMH 20.0        // More realistic max speed

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

// Calculate speed with non-linear calibration
float calculateCalibratedSpeed(unsigned long timeDiff) {
    float speed_raw = NOMINAL_DISTANCE / (timeDiff / 1000.0) * 3.6; // km/h
    
    // Non-linear correction based on speed
    float correction;
    if (speed_raw < 4.0) {
        correction = 1.00;  // No correction at low speed
    } else if (speed_raw < 7.0) {
        correction = 1.02;  // +2% at medium speed
    } else if (speed_raw < 10.0) {
        correction = 0.97;  // -3% at 7-10 km/h
    } else if (speed_raw < 15.0) {
        correction = 0.95;  // -5% at 10-15 km/h
    } else {
        correction = 0.93;  // -7% at high speed
    }
    
    return speed_raw * correction * CALIBRATION_FACTOR;
}

// Update debouncing based on current speed
void updateDynamicDebounce() {
    if (filteredSpeed < 3.0) {
        dynamicDebounceDelay = 80;  // Slow: longer debouncing (reduced from 150)
    } else if (filteredSpeed < 8.0) {
        dynamicDebounceDelay = 40;  // Medium: medium debouncing (reduced from 80)
    } else {
        dynamicDebounceDelay = 20;  // Fast: short debouncing (reduced from 40)
    }
}

// Mark detection with dynamic debouncing - OPTIMIZED VERSION
void checkForMark() {
    int sensorValue = digitalRead(IR_SENSOR_PIN);
    unsigned long now = millis();
    
    // AZDelivery sensor typically gives:
    // 0 = object detected (black)
    // 1 = no object (white)
    bool seesBlack = (sensorValue == 0);
    
    if (seesBlack) {
        // ADAPTIVE DEBOUNCING - MORE AGGRESSIVE
        unsigned long minDebounce = 20;  // Absolute minimum
        unsigned long maxDebounce = 80;  // Reduced maximum
        
        // Calculate debounce based on current speed
        if (filteredSpeed > 8.0) {
            dynamicDebounceDelay = minDebounce;
        } else if (filteredSpeed > 5.0) {
            dynamicDebounceDelay = 30;
        } else if (filteredSpeed > 3.0) {
            dynamicDebounceDelay = 50;
        } else {
            dynamicDebounceDelay = maxDebounce;
        }
        
        if (now - lastDetectionTime > dynamicDebounceDelay) {
            lastDetectionTime = now;
            
            if (currentMarkTime > 0 && lastMarkTime > 0) {
                unsigned long timeDiff = currentMarkTime - lastMarkTime;
                
                // OPTIMIZED TIME RANGE FOR TREADMILL
                if (timeDiff >= MIN_VALID_TIME_DIFF && timeDiff <= MAX_VALID_TIME_DIFF) {
                    float speed_kmh = calculateCalibratedSpeed(timeDiff);
                    
                    // REALISTIC SPEED RANGE FOR TREADMILL
                    if (speed_kmh >= 0.5 && speed_kmh <= MAX_SPEED_KMH) {
                        // ADD LATENCY COMPENSATION
                        unsigned long correctedTimeDiff = timeDiff + 5; // +5ms compensation
                        
                        speed_kmh = calculateCalibratedSpeed(correctedTimeDiff);
                        filteredSpeed = speed_kmh;
                        totalDistance += (uint32_t)(NOMINAL_DISTANCE * CALIBRATION_FACTOR * 100);
                        
                        Serial.printf("🎯 Speed: %.2f km/h (raw: %.2f, time: %lu ms) | Debounce: %lums\n", 
                                     speed_kmh, NOMINAL_DISTANCE / (timeDiff / 1000.0) * 3.6, 
                                     timeDiff, dynamicDebounceDelay);
                        
                        // Shorter LED flash at high speeds
                        digitalWrite(LED_PIN, HIGH);
                        delay(filteredSpeed > 8 ? 5 : 10);
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

// Improved smoothing filter with moving average
void updateFilteredSpeed() {
    static float speedHistory[5] = {0};
    static int historyIndex = 0;
    
    if (filteredSpeed > 0) {
        // Add to history
        speedHistory[historyIndex] = filteredSpeed;
        historyIndex = (historyIndex + 1) % 5;
        
        // Calculate moving average
        float sum = 0;
        int count = 0;
        for (int i = 0; i < 5; i++) {
            if (speedHistory[i] > 0) {
                sum += speedHistory[i];
                count++;
            }
        }
        
        if (count > 0) {
            filteredSpeed = sum / count;
        }
    }
}

// BLE Footpod configuration
void setupBLEFootpod() {
    BLEDevice::init("esp32_Footpod");
    
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

// Send running data to device
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

// Detailed debug information
void printDetailedDebug() {
    Serial.println("\n=== DEBUG DETAILS ===");
    Serial.printf("Calibration: %.3f\n", CALIBRATION_FACTOR);
    Serial.printf("Nominal distance: %.2f m\n", NOMINAL_DISTANCE);
    Serial.printf("Current debounce: %lu ms\n", dynamicDebounceDelay);
    Serial.printf("Time since last mark: %lu ms\n", millis() - currentMarkTime);
    Serial.printf("Current filtered speed: %.2f km/h\n", filteredSpeed);
    Serial.println("=====================\n");
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
    
    Serial.println("\n=== FOOTPOD SYSTEM - HIGH SPEED OPTIMIZED ===");
    Serial.printf("Calibration factor: %.3f\n", CALIBRATION_FACTOR);
    Serial.printf("Nominal distance: %.2f m\n", NOMINAL_DISTANCE);
    Serial.println("Dynamic debouncing: 20-80ms");
    Serial.println("Speed range: 0.5-20.0 km/h");
    Serial.println("Commands:");
    Serial.println("  't' - High speed test");
    Serial.println("  'd' - Detailed debug info");
    Serial.println("=============================================\n");
    
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
        } else if (cmd == 'd' || cmd == 'D') {
            printDetailedDebug();
        } else if (cmd == 'c' || cmd == 'C') {
            // Real-time calibration adjustment
            Serial.print("Current calibration factor: ");
            Serial.println(CALIBRATION_FACTOR);
            Serial.println("To change, modify #define CALIBRATION_FACTOR and re-upload");
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
