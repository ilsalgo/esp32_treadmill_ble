#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

/* ================= CONFIG ================= */

#define SENSOR_PIN        34
#define ROLLER_DISTANCE   1.80        // metri per impulso
#define DEBOUNCE_US       1500
#define SPEED_TIMEOUT_US  2000000
#define FILTER_ALPHA      0.3

/* ================= BLE UUID ================= */

#define RSC_SERVICE_UUID  BLEUUID((uint16_t)0x1814)
#define RSC_MEAS_UUID     BLEUUID((uint16_t)0x2A53)

/* ================= VARIABILI ================= */

volatile unsigned long lastPulseTime = 0;
volatile unsigned long pulseInterval = 0;
volatile bool newPulse = false;

float speed_kmh = 0.0;
float speed_kmh_filtered = 0.0;

BLECharacteristic* rscChar;

/* ================= ISR ================= */

void IRAM_ATTR pulseISR() {
  unsigned long now = micros();
  if (now - lastPulseTime < DEBOUNCE_US) return;

  pulseInterval = now - lastPulseTime;
  lastPulseTime = now;
  newPulse = true;
}

/* ================= SETUP ================= */

void setup() {
  Serial.begin(115200);

  pinMode(SENSOR_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(SENSOR_PIN), pulseISR, FALLING);

  BLEDevice::init("ESP32 RSC Speed");
  BLEServer* server = BLEDevice::createServer();

  BLEService* rscService = server->createService(RSC_SERVICE_UUID);

  rscChar = rscService->createCharacteristic(
    RSC_MEAS_UUID,
    BLECharacteristic::PROPERTY_NOTIFY
  );

  rscChar->addDescriptor(new BLE2902());

  rscService->start();

  BLEAdvertising* advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(RSC_SERVICE_UUID);
  advertising->setScanResponse(true);
  advertising->start();

  Serial.println("ESP32 RSC Sensor READY (Garmin compatible)");
}

/* ================= LOOP ================= */

void loop() {
  unsigned long now = micros();

  if (newPulse) {
    noInterrupts();
    unsigned long interval = pulseInterval;
    newPulse = false;
    interrupts();

    if (interval > 0) {
      float speed_mps = ROLLER_DISTANCE / (interval / 1000000.0);
      speed_kmh = speed_mps * 3.6;

      speed_kmh_filtered =
        FILTER_ALPHA * speed_kmh +
        (1.0 - FILTER_ALPHA) * speed_kmh_filtered;
    }
  }

  // Tapis fermo
  if (now - lastPulseTime > SPEED_TIMEOUT_US) {
    speed_kmh = 0.0;
    speed_kmh_filtered *= 0.9;
    if (speed_kmh_filtered < 0.05) speed_kmh_filtered = 0.0;
  }

  /* INVIO BLE (5–10 Hz è perfetto per Garmin) */
  static unsigned long lastNotify = 0;
  if (millis() - lastNotify > 120) {
    lastNotify = millis();

    uint8_t data[4];

    // Flags:
    // bit0 = velocità presente
    // bit1 = cadenza presente
    data[0] = 0x03;

    // Velocità (m/s * 256)
    uint16_t speed_rsc =
      (uint16_t)((speed_kmh_filtered / 3.6) * 256.0);

    data[1] = speed_rsc & 0xFF;
    data[2] = speed_rsc >> 8;

    // Cadenza (0 = non usata)
    data[3] = 0x00;

    rscChar->setValue(data, sizeof(data));
    rscChar->notify();
  }

  /* DEBUG */
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 500) {
    lastPrint = millis();
    Serial.print("Velocità Garmin: ");
    Serial.print(speed_kmh_filtered, 2);
    Serial.println(" km/h");
  }
}
