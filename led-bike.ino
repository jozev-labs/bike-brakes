#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <avr/wdt.h>  // For watchdog timer reset

// LED pin definitions
const int ledPins1[] = {2, 3, 4};  // LEDs D2-D4 (left turn)
const int ledPins2[] = {5, 6, 7, 8};  // LEDs D5-D8 (red, braking)
const int ledPins3[] = {9, 10, 11};  // LEDs D9-D11 (right turn)

// Timing constants (ms)
const int blinkInterval = 100;  // Blink rate for turning and braking LEDs
const int brakingHoldTime = 1000;  // Minimum time braking LEDs blink
unsigned long lastUpdate = 0;
unsigned long lastBrakingTime = 0;  // Timestamp of last braking event
bool blinkState = false;
bool brakingActive = false;

// Lockup detection
const int lockupCheckInterval = 100;  // Check every 100ms
const int maxIdenticalReadings = 10;  // Number of identical readings to detect lockup
float lastAccelX = 0, lastAccelY = 0, lastAccelZ = 0;
int identicalCount = 0;
unsigned long lastLockupCheck = 0;

// Optional: MPU power pin (uncomment if using transistor)
// const int mpuPowerPin = 12;  // Pin to control MPU-6050 VCC

// MPU6050 object
Adafruit_MPU6050 mpu;

// Thresholds (in g)
const float turnThreshold = 0.5;  // Y-axis acceleration for turning
const float brakingThreshold = 0.7;  // X-axis deceleration for braking
const float noMovementThreshold = 0.3;  // Threshold for no movement (X and Y)
const float lockupTolerance = 0.01;  // Tolerance for identical readings (g)

void setup() {
  // Initialize LED pins
  for (int i = 0; i < 3; i++) pinMode(ledPins1[i], OUTPUT);
  for (int i = 0; i < 4; i++) pinMode(ledPins2[i], OUTPUT);
  for (int i = 0; i < 3; i++) pinMode(ledPins3[i], OUTPUT);

  // Turn off turning LEDs, turn on red LEDs initially
  for (int i = 0; i < 3; i++) digitalWrite(ledPins1[i], LOW);
  for (int i = 0; i < 4; i++) digitalWrite(ledPins2[i], HIGH);  // Red LEDs ON
  for (int i = 0; i < 3; i++) digitalWrite(ledPins3[i], LOW);

  // Initialize Serial for debugging
  Serial.begin(9600);

  // Optional: Initialize MPU power pin (uncomment if using transistor)
  // pinMode(mpuPowerPin, OUTPUT);
  // digitalWrite(mpuPowerPin, HIGH);  // Turn on MPU (HIGH for PNP transistor)

  // Initialize MPU6050
  if (!initializeMPU()) {
    Serial.println("Initial MPU6050 setup failed!");
    triggerNanoReset();  // Reset Nano if MPU fails
  }
}

bool initializeMPU() {
  // Optional: Power cycle MPU (uncomment if using transistor)
  // digitalWrite(mpuPowerPin, LOW);  // Turn off MPU
  // delay(100);
  // digitalWrite(mpuPowerPin, HIGH);  // Turn on MPU
  // delay(100);

  // Initialize I2C and MPU6050
  Wire.begin();  // Reset I2C bus
  if (mpu.begin()) {
    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);  // ±2g range
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);  // Reduce noise
    Serial.println("MPU6050 initialized");
    identicalCount = 0;  // Reset lockup counter
    return true;
  }
  return false;
}

void triggerNanoReset() {
  Serial.println("Triggering Nano reset...");
  wdt_enable(WDTO_15MS);  // Enable watchdog timer with 15ms timeout
  while (1);  // Wait for reset
}

void loop() {
  // Get accelerometer data
  sensors_event_t accel, gyro, temp;
  bool dataValid = mpu.getEvent(&accel, &gyro, &temp);

  // Convert accelerations to g (m/s² ÷ 9.81)
  float accelX = dataValid ? accel.acceleration.x / 9.81 : 0;
  float accelY = dataValid ? accel.acceleration.y / 9.81 : 0;
  float accelZ = dataValid ? (accel.acceleration.z - 9.81) / 9.81 : 0;

  // Check for MPU lockup (identical readings)
  unsigned long currentMillis = millis();
  if (currentMillis - lastLockupCheck >= lockupCheckInterval) {
    if (dataValid && abs(accelX - lastAccelX) < lockupTolerance &&
        abs(accelY - lastAccelY) < lockupTolerance &&
        abs(accelZ - lastAccelZ) < lockupTolerance) {
      identicalCount++;
      if (identicalCount >= maxIdenticalReadings) {
        Serial.println("MPU6050 lockup detected, attempting reinitialization...");
        if (!initializeMPU()) {
          Serial.println("Reinitialization failed, resetting Nano...");
          triggerNanoReset();
        }
        identicalCount = 0;  // Reset counter after successful reinitialization
      }
    } else {
      identicalCount = 0;  // Reset counter if readings change
    }
    lastAccelX = accelX;
    lastAccelY = accelY;
    lastAccelZ = accelZ;
    lastLockupCheck = currentMillis;
  }

  // Check for braking (sudden negative X acceleration)
  static float prevAccelX = 0;
  float accelXChange = prevAccelX - accelX;  // Negative acceleration
  if (accelXChange > brakingThreshold) {
    brakingActive = true;
    lastBrakingTime = currentMillis;  // Start hold timer
  }

  // Keep braking LEDs blinking for at least brakingHoldTime
  if (brakingActive && currentMillis - lastBrakingTime > brakingHoldTime) {
    brakingActive = false;  // End braking period after 1 second
  }

  // Check for turning (Y-axis acceleration)
  bool isLeftTurn = accelY > turnThreshold;   // Positive Y = left turn
  bool isRightTurn = accelY < -turnThreshold; // Negative Y = right turn

  // Check for no movement (X and Y near zero, ignore Z)
  bool noMovement = abs(accelX) < noMovementThreshold && abs(accelY) < noMovementThreshold;

  // Update LEDs
  if (currentMillis - lastUpdate >= blinkInterval) {
    blinkState = !blinkState;  // Toggle blink state

    // LED Set 2 (D5-D8, red) - Always ON, blink during braking
    for (int i = 0; i < 4; i++) {
      digitalWrite(ledPins2[i], brakingActive ? blinkState : HIGH);
    }

    // LED Sets 1 and 3 (D2-D4 for left, D9-D11 for right) - Blink for turning, OFF if braking or no movement
    if (!brakingActive && !noMovement) {
      for (int i = 0; i < 3; i++) {
        digitalWrite(ledPins1[i], isLeftTurn ? blinkState : LOW);
        digitalWrite(ledPins3[i], isRightTurn ? blinkState : LOW);
      }
    } else {
      for (int i = 0; i < 3; i++) {
        digitalWrite(ledPins1[i], LOW);
        digitalWrite(ledPins3[i], LOW);
      }
    }

    prevAccelX = accelX;  // Store for next iteration
    lastUpdate = currentMillis;

    // Debug output
    Serial.print("Accel (g): X=");
    Serial.print(accelX);
    Serial.print(", Y=");
    Serial.print(accelY);
    Serial.print(", Z=");
    Serial.print(accelZ);
    Serial.print(", Braking=");
    Serial.print(brakingActive ? "YES" : "NO");
    Serial.print(", LeftTurn=");
    Serial.print(isLeftTurn ? "YES" : "NO");
    Serial.print(", RightTurn=");
    Serial.println(isRightTurn ? "YES" : "NO");
  }
}
