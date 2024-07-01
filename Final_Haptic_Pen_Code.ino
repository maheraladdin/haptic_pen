// include modules
#include <Wire.h>
#include <AccelStepper.h>
#include <SoftwareSerial.h>
#include <string.h>
#include <stdlib.h>

// Define the sensitivity scale factors
const float ACCEL_SENSITIVITY = 16384.0; // Assuming 16384 LSB/g for the accelerometer
const float GYRO_SENSITIVITY = 131.0;    // Assuming 131 LSB/(°/s) for the gyroscope
const float GRAVITY = 9.81;              // Gravity constant for conversion to m/s^2
const float TEMP_SENSITIVITY = 340.0;    // 340 LSB/°C
const float TEMP_OFFSET = 36.53;         // Offset in °C

// define pins
#define X_DIR_PIN       2
#define X_STEP_PIN      3
#define buttonPin       12
#define vibrationPin    4
#define stepperENPin    8

// define stepper motor config variables
#define STEPPER_MAX_SPEED       200
#define STEPPER_ACCELERATION    100

// Bluetooth setup
SoftwareSerial bluetooth(9, 10);

// Stepper motor setup
AccelStepper stepper1(1, X_STEP_PIN, X_DIR_PIN);

// Global variables
char bluetoothInput[50];
int feedAmount, index = 0;
bool vibrationState = false;
bool simulationState = false;
char receivedChar;

const int MPU_ADDR = 0x68; // I2C address of the MPU-6050

int16_t ax, ay, az; // variables for accelerometer raw data
int16_t gx, gy, gz; // variables for gyro raw data
int16_t tem; // variables for temperature data
float accelX, accelY, accelZ;
float gyroX, gyroY, gyroZ;
float temperature;


float dx = 0.0, dy = 0.0, dz = 0.0;  // Displacement components
float lastAx = 0.0, lastAy = 0.0, lastAz = 0.0;  // Last acceleration values

unsigned long lastUpdateTime = 0; // time of last update for calculations

void setup() {
  Serial.begin(9600);
  bluetooth.begin(9600);
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0);    // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(vibrationPin, OUTPUT);
  pinMode(X_DIR_PIN, OUTPUT);
  pinMode(X_STEP_PIN, OUTPUT);
  pinMode(stepperENPin, OUTPUT);

  stepper1.setMaxSpeed(STEPPER_MAX_SPEED);
  stepper1.setAcceleration(STEPPER_ACCELERATION);


  digitalWrite(stepperENPin, HIGH);

  Serial.println("HELLO WORLD!");
}

void loop() {
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastUpdateTime) / 1000.0; // convert to seconds

  changeStates();
  ActionOne();
  simulatePenPosition(deltaTime);
  
  lastUpdateTime = currentTime;
}

void simulatePenPosition(float deltaTime) {
  if (simulationState && !bluetooth.available()) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 6 * 2, true); // request 12 registers (only accelerometer and temperature)

    ax = Wire.read() << 8 | Wire.read();
    ay = Wire.read() << 8 | Wire.read();
    az = Wire.read() << 8 | Wire.read();
    tem = Wire.read() << 8 | Wire.read(); // Read temperature (discard for displacement calculation)
    gx = Wire.read() << 8 | Wire.read();
    gy = Wire.read() << 8 | Wire.read();
    gz = Wire.read() << 8 | Wire.read();

    // Convert raw data to meaningful units
    accelX = (float)ax / ACCEL_SENSITIVITY * GRAVITY; // m/s^2
    accelY = (float)ay / ACCEL_SENSITIVITY * GRAVITY; // m/s^2
    accelZ = (float)az / ACCEL_SENSITIVITY * GRAVITY; // m/s^2

    gyroX = (float)gx / GYRO_SENSITIVITY; // °/s
    gyroY = (float)gy / GYRO_SENSITIVITY; // °/s
    gyroZ = (float)gz / GYRO_SENSITIVITY; // °/s

    temperature = (float)tem / TEMP_SENSITIVITY + TEMP_OFFSET;

    // Calculate displacement (distance) using trapezoidal integration method
    dx += integrateAcceleration(accelX, lastAx, deltaTime);
    dy += integrateAcceleration(accelY, lastAy, deltaTime);
    dz += integrateAcceleration(accelZ, lastAz, deltaTime);

    // Update last acceleration values
    lastAx = accelX;
    lastAy = accelY;
    lastAz = accelZ;

    bluetooth.println(String(dx) + "," + String(dy) + "," + String(dz) + ":" + String(gyroX) + "," + String(gyroY) + "," + String(gyroZ) + ":" + String(temperature));
    Serial.println(String(dx) + "," + String(dy) + "," + String(dz) + ":" + String(gyroX) + "," + String(gyroY) + "," + String(gyroZ) + ":" + String(temperature));
  }
}

// Function to integrate acceleration to displacement
float integrateAcceleration(float acc, float lastAcc, float dt) {
  // Trapezoidal integration: (current acceleration + last acceleration) * dt / 2
  return 0.5 * (acc + lastAcc) * dt * dt;
}

void ActionOne() {
  if (digitalRead(buttonPin) == LOW && !bluetooth.available()) {
    static unsigned long lastDebounceTime = 0;
    unsigned long debounceDelay = 100;

    if (millis() - lastDebounceTime > debounceDelay) {
      bluetooth.println("ACTION_ONE");
      Serial.println("ACTION_ONE");
      if(vibrationState) {
          digitalWrite(vibrationPin, HIGH);
      }
      lastDebounceTime = millis();
    }
  } else {
    digitalWrite(vibrationPin, LOW);
  }
}

void changeSimulationStatus() {
  if (strncmp(bluetoothInput, "SIMULATE", 8) == 0) {
    simulationState = true;
    memset(bluetoothInput, 0, sizeof(bluetoothInput));
    index = 0;
  } else if (strncmp(bluetoothInput, "STOP_SIMULATE", 13) == 0) {
    simulationState = false;
    memset(bluetoothInput, 0, sizeof(bluetoothInput));
    index = 0;
  }
}

void changeVibrationStatus() {
  if (strncmp(bluetoothInput, "VIBRATION_ON", 12) == 0) {
    vibrationState = true;
    memset(bluetoothInput, 0, sizeof(bluetoothInput));
    index = 0;
  } else if (strncmp(bluetoothInput, "VIBRATION_OFF", 13) == 0) {
    vibrationState = false;
    memset(bluetoothInput, 0, sizeof(bluetoothInput));
    index = 0;
  }
}

void resist() {
  digitalWrite(stepperENPin, LOW);

  if (strncmp(bluetoothInput, "RESIST_X_", 9) == 0) {
    feedAmount = atoi(bluetoothInput + 9);
    signed long currentX = stepper1.currentPosition();
    stepper1.runToNewPosition(currentX + feedAmount);
    memset(bluetoothInput, 0, sizeof(bluetoothInput));
    index = 0;
  }

  digitalWrite(stepperENPin, HIGH);
}

void changeStates() {
  while (Serial.available()) {
    receivedChar = Serial.read();
    if(receivedChar == '\n') continue;
    bluetoothInput[index++] = receivedChar;
    if (receivedChar == '$') {
      bluetoothInput[index - 1] = '\0'; // terminate the string properly
      changeSimulationStatus();
      changeVibrationStatus();
      resist();
      index = 0; // Reset the index after processing the command
    }
  }

  while (bluetooth.available()) {
    receivedChar = bluetooth.read();
    if(receivedChar == '\n') continue;
    bluetoothInput[index++] = receivedChar;

    if (receivedChar == '$') {
      bluetoothInput[index - 1] = '\0'; // terminate the string properly
      changeSimulationStatus();
      changeVibrationStatus();
      resist();
      index = 0; // Reset the index after processing the command
    }
  }
}
