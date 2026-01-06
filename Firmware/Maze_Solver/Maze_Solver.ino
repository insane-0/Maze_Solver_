#include <Encoder.h>
#include <Wire.h>
#include <VL53L1X.h>
#include <Arduino.h>

volatile bool stopNow = false;   // emergency stop flag
volatile int distFrontISR = 500; // latest front distance from ISR
const int SAFE_DIST = 40;       // stop threshold in mm
int error =0;

// -------------------- Pins --------------------
#define LED_PIN 10   // LED on pin 10

// -------------------- Wall sensor flags --------------------
volatile bool flagLeft = false;
volatile bool flagRight = false;
volatile bool flagFront = false;

// -------------------- Distance readings --------------------
int distLeft = 500;   // initialize to a safe value
int distRight = 500;
int distFront = 500;

// -------------------- Wall-following PID --------------------
float integral_wall = 0;
int lastError_wall = 0;

// PID constants for wall-following
const float Kp_wall = 0.5;
const float Ki_wall = 0.3;
const float Kd_wall = 2.0;

// Desired distance from the wall in mm (adjust as per your robot)
const int DESIRED_DIST = 70;  // 20 cm


// -------------------- Sensors --------------------
VL53L1X sensorFront;
VL53L1X sensorRight;
VL53L1X sensorLeft;

// Flags from interrupts
volatile bool frontReady = false;
volatile bool rightReady = false;
volatile bool leftReady  = false;

// Error flag
bool initError = false;

// -------------------- ISRs --------------------
void frontISR() {
  distFrontISR = sensorFront.read();
  frontReady = true;               // <--- add this
  if (distFrontISR < SAFE_DIST) {
    stopNow = true;
  }
}


void rightISR() { rightReady = true; }
void leftISR()  { leftReady  = true; }
// -------------------------
// Motor Driver Pins
// -------------------------
#define AIN1 15
#define AIN2 16
#define BIN1 13
#define BIN2 14
#define PWMA 4
#define PWMB 3

// Interrupt pins
#define INT_FRONT 1
#define INT_RIGHT 12
#define INT_LEFT 7
#define FORWARD_SPEED 50
// XSHUT pins
#define XSHUT_FRONT 0
#define XSHUT_RIGHT 11
#define XSHUT_LEFT 6

// -------------------------
// Encoder Setup
// -------------------------
Encoder leftEncoder(21, 20);
Encoder rightEncoder(23, 22);

// -------------------------
// Movement Config
// -------------------------
#define MIN_SPEED 40 
#define MAX_SPEED 60

int df = 0;
int dr = 0;
int dl =0;


long mmToTicks(int mm) {
  return mm * 10.13; // Adjust experimentally
}

// experimentally tuned conversion
long angleToTicks(int deg) {
  return deg * 11.048; // adjust for your robot
}
const long RAMP_DISTANCE = 150; // pulses for ramp in/out

// -------------------------
// Motor Control
// -------------------------
void setMotor(int in1, int in2, int pwm, int speed) {
  if (speed > 0) {          // Forward
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } 
  else if (speed < 0) {     // Reverse
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } 
  else {                    // Coast
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
  analogWrite(pwm, abs(speed));
}

void stopMotors() {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
}

void resetEncoders() {
  leftEncoder.write(0);
  rightEncoder.write(0);
}

//Move forward
// Sensor-based PID move forward
void moveForwardPID(int cell_mm) {
    resetEncoders();  

    float integral = 0;
    int prev_error = 0;
    stopNow = false;   // reset stop flag before starting

    while (true) {
        // 🚨 Emergency stop check
        if (stopNow) {
            stopMotors();
            return;   // exit immediately, back to loop()
        }

        // --- Compute error for centering ---
        // Update sensor values each loop
        if(leftReady){ dl = sensorLeft.read(); leftReady = false; }
        if(rightReady){ dr = sensorRight.read(); rightReady = false; }
        if(frontReady){ df = sensorFront.read(); frontReady = false; }
        if(dl<100 && dr < 100){
          error = (dl - dr);   // don’t multiply by 500 yet
        }
        else{
          if(dl<100 && dr > 100){
             error = (dl - 57.5)*1.25;
          }
          else{
            error = -(dr - 57.5)*1.25;
          }
        }

        // --- PID calculations ---
        integral += error;
        integral = constrain(integral, -500, 500);
        int derivative = error - prev_error;
        int correction = Kp_wall * error + Kd_wall * (error - prev_error);
        prev_error = error;

        // --- Base forward speed ---
        int l_speed = FORWARD_SPEED;
        int r_speed = FORWARD_SPEED;

        // --- Adjust motor speeds based on walls ---
        if (dl < 100 && dr < 100) {
            l_speed += correction;
            r_speed -= correction;
        } else if (dl < 100) {
            r_speed -= correction;
        } else if (dr < 100) {
            l_speed += correction;
        }

        // --- Constrain motor speeds ---
        l_speed = constrain(l_speed, 0, MAX_SPEED);
        r_speed = constrain(r_speed, 0, MAX_SPEED);

        // --- Set motor speeds ---
        setMotor(AIN1, AIN2, PWMA, l_speed);
        setMotor(BIN1, BIN2, PWMB, r_speed);

        // --- Stop condition based on distance traveled ---
        long leftTicks  = abs(leftEncoder.read());
        long rightTicks = abs(rightEncoder.read());
        long traveled = (leftTicks + rightTicks) / 2;

        if (traveled >= mmToTicks(cell_mm)) break;
    }

    
}

// -------------------------
// Rotate Function
// -------------------------
void rotate(int direction, int angle) {
  resetEncoders();
  long ticks = angleToTicks(angle);

  while (true) {
    long leftTicks = abs(leftEncoder.read());
    long rightTicks = abs(rightEncoder.read());
    long traveled = max(leftTicks, rightTicks);
    long remaining = ticks - traveled;

    if (remaining <= 0) break;

    int baseSpeed;
    if (traveled < RAMP_DISTANCE)
      baseSpeed = map(traveled, 0, RAMP_DISTANCE, MIN_SPEED, MAX_SPEED);
    else if (remaining < RAMP_DISTANCE)
      baseSpeed = map(remaining, 0, RAMP_DISTANCE, MIN_SPEED, MAX_SPEED);
    else
      baseSpeed = MAX_SPEED;

    setMotor(AIN1, AIN2, PWMA, direction > 0 ? baseSpeed : -baseSpeed);
    setMotor(BIN1, BIN2, PWMB, direction > 0 ? -baseSpeed : baseSpeed);
  }
  stopMotors();
}

// -------------------------
// Setup & Loop
// -------------------------
void setup() {
  Serial.begin(115200);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Reset sensors
  pinMode(XSHUT_FRONT, OUTPUT);
  pinMode(XSHUT_RIGHT, OUTPUT);
  pinMode(XSHUT_LEFT, OUTPUT);

  digitalWrite(XSHUT_FRONT, LOW);
  digitalWrite(XSHUT_RIGHT, LOW);
  digitalWrite(XSHUT_LEFT, LOW);
  delay(10);

  Wire.begin();
  Wire.setClock(400000);

  // --- Front ---
  digitalWrite(XSHUT_FRONT, HIGH);
  delay(10);
  sensorFront.setBus(&Wire);
  if (!sensorFront.init()) {
    Serial.println("Front sensor not found!");
    initError = true;
  } else {
    sensorFront.setAddress(0x2A);
    Serial.println("Front sensor ready at 0x2A");
  }

  // --- Right ---AC
  digitalWrite(XSHUT_RIGHT, HIGH);
  delay(10);
  sensorRight.setBus(&Wire);
  if (!sensorRight.init()) {
    Serial.println("Right sensor not found!");
    initError = true;
  } else {
    sensorRight.setAddress(0x2B);
    Serial.println("Right sensor ready at 0x2B");
  }

  // --- Left ---
  digitalWrite(XSHUT_LEFT, HIGH);
  delay(10);
  sensorLeft.setBus(&Wire);
  if (!sensorLeft.init()) {
    Serial.println("Left sensor not found!");
    initError = true;
  } else {
    sensorLeft.setAddress(0x2C);
    Serial.println("Left sensor ready at 0x2C");
  }

  // LED indicates error state
  if (initError) {
    digitalWrite(LED_PIN, HIGH); // turn ON (error)
  } else {
    digitalWrite(LED_PIN, LOW);  // turn OFF (all good)
  }

  if (initError) return; // don't continue if any failed

  // Attach interrupts
  pinMode(INT_FRONT, INPUT);
  pinMode(INT_RIGHT, INPUT);
  pinMode(INT_LEFT, INPUT);
  attachInterrupt(digitalPinToInterrupt(INT_FRONT), frontISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(INT_RIGHT), rightISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(INT_LEFT), leftISR, FALLING);

  // Start continuous ranging
  sensorFront.startContinuous(20);
  sensorRight.startContinuous(20);
  sensorLeft.startContinuous(20);

  delay(1000);
}

void loop() {

  if (initError) return;

  // Read sensors
   df = sensorFront.read();
   dr = sensorRight.read();
   dl = sensorLeft.read();

  if(leftReady){
    dl=sensorLeft.read();
    leftReady=false;
  }
  if(rightReady){
    dr=sensorRight.read();
    rightReady=false;
  }
  if(frontReady){
    df=sensorFront.read();
    frontReady=false;
  }

  // Only react if sensor detects something closer than threshold
  const int THRESHOLD = 150;
  Serial.println(df);
  Serial.println(dl);
  Serial.println(dr);

  // If path forward is clear, go straight
  if (df > THRESHOLD ) {
      moveForwardPID(250);  // forward 200 mm
  }
  else if (df < THRESHOLD && dl < THRESHOLD && dr < THRESHOLD){
    rotate(1,190);
    delay(500);
  }
  else if (df < THRESHOLD) {
      // front blocked, rotate based on open side
      if (dl > dr) rotate(1, 90);     // turn left
      else rotate(-1, 90);            // turn right
      delay(200);
      moveForwardPID(250);
  }
}