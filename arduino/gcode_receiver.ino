// improved_firmware.ino
#include <AccelStepper.h>
#include <MultiStepper.h>

// Configuration
const int BUF_SIZE = 64;  // Increased buffer size (Mega has 8KB RAM)
const long BAUDRATE = 115200;

// Motor Pins (RAMPS 1.4)
// X Axis (Joint 1)
AccelStepper s1(AccelStepper::DRIVER, 54, 55); 
// Y Axis (Joint 2)
AccelStepper s2(AccelStepper::DRIVER, 60, 61); 
// Z Axis (Joint 3)
AccelStepper s3(AccelStepper::DRIVER, 46, 48); 
// E0 Axis (Joint 4)
AccelStepper s4(AccelStepper::DRIVER, 26, 28); 
// E1 Axis (Joint 5)
AccelStepper s5(AccelStepper::DRIVER, 36, 34); 

// MultiStepper instance for coordinated motion
MultiStepper steppers;

// Enable Pins
const int enablePins[] = {38, 56, 62, 24, 30}; // X, Y, Z, E0, E1 Enable pins on RAMPS 1.4

// Buffer for smooth streaming
long moveBuffer[BUF_SIZE][5];
float speedBuffer[BUF_SIZE];
volatile int head = 0;
volatile int tail = 0;

void setup() {
  Serial.begin(BAUDRATE);

  // Configure Enable Pins
  // X=38, Y=A2(56), Z=A8(62), E0=24, E1=30
  int enPins[] = {38, 56, 62, 24, 30}; 
  for(int i=0; i<5; i++) {
    pinMode(enPins[i], OUTPUT);
    digitalWrite(enPins[i], LOW); // Enable motors (Low active)
  }

  // Configure Max Speeds (Steps per second)
  // Higher values = faster robot.
  float maxSpeed = 4000.0; 
  s1.setMaxSpeed(maxSpeed); 
  s2.setMaxSpeed(maxSpeed); 
  s3.setMaxSpeed(maxSpeed);
  s4.setMaxSpeed(maxSpeed); 
  s5.setMaxSpeed(maxSpeed);

  // Add to MultiStepper
  steppers.addStepper(s1); 
  steppers.addStepper(s2); 
  steppers.addStepper(s3);
  steppers.addStepper(s4); 
  steppers.addStepper(s5);

  Serial.println("READY");
}

void loop() {
  // 1. KEEP MOTORS MOVING
  // run() returns true if the robot is still moving to the current target
  bool isMoving = steppers.run();

  // 2. LOAD NEXT MOVE IF IDLE & BUFFER HAS DATA
  if (!isMoving && head != tail) {
    // We finished the previous move (or are just starting)
    // Signal that a buffer slot has been freed
    Serial.println("DONE");

    // Set the speed for this segment
    float segmentSpeed = speedBuffer[tail];
    
    // Update max speeds for this specific move
    // MultiStepper calculates the move duration based on the slowest axis relative to its max speed
    // setting all to segmentSpeed ensures the fastest part of the move respects this limit.
    // However, if we want the move to take exactly 'dt', we rely on the speed calculation from Python.
    s1.setMaxSpeed(segmentSpeed); 
    s2.setMaxSpeed(segmentSpeed); 
    s3.setMaxSpeed(segmentSpeed); 
    s4.setMaxSpeed(segmentSpeed); 
    s5.setMaxSpeed(segmentSpeed);
    
    // Set target
    steppers.moveTo(moveBuffer[tail]);
    
    // Advance tail
    tail = (tail + 1) % BUF_SIZE;
  }

  // 3. READ SERIAL COMMANDS
  if (Serial.available()) {
    // Only read if we have space in buffer
    // To calculate space:
    int nextHead = (head + 1) % BUF_SIZE;
    if (nextHead != tail) {
      String cmd = Serial.readStringUntil('\n');
      cmd.trim();
      
      if (cmd.startsWith("G1")) {
        long newTarget[5];
        // Parse X, Y, Z, A, B (Joints 1-5)
        newTarget[0] = parseVal(cmd, 'X');
        newTarget[1] = parseVal(cmd, 'Y');
        newTarget[2] = parseVal(cmd, 'Z');
        newTarget[3] = parseVal(cmd, 'A');
        newTarget[4] = parseVal(cmd, 'B');
        
        float feedrate = parseValFloat(cmd, 'F');
        if (feedrate <= 0) feedrate = 1000; // Default fallback
        
        // Store in buffer
        for(int i=0; i<5; i++) moveBuffer[head][i] = newTarget[i];
        speedBuffer[head] = feedrate;
        
        head = nextHead;
        Serial.println("OK"); // Acknowledge receipt
      } 
      else if (cmd.startsWith("G28")) {
         // Homing / Zeroing
         long zeros[] = {0,0,0,0,0};
         s1.setCurrentPosition(0); 
         s2.setCurrentPosition(0); 
         s3.setCurrentPosition(0);
         s4.setCurrentPosition(0); 
         s5.setCurrentPosition(0);
         
         // Clear buffer
         head = tail; 
         
         Serial.println("OK");
      }
    }
    // If buffer is full, we DO NOT read from Serial. 
    // This uses the hardware serial buffer effectively as backpressure.
    // However, the hardware buffer is small (64 bytes).
    // The Python script should manage flow control to avoid overflow.
  }
}

long parseVal(String cmd, char key) {
  int idx = cmd.indexOf(key);
  if (idx == -1) return 0; 
  // Look for the end of the number
  int endIdx = idx + 1;
  while(endIdx < cmd.length() && (isDigit(cmd[endIdx]) || cmd[endIdx] == '-' || cmd[endIdx] == '.')) {
    endIdx++;
  }
  String sub = cmd.substring(idx + 1, endIdx);
  return sub.toInt();
}

float parseValFloat(String cmd, char key) {
  int idx = cmd.indexOf(key);
  if (idx == -1) return 0.0;
  int endIdx = idx + 1;
  while(endIdx < cmd.length() && (isDigit(cmd[endIdx]) || cmd[endIdx] == '-' || cmd[endIdx] == '.')) {
    endIdx++;
  }
  String sub = cmd.substring(idx + 1, endIdx);
  return sub.toFloat();
}
