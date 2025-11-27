// improved_firmware.ino
#include <AccelStepper.h>
#include <MultiStepper.h>

// Motor Pins (RAMPS 1.4)
AccelStepper s1(AccelStepper::DRIVER, A0, A1); // X
AccelStepper s2(AccelStepper::DRIVER, A6, A7); // Y
AccelStepper s3(AccelStepper::DRIVER, 46, 48); // Z
AccelStepper s4(AccelStepper::DRIVER, 26, 28); // E0
AccelStepper s5(AccelStepper::DRIVER, 36, 34); // E1

// MultiStepper instance for coordinated motion
MultiStepper steppers;

// Enable Pins
const int enablePins[] = {38, A2, A8, 24, 30};

// Buffer for smooth streaming
const int BUF_SIZE = 32; // Store up to 32 moves
long moveBuffer[BUF_SIZE][5];
float speedBuffer[BUF_SIZE];
volatile int head = 0;
volatile int tail = 0;

void setup() {
  Serial.begin(115200);

  // Configure Enable Pins
  for(int i=0; i<5; i++) {
    pinMode(enablePins[i], OUTPUT);
    digitalWrite(enablePins[i], LOW); // Enable motors
  }

  // Configure Max Speeds (Steps per second) - ADJUST THESE AS NEEDED
  // Higher values = faster robot. Start conservative.
  float maxSpeed = 4000.0; 
  s1.setMaxSpeed(maxSpeed); s2.setMaxSpeed(maxSpeed); s3.setMaxSpeed(maxSpeed);
  s4.setMaxSpeed(maxSpeed); s5.setMaxSpeed(maxSpeed);

  // Add to MultiStepper
  steppers.addStepper(s1); steppers.addStepper(s2); steppers.addStepper(s3);
  steppers.addStepper(s4); steppers.addStepper(s5);

  Serial.println("READY");
}

void loop() {
  // 1. KEEP MOTORS MOVING
  // run() returns true if the robot is still moving to the current target
  bool isMoving = steppers.run();

  // 2. LOAD NEXT MOVE IF IDLE & BUFFER HAS DATA
  if (!isMoving && head != tail) {
    // Set the speed for this segment (simulating Feedrate)
    float segmentSpeed = speedBuffer[tail];
    
    // Update max speeds for this specific move to control feedrate
    // Note: MultiStepper obeys the lowest max speed limit relative to distance
    s1.setMaxSpeed(segmentSpeed); s2.setMaxSpeed(segmentSpeed); 
    s3.setMaxSpeed(segmentSpeed); s4.setMaxSpeed(segmentSpeed); 
    s5.setMaxSpeed(segmentSpeed);
    
    // Set target
    steppers.moveTo(moveBuffer[tail]);
    
    // Advance tail
    tail = (tail + 1) % BUF_SIZE;
  }

  // 3. READ SERIAL COMMANDS
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    
    if (cmd.startsWith("G1")) {
      int nextHead = (head + 1) % BUF_SIZE;
      
      // Only accept if buffer is not full
      if (nextHead != tail) {
        long newTarget[5];
        newTarget[0] = parseVal(cmd, 'X');
        newTarget[1] = parseVal(cmd, 'Y');
        newTarget[2] = parseVal(cmd, 'Z');
        newTarget[3] = parseVal(cmd, 'A');
        newTarget[4] = parseVal(cmd, 'B');
        
        float feedrate = parseVal(cmd, 'F');
        if (feedrate <= 0) feedrate = 2000; // Default
        
        // Store in buffer
        for(int i=0; i<5; i++) moveBuffer[head][i] = newTarget[i];
        speedBuffer[head] = feedrate;
        
        head = nextHead;
        Serial.println("OK"); // Signal MATLAB we are ready for next
      } 
    } 
    else if (cmd.startsWith("G28")) {
       // Handle homing/reset here if needed
       // For now, just zero positions
       long zeros[] = {0,0,0,0,0};
       for(int i=0; i<5; i++) { 
         moveBuffer[head][i] = 0; 
       }
       s1.setCurrentPosition(0); s2.setCurrentPosition(0); s3.setCurrentPosition(0);
       s4.setCurrentPosition(0); s5.setCurrentPosition(0);
       head = tail; // Clear buffer
       Serial.println("OK");
    }
  }
}

long parseVal(String cmd, char key) {
  int idx = cmd.indexOf(key);
  if (idx == -1) return 0; // Or keep previous position? 0 for now.
  return cmd.substring(idx + 1).toInt();
}
