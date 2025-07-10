#include <PestoLink-Receive.h>
#include <Alfredo_NoU3.h>
#include <constants.h>

// Motors                         //- 654321 +//
NoU_Motor frontLeftMotor(1);      //5        4//
NoU_Motor frontRightMotor(8);     //6        3//
NoU_Motor backLeftMotor(4);       //7        2//
NoU_Motor backRightMotor(5);      //8        1//
NoU_Motor intakeMotor(3);         //+  USBC  -//

// Servos
NoU_Servo stageI(1);
NoU_Servo stageII(2);
NoU_Servo clawServo(3);

// Global State
float intakeT = 0;
float angleI = 40.0; 
float angleII = 140.0;
float clawAngle = grab;
int setpoint = 0;
int oldSetpoint = -1;

float angular_scale;
float measured_angle;

// Drivetrain

void setup() {
  PestoLink.begin("Team 74");
  Serial.begin(115200);

  NoU3.begin();
  NoU3.setServiceLight(LIGHT_ENABLED);

  measured_angle = 31.416; // Tune this by spinning 5 full times
  angular_scale = (5.0 * 2.0 * PI) / measured_angle;
  NoU3.calibrateIMUs(); // Takes exactly 1 second
}

void loop() {
  chassis(); 
  arm();
  claw();
  intake();

  intakeMotor.set(intakeT);
  stageI.write(angleI);
  stageII.write(angleII);
  clawServo.write(clawAngle);
}

float tuneMotorPower(float input, float deadband, float minPower, float maxPower, float exponent = 1.0) {
  if (fabs(input) < deadband) return 0;

  float sign = (input >= 0) ? 1.0 : -1.0;
  float curved = pow(fabs(input), exponent);            // Apply input shaping
  float scaled = curved * maxPower;                     // Scale to max power

  if (scaled < minPower) return 0;                      // Below motor stall? Zero it.
  return sign * scaled;                                 // Return shaped output
}


void chassis() {
  if (PestoLink.update()) {
    float y = -PestoLink.getAxis(1);
    float x = PestoLink.getAxis(0);
    float rx = PestoLink.getAxis(2);

    // Drive tuning parameters
    constexpr float DEADBAND = 0.125;
    constexpr float MIN_POWER = 0.4;
    constexpr float MAX_POWER = 0.85;
    constexpr float INPUT_EXPONENT = 1.4;  // Try 1.0 (linear), 2.0 (quadratic), etc.

    // Apply deadband & curve shaping to inputs before rotation math
    x  = tuneMotorPower(x,  DEADBAND, 0, 1, INPUT_EXPONENT);
    y  = tuneMotorPower(y,  DEADBAND, 0, 1, INPUT_EXPONENT);
    rx = tuneMotorPower(rx, DEADBAND, 0, 1, INPUT_EXPONENT);

    float botHeading = NoU3.yaw * angular_scale;

    float rotX = x * cos(-botHeading) - y * sin(-botHeading);
    float rotY = x * sin(-botHeading) + y * cos(-botHeading);
    rotX = rotX * 1.1;

    float calc = fabs(rotY) + fabs(rotX) + fabs(rx);
    float denominator = (calc >= 1) ? calc : 1.0;

    float frontLeftPower  = (rotY + rotX + rx) / denominator;
    float backLeftPower   = (rotY - rotX + rx) / denominator;
    float frontRightPower = (rotY - rotX - rx) / denominator;
    float backRightPower  = (rotY + rotX - rx) / denominator;

    // Apply motor output shaping after normalization
    frontLeftPower  = tuneMotorPower(frontLeftPower, 0, MIN_POWER, MAX_POWER);
    backLeftPower   = tuneMotorPower(backLeftPower,  0, MIN_POWER, MAX_POWER);
    frontRightPower = tuneMotorPower(frontRightPower, 0, MIN_POWER, MAX_POWER);
    backRightPower  = tuneMotorPower(backRightPower,  0, MIN_POWER, MAX_POWER);

    frontLeftMotor.set(frontLeftPower);
    backLeftMotor.set(backLeftPower);
    frontRightMotor.set(frontRightPower);
    backRightMotor.set(backRightPower);
    NoU3.setServiceLight(LIGHT_ENABLED);

    // Battery voltage telemetry
    float batteryVoltage = NoU3.getBatteryVoltage();
    PestoLink.printBatteryVoltage(batteryVoltage);

  } else {
    frontLeftMotor.set(0);
    backLeftMotor.set(0);
    frontRightMotor.set(0);
    backRightMotor.set(0);
    NoU3.setServiceLight(LIGHT_DISABLED);
  }
}


void arm() {
  if (PestoLink.buttonHeld(BUTTON_BOTTOM)) {
    setpoint = 1; PestoLink.printTerminal("L1");
  } else if (PestoLink.buttonHeld(BUTTON_LEFT)) {
    setpoint = 2; PestoLink.printTerminal("L2");
  } else if (PestoLink.buttonHeld(BUTTON_RIGHT)) {
    setpoint = 3; PestoLink.printTerminal("L3");
  } else if (PestoLink.buttonHeld(BUTTON_TOP)) {
    setpoint = 4; PestoLink.printTerminal("L4");
  } else if (PestoLink.buttonHeld(LEFT_BUMPER) || PestoLink.buttonHeld(LEFT_TRIGGER)) {
    setpoint = 0; updateArm(setpoint); PestoLink.printTerminal("Intake");
  } else if (PestoLink.buttonHeld(D_LEFT)) {
    setpoint = 5; PestoLink.printTerminal("Algae L2");
  } else if (PestoLink.buttonHeld(D_RIGHT)) {
    setpoint = 6; PestoLink.printTerminal("Algae L3");
  } else if (PestoLink.buttonHeld(D_DOWN)) {
    setpoint = 7; PestoLink.printTerminal("Processor");
  } else if (PestoLink.buttonHeld(D_UP)) {
    setpoint = 8; PestoLink.printTerminal("Barge");
  } else if (PestoLink.buttonHeld(MID_RIGHT)) {
    setpoint = 9; updateArm(setpoint); PestoLink.printTerminal("Stow");
  }

  if (PestoLink.buttonHeld(RIGHT_TRIGGER) && setpoint != oldSetpoint) {
    updateArm(setpoint);
    oldSetpoint = setpoint;
  }
}

void updateArm(int state) {
  angleI = positions[state][0];
  angleII = positions[state][1];
}

void claw() {
  if (PestoLink.buttonHeld(LEFT_BUMPER) || PestoLink.buttonHeld(RIGHT_TRIGGER)) {
    clawAngle = stow;
  } else {
    clawAngle = grab;
  }
}

void intake() {
  if (PestoLink.buttonHeld(LEFT_BUMPER)) {
    intakeT = 1;
  } else if (PestoLink.buttonHeld(LEFT_TRIGGER)) {
    intakeT = -1;
  } else {
    intakeT = 0;
  }
}
