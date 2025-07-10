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

float applyDeadband(float value, float deadband) {
  if (fabs(value) < deadband) return 0;
  return value;
}

void chassis() {
  if (PestoLink.update()) {
    float y = -PestoLink.getAxis(1); // Remember, Y stick value is reversed
    float x = PestoLink.getAxis(0);
    float rx = PestoLink.getAxis(2);

    // Apply deadband to controller inputs
    x = applyDeadband(x, 0.1);
    y = applyDeadband(y, 0.1);
    rx = applyDeadband(rx, 0.1);

    float botHeading = NoU3.yaw * angular_scale;

    // Rotate the movement direction counter to the bot's rotation
    float rotX = x * cos(-botHeading) - y * sin(-botHeading);
    float rotY = x * sin(-botHeading) + y * cos(-botHeading);

    rotX = rotX * 1.1;  // Counteract imperfect strafing

    float calc = fabs(rotY) + fabs(rotX) + fabs(rx);
    float denominator = (calc >= 1) ? calc : 1.0;

    float frontLeftPower = (rotY + rotX + rx) / denominator;
    float backLeftPower = (rotY - rotX + rx) / denominator;
    float frontRightPower = (rotY - rotX - rx) / denominator;
    float backRightPower = (rotY + rotX - rx) / denominator;

    frontLeftMotor.set(frontLeftPower);
    backLeftMotor.set(backLeftPower);
    frontRightMotor.set(frontRightPower);
    backRightMotor.set(backRightPower);
    NoU3.setServiceLight(LIGHT_ENABLED);
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
