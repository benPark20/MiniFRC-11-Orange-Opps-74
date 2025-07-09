#include <PestoLink-Receive.h>
#include <Alfredo_NoU3.h>
#include <constants.h>

// Motors                         // 654321 //
NoU_Motor frontLeftMotor(5);      //5      4//
NoU_Motor frontRightMotor(4);     //6      3//
NoU_Motor rearLeftMotor(8);       //7      2//
NoU_Motor rearRightMotor(1);      //8      1//
NoU_Motor intakeMotor(3);         // USB-C  //

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
NoU_Drivetrain drivetrain(&frontLeftMotor, &frontRightMotor, &rearLeftMotor, &rearRightMotor);

void setup() {
  PestoLink.begin("Team 74");
  Serial.begin(115200);

  NoU3.begin();
  NoU3.setServiceLight(LIGHT_ENABLED);
  drivetrain.setMaximumOutput(0.8);
  drivetrain.setMinimumOutput(0.3);
  drivetrain.setInputDeadband(0.15);

  measured_angle = 31.416; // Tune this by spinning 5 full times
  angular_scale = (5.0 * 2.0 * PI) / measured_angle;
  NoU3.calibrateIMUs(); // Takes exactly 1 second

  frontLeftMotor.setInverted(true);
  rearLeftMotor.setInverted(true);
  frontRightMotor.setInverted(false);
  rearRightMotor.setInverted(false);
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

void chassis() {
  if (PestoLink.update()) {
    float driveY = -PestoLink.getAxis(1); // Forward/Backward
    float driveX = PestoLink.getAxis(0);  // Strafing
    float rotate = -PestoLink.getAxis(2); // Rotation

    //float heading = NoU3.yaw * angular_scale;

    //float cosA = cos(heading);
    //float sinA = sin(heading);

    //float robotX = driveX * cosA + driveY * sinA;
    //float robotY = -driveX * sinA + driveY * cosA;

    drivetrain.holonomicDrive(driveX, driveY, rotate); //change to robot for field oriented

    // Battery voltage telemetry
    float batteryVoltage = NoU3.getBatteryVoltage();
    PestoLink.printBatteryVoltage(batteryVoltage);
    NoU3.setServiceLight(LIGHT_ENABLED);
  } else {
    drivetrain.holonomicDrive(0, 0, 0);
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
