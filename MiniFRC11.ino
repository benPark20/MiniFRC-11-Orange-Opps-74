#include <PestoLink-Receive.h>
#include <Alfredo_NoU3.h>
#include <constants.h>

// If your robot has more than a drivetrain, add those actuators here
NoU_Motor frontLeftMotor(1);
NoU_Motor frontRightMotor(2);
NoU_Motor rearLeftMotor(3);
NoU_Motor rearRightMotor(4);

NoU_Motor intakeMotor(5);
float intakeT = 0;

NoU_Servo stageI(1);
float angleI = positions[9][0];
NoU_Servo stageII(2);
float angleII = positions[9][1];
NoU_Servo clawServo(3);
float clawAngle = grab;

int setpoint = 0;
// This creates the drivetrain object, you shouldn't have to mess with this
NoU_Drivetrain drivetrain(&frontLeftMotor, &frontRightMotor, &rearLeftMotor, &rearRightMotor);
float angular_scale;
float measured_angle;
void setup() {
  PestoLink.begin("Team 74");
  Serial.begin(115200);

  NoU3.begin();

  //The gyroscope sensor is by default precise, but not accurate. This is fixable by adjusting the angular scale factor.
  //Tuning procedure:
  //Rotate the robot in place 5 times. Use the Serial printout to read the current gyro angle in Radians, we will call this "measured_angle".
  //measured_angle should be nearly 31.416 which is 5*2*pi. Update measured_angle below to complete the tuning process.
  measured_angle = 31.416;
  angular_scale = (5.0 * 2.0 * PI) / measured_angle;
  NoU3.calibrateIMUs();  // this takes exactly one second. Do not move the robot during calibration.

  frontLeftMotor.setInverted(true);
  rearLeftMotor.setInverted(true);
  frontRightMotor.setInverted(false);
  rearRightMotor.setInverted(false);
  updateArm(0);
}

unsigned long lastPrintTime = 0;

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
  if (lastPrintTime + 100 < millis()) {
    Serial.printf("gyro yaw (radians): %.3f\r\n", NoU3.yaw * 1.145);
    lastPrintTime = millis();
  }



  // This measures your batteries voltage and sends it to PestoLink
  float batteryVoltage = NoU3.getBatteryVoltage();
  PestoLink.printBatteryVoltage(batteryVoltage);

  if (PestoLink.update()) {
    float fieldPowerX = -PestoLink.getAxis(1);
    float fieldPowerY = PestoLink.getAxis(0);
    float rotationPower = -PestoLink.getAxis(2);

    // Get robot heading (in radians) from the gyro
    float heading = NoU3.yaw * angular_scale;

    // Rotate joystick vector to be robot-centric
    float cosA = cos(heading);
    float sinA = sin(heading);

    float robotPowerX = fieldPowerX * cosA + fieldPowerY * sinA;
    float robotPowerY = -fieldPowerX * sinA + fieldPowerY * cosA;

    //set motor power
    drivetrain.holonomicDrive(robotPowerX, robotPowerY, rotationPower);

    NoU3.setServiceLight(LIGHT_ENABLED);
  } else {
    NoU3.setServiceLight(LIGHT_DISABLED);
  }
}

void arm() {
  if(PestoLink.buttonHeld(BUTTON_BOTTOM)){
    setpoint = 1; //L1
    PestoLink.printTerminal("L1");
  } else if(PestoLink.buttonHeld(BUTTON_LEFT)){
    setpoint = 2; //L2
    PestoLink.printTerminal("L2");
  } else if(PestoLink.buttonHeld(BUTTON_RIGHT)){
    setpoint = 3; //L3
    PestoLink.printTerminal("L3");
  } else if(PestoLink.buttonHeld(BUTTON_TOP)){
    setpoint = 4; //L4
    PestoLink.printTerminal("L4");
  } else if(PestoLink.buttonHeld(LEFT_BUMPER) || PestoLink.buttonHeld(LEFT_TRIGGER)){
    setpoint = 0; //Intake
    updateArm(setpoint);
    PestoLink.printTerminal("Intake");
  } else if(PestoLink.buttonHeld(D_LEFT)){
    setpoint = 5; //Algae L2
    PestoLink.printTerminal("Algae L2");
  } else if(PestoLink.buttonHeld(D_RIGHT)){
    setpoint = 6; //Algae L3
    PestoLink.printTerminal("Algae L3");
  } else if(PestoLink.buttonHeld(D_DOWN)){
    setpoint = 7; //Processor
    PestoLink.printTerminal("Processor");
  } else if(PestoLink.buttonHeld(D_UP)){
    setpoint = 8; //Barge
    PestoLink.printTerminal("Barge");
  } else if(PestoLink.buttonHeld(MID_RIGHT)){
    setpoint = 9; //Stow
    updateArm(setpoint);
    PestoLink.printTerminal("Stow");
  }
  if(PestoLink.buttonHeld(RIGHT_TRIGGER)){
    updateArm(setpoint);
  }
}

void updateArm(int state){
  angleI = positions[state][0];
  angleII = positions[state][1];
}

void claw(){
  if(PestoLink.buttonHeld(LEFT_BUMPER) || PestoLink.buttonHeld(RIGHT_TRIGGER)){
    clawAngle = stow;
  } else {
    clawAngle = grab;
  }
}

void intake(){
  if(PestoLink.buttonHeld(LEFT_BUMPER)){
    intakeT = 1;
  } else if(PestoLink.buttonHeld(LEFT_TRIGGER)){
    intakeT = -1;
  } else {
    intakeT = 0;
  }
}