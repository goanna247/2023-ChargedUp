#include "Robot.h"

#include <frc/smartdashboard/SmartDashboard.h>

using namespace frc;

double currentTimeStamp;
double lastTimeStamp;
double dt;

void Robot::RobotInit() {

  // auto camera = CameraServer::GetInstance()->StartAutomaticCapture(0);
  // camera.SetFPS(30);
  // camera.SetResolution(160, 120);

  driver = new frc::XboxController(0);

  leftOne = new TalonSRX(1);
  leftTwo = new TalonSRX(3);
  leftThree = new TalonSRX(4);

  rightOne = new TalonSRX(7);
  rightTwo = new TalonSRX(5);
  rightThree = new TalonSRX(6);

  // arm = new TalonSRX(5);
  // gripper = new frc::DoubleSolenoid(6, frc::PneumaticsModuleType::CTREPCM, 0, 1);
}
void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {}
void Robot::TeleopPeriodic() {
  double leftSpeed = driver->GetRightY() > 0.1 || driver->GetRightY() < -0.1 ? driver->GetRightY() : 0;
  double rightSpeed = driver->GetLeftY() > 0.1 || driver->GetLeftY() < -0.1 ? driver->GetLeftY() : 0;

  rightOne->Set(ControlMode::PercentOutput, leftSpeed); //yes
  rightTwo->Set(ControlMode::PercentOutput, -leftSpeed); //yes
  rightThree->Set(ControlMode::PercentOutput, -leftSpeed);
  leftOne->Set(ControlMode::PercentOutput, -rightSpeed);
  leftTwo->Set(ControlMode::PercentOutput, rightSpeed); //yes
  leftThree->Set(ControlMode::PercentOutput, -rightSpeed); //yes

  // double armSpeed = driver->GetLeftTriggerAxis() > 0.1 ? driver->GetLeftTriggerAxis() : 0;
  // armSpeed = driver->GetRightTriggerAxis() > 0.1 ? -driver->GetRightTriggerAxis() : 0;

  // armSpeed *= 0.4;

  // arm->Set(ControlMode::PercentOutput, armSpeed);

  // if (driver->GetRightBumper()) {
  //   gripper->DoubleSolenoid::Set(frc::DoubleSolenoid::kForward);
  // } else {
  //   gripper->DoubleSolenoid::Set(frc::DoubleSolenoid::kReverse);
  // }

}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}