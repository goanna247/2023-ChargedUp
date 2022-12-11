#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/XboxController.h>
#include <frc/DoubleSolenoid.h>
#include <ctre/Phoenix.h>
#include <iostream>
#include <cameraserver/CameraServer.h>

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;

 private:
  frc::XboxController *driver;
  frc::DoubleSolenoid *gripper;
  TalonSRX *leftOne, *leftTwo, *leftThree, *rightOne, *rightTwo, *rightThree, *intake;
  VictorSPX *shoot;
  // VictorSPX *rightTwo;
};
