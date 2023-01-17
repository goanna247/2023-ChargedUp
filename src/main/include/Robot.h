#pragma once

#include "RobotMap.h"
#include "Vision.h"
#include <string>
#include <frc/TimedRobot.h>
#include <frc/event/EventLoop.h>
#include <ctre/phoenix.h>
#include <ctre/phoenix/motorcontrol/can/WPI_TalonSRX.h>

using namespace frc;

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

  void SimulationInit() override;
  void SimulationPeriodic() override;

 private:
  frc::EventLoop loop;
  
  RobotMap map;
  // Vision *vision;
  Armavator *armavator;
  wom::SwerveDrive *swerve;
  Intake *intake;

  // WPI_TalonSRX armMotor, elevatorMotor;
};