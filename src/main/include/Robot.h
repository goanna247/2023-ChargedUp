#pragma once

#include <string>
#include "RobotMap.h"

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
// #include "startup.h"
// #include "RobotStart.h"
// #include "RobotStartup.h"
// #include "Encoder.h"
// #include "Shooter.h"

#include <ctre/Phoenix.h>
#include <frc/XboxController.h>

// #include <frc/RobotBase.h>

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
  RobotMap map;
  // wom::Shooter *shooter;

  // TalonSRX *intakeMotor;
  // VictorSPX *otherMotor;
  // TalonFX *falconMotor;
};
