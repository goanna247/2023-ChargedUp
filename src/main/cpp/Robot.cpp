#include "Robot.h"

#include <frc/smartdashboard/SmartDashboard.h>

#include <behaviour/BehaviourScheduler.h>

using namespace frc;
using namespace wom;
using namespace behaviour;

double currentTimeStamp;
double lastTimeStamp;
double dt;

// TalonSRX intakeMotor(4);

void Robot::RobotInit() {
  // ShooterParams shooterParams{map.shooter.shooterGearbox, map.shooter.pid, map.shooter.currentLimit};
  // shooter = new Shooter("shooter", shooterParams);

  
  // BehaviourScheduler::GetInstance()->Register(shooter);
  // shooter->SetDefaultBehaviour([this]() {
  //   return make<ShooterConstant>(shooter, 0_V);
  // });

  // intakeMotor = new TalonSRX(99);
}
void Robot::RobotPeriodic() {
  // shooter->OnUpdate(20_ms);
  
  // BehaviourScheduler::GetInstance()->Tick();
}

void Robot::AutonomousInit() {
  // BehaviourScheduler::GetInstance()->Schedule(
  //   make<ShooterSpinup>(shooter, 1500_rpm)
  //   << make<WaitTime>(1_s)
  //   << make<ShooterSpinup>(shooter, 2000_rpm)
  //   << make<WaitTime>(1_s)
  //   << make<ShooterSpinup>(shooter, 3000_rpm)
  // );

}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {}
void Robot::TeleopPeriodic() {
  // intakeMotor.Set(ControlMode::PercentOutput, 0.6);
  // intakeMotor->SetVoltage(12_V);
}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}