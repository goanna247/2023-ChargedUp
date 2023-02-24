#include "Robot.h"
#include "behaviour/BehaviourScheduler.h"
#include "behaviour/Behaviour.h"
#include "behaviour/SwerveBaseBehaviour.h"
#include "behaviour/SideIntakeBehaviour.h"
#include "behaviour/GripperBehaviour.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/event/BooleanEvent.h>
#include <units/math.h>
#include <networktables/NetworkTableInstance.h>


#include "Auto.h"

using namespace frc;
using namespace behaviour;

static units::second_t lastPeriodic;

void Robot::RobotInit() {
  lastPeriodic = wom::now();

  // motor = new rev::CANSparkMax(11, rev::CANSparkMax::MotorType::kBrushless);
  // encoder = new rev::CANEncoder(&motor);
}

void Robot::RobotPeriodic() {
  auto dt = wom::now() - lastPeriodic;
  lastPeriodic = wom::now();
  
  loop.Poll();
  BehaviourScheduler::GetInstance()->Tick();

  frc::SmartDashboard::PutNumber("Encoder Position", map.armavator.elevator.m_encoder.GetPosition());
  frc::SmartDashboard::PutNumber("Encoder Velocity", map.armavator.elevator.m_encoder.GetVelocity());

  // std::cout << encoder->GetPosition() << std::endl;

}

void Robot::AutonomousInit() {

  BehaviourScheduler *sched = BehaviourScheduler::GetInstance();
  sched->Schedule(Drive(swerve, &map.swerveBase.gyro));
 }

void Robot::AutonomousPeriodic() { }

void Robot::TeleopInit() {
  loop.Clear();
  BehaviourScheduler *sched = BehaviourScheduler::GetInstance();
  sched->InterruptAll(); // removes all previously scheduled behaviours

}

void Robot::TeleopPeriodic() {

  // std::cout << "encoder: " << map.armavator.elevator.rightEncoder.GetEncoderRawTicks() << std::endl;

  map.armTable.armManualTable->GetEntry("armSetpoint").SetDouble(_armSetpoint.convert<units::degree>().value());
  map.armTable.armManualTable->GetEntry("elevatorSetpoint").SetDouble(_elevatorSetpoint.convert<units::meter>().value());
}

void Robot::DisabledInit() { 
  // map.controlSystem.pcmCompressor.Disable();
}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() { }
void Robot::TestPeriodic() { }







// /*----------------------------------------------------------------------------*/
// /* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
// /* Open Source Software - may be modified and shared by FRC teams. The code   */
// /* must be accompanied by the FIRST BSD license file in the root directory of */
// /* the project.                                                               */
// /*----------------------------------------------------------------------------*/

// #include <frc/Joystick.h>
// #include <frc/TimedRobot.h>
// #include <frc/smartdashboard/SmartDashboard.h>
// #include "rev/CANSparkMax.h"

// /**
//  * Sample program displaying position and velocity on the SmartDashboard
//  * 
//  * Position is displayed in revolutions and velocity is displayed in RPM
//  */
// class Robot : public frc::TimedRobot {
//   // initialize SPARK MAX
//   static const int deviceID = 11;
//   rev::CANSparkMax m_motor{deviceID, rev::CANSparkMax::MotorType::kBrushless};
//   rev::SparkMaxRelativeEncoder m_encoder = m_motor.GetEncoder();

//  public:
//   Robot() { }

//   void TeleopPeriodic() override {
//     frc::SmartDashboard::PutNumber("Encoder Position", m_encoder.GetPosition());
//     frc::SmartDashboard::PutNumber("Encoder Velocity", m_encoder.GetVelocity());
//   }
// };

// #ifndef RUNNING_FRC_TESTS
// int main() { return frc::StartRobot<Robot>(); }
// #endif