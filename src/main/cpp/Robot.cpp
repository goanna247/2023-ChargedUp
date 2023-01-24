#include "Robot.h"

#include "behaviour/BehaviourScheduler.h"
#include "behaviour/Behaviour.h"
#include "behaviour/SwerveBaseBehaviour.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/event/BooleanEvent.h>
#include <units/math.h>


using namespace frc;
using namespace behaviour;

static units::second_t lastPeriodic;

void Robot::RobotInit() {
  lastPeriodic = wom::now();

  // armavator = new Armavator(map.armavator.config);
  // BehaviourScheduler::GetInstance()->Register(armavator);

  swerve = new wom::SwerveDrive(map.swerveBase.config, frc::Pose2d());
  // map.swerveBase.moduleConfigs[0].turnMotor.transmission->SetInverted(true);
  // map.swerveBase.moduleConfigs[2].turnMotor.transmission->SetInverted(true);
  BehaviourScheduler::GetInstance()->Register(swerve);
  // swerve->SetDefaultBehaviour([this]() {
  //   return make<ManualDrivebase>(swerve, &map.controllers.driver);
  // });

  
  armavator = new Armavator(map.armavator.arm.gearbox, map.armavator.elevator.gearbox, map.armavator.config);
  BehaviourScheduler::GetInstance()->Register(armavator);

  // swerve = new wom::SwerveDrive(map.SwerveBase.config, frc::Pose2d());
  // BehaviourScheduler::GetInstance()->Register(swerve);
  // swerve->SetDefaultBehaviour([this]() {
  //   return make<ManualDrivebase>(swerve, &map.controllers.driver);
  // });
}

void Robot::RobotPeriodic() {
  auto dt = wom::now() - lastPeriodic;
  lastPeriodic = wom::now();
  
  loop.Poll();
  BehaviourScheduler::GetInstance()->Tick();

  // // armavator->OnUpdate(dt);
  swerve->OnUpdate(dt);
  armavator->OnUpdate(dt);
  // swerve->OnUpdate(dt);
}

void Robot::AutonomousInit() { }
void Robot::AutonomousPeriodic() { }

void Robot::TeleopInit() {
  loop.Clear();
  // loop.Clear();

  // //Creates an instance of a behavior scheduler
  // BehaviourScheduler *sched = BehaviourScheduler::GetInstance();
  // map.controllers.codriver.A(&loop).Rising().IfHigh([sched, this]() {
  //   sched->Schedule(make<ArmavatorGoToPositionBehaviour>(armavator, ArmavatorPosition{0.2_m, 0_deg}, map.controllers.codriver));
  // });

  // map.controllers.codriver.B(&loop).Rising().IfHigh([sched, this]() {
  //   sched->Schedule(make<ArmavatorGoToPositionBehaviour>(armavator, ArmavatorPosition{1.2_m, 75_deg}, map.controllers.codriver));
  // });

  // map.controllers.codriver.X(&loop).Rising().IfHigh([sched, this]() {
  //   sched->Schedule(make<ArmavatorGoToPositionBehaviour>(armavator, ArmavatorPosition{1.0_m, 240_deg}, map.controllers.codriver));
  // });

  // map.controllers.codriver.Y(&loop).Rising().IfHigh([sched, this]() {
  //   sched->Schedule(make<ArmavatorGoToPositionBehaviour>(armavator, ArmavatorPosition{0_m, 0_deg}, map.controllers.codriver));
  // });
}

void Robot::TeleopPeriodic() {
  // map.swerveBase.turnMotors[0]->Set(0.5);
  double intakeSpeed = wom::deadzone(map.controllers.codriver.GetLeftY());

  // units::newton_meter_t max_torque_at_current_limit_i = map.intake.rightMotor.Torque(20_A);
  // units::volt_t max_voltage_for_current_limit_i = map.intake.rightMotor.Voltage(max_torque_at_current_limit_i, map.intake.rightMotor.encoder->GetEncoderAngularVelocity());
  // intakeSpeed = units::math::max(units::math::min(intakeSpeed * 11_V, max_voltage_for_current_limit_d), -max_voltage_for_current_limit_i);

  map.intake.rightMotor.Set(intakeSpeed);
  map.intake.leftMotor.Set(-intakeSpeed);

  if (map.controllers.codriver.GetAButtonReleased()) {
    if (intakeSol) {
      intakeSol = false;
    } else {
      intakeSol = true;
    }
  }

  if (intakeSol) {
    map.intake.leftSolenoid.Set(frc::DoubleSolenoid::Value::kForward);
    // leftSolenoid.Set(Value::kForward);
    std::cout << "out" << std::endl;
  } else {
    map.intake.leftSolenoid.Set(frc::DoubleSolenoid::Value::kReverse);
    // rightSolenoid.Set(Value::kReverse);
  }
  map.intake.compressor.EnableDigital();
  // if(!map.controllers.codriver.GetAButton() && !map.controllers.codriver.GetBButton() && map.controllers.codriver.GetRightTriggerAxis() <= 0.05 && map.controllers.codriver.GetLeftTriggerAxis() <= 0.05) {
  //   map.armavator.arm.gearbox.transmission->SetVoltage(0_V);
  //   map.armavator.elevator.gearbox.transmission->SetVoltage(0_V);
  // } else{
  //   if(map.controllers.codriver.GetAButton()) {
  //     map.armavator.arm.gearbox.transmission->SetVoltage(13_V);
  //   } else if (map.controllers.codriver.GetBButton()) {
  //     map.armavator.arm.gearbox.transmission->SetVoltage(-13_V);
  //   }else if(map.controllers.codriver.GetRightTriggerAxis() > 0.05) {
  //     map.armavator.elevator.gearbox.transmission->SetVoltage(13_V * map.controllers.codriver.GetRightTriggerAxis());
  //   } else if (map.controllers.codriver.GetLeftTriggerAxis() > 0.05) {
  //     map.armavator.elevator.gearbox.transmission->SetVoltage(-13_V * map.controllers.codriver.GetLeftTriggerAxis() );
  //   }
  // }
 }

void Robot::DisabledInit() { }
void Robot::DisabledPeriodic() { }

void Robot::TestInit() { }
void Robot::TestPeriodic() { }

/* SIMULATION */

// #include <frc/simulation/BatterySim.h>
// #include <frc/simulation/RoboRioSim.h>
// #include <networktables/NetworkTableInstance.h>
// #include "ControlUtil.h"

// static units::second_t lastSimPeriodic{0};
// static auto simTable = nt::NetworkTableInstance::GetDefault().GetTable("/sim");

// struct SimConfig {
//   ::sim::ArmavatorSim arm;
//   wom::sim::SwerveDriveSim swerveSim; 
// };
// SimConfig *simConfig;

// void Robot::SimulationInit() {
//   simConfig = new SimConfig{
//     ::sim::ArmavatorSim(map.armavator.config),
//     wom::sim::SwerveDriveSim(map.swerveBase.config, 0.5 * 6_lb * 7.5_in * 7.5_in)
//   };

//   lastSimPeriodic = wom::now();
// }

// void Robot::SimulationPeriodic() {
//   auto dt = wom::now() - lastSimPeriodic;

//   simConfig->arm.OnUpdate(dt);
//   simConfig->swerveSim.Update(dt);

//   auto batteryVoltage = units::math::min(units::math::max(frc::sim::BatterySim::Calculate({
//   }), 0_V), 12_V);
//   frc::sim::RoboRioSim::SetVInVoltage(batteryVoltage);
//   simTable->GetEntry("batteryVoltage").SetDouble(batteryVoltage.value()); 

//   lastSimPeriodic = wom::now();
