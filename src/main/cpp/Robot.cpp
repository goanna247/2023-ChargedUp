#include "Robot.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include "behaviour/BehaviourScheduler.h"
#include "behaviour/Behaviour.h"

using namespace frc;
using namespace behaviour;

void Robot::RobotInit() {
  /* Create a new intake */
  intake = new Intake(map.intake.config);
  BehaviourScheduler::GetInstance()->Register(intake);

  mecanumDrivebase = new MecanumDrivebase(map.mecanumDriveSystem.config);
  BehaviourScheduler::GetInstance()->Register(mecanumDrivebase);
  mecanumDrivebase->SetDefaultBehaviour([this]() {
    return make<ManualDrivebase>(mecanumDrivebase, &map.controllers.driver);
  });

  arm = new Arm(map.arm.config);//new arm
  BehaviourScheduler::GetInstance()->Register(arm);
  map.arm.config.gearbox.transmission->SetInverted(false);

  climber = new Climber(map.climber.config);
  BehaviourScheduler::GetInstance()->Register(climber);
}
void Robot::RobotPeriodic() {
  /* Update the intake */
  intake->OnUpdate(20_ms);//intake
  mecanumDrivebase->OnUpdate(20_ms);
  arm->OnUpdate(20_ms);
  climber->OnUpdate(20_ms);

  std::cout << arm->GetAngle().convert<units::degree>().value() << std::endl;
  BehaviourScheduler::GetInstance()->Tick();
}

void Robot::AutonomousInit() { }
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
  // arm->ZeroEncoder()

  auto sched = BehaviourScheduler::GetInstance();
  sched->Schedule(make<IntakeBehaviour>(intake, &map.controllers.coDriver));
  sched->Schedule(make<ClimberBehaviour>(climber, &map.controllers.coDriver));
  sched->Schedule(make<ManualDrivebase>(mecanumDrivebase, &map.controllers.driver));

  arm->SetZeroing();//set the amr to zero
}
void Robot::TeleopPeriodic() {
  // if(map.controllers.driver.GetAButton())
  //   intake->SetIntaking();
  // if(map.controllers.driver.GetBButton())
  //   intake->SetOuttaking();

    // double l_x = map.controllers.driver.GetLeftX();
    // double l_y = map.controllers.driver.GetLeftY();
    // double r_x = map.controllers.driver.GetRightX();

    // // deals with controller deadzones
    // if (-driverDeadzone <= l_x && l_x <= driverDeadzone) {l_x = 0;}
    // if (-driverDeadzone <= l_y && l_y <= driverDeadzone) {l_y = 0;}
    // if (-turningDeadzone <= r_x && r_x <= turningDeadzone) {r_x = 0;}

    // mecanumDrivebase->SetVelocity(frc::ChassisSpeeds {
    //     l_x * maxMovementMagnitude,
    //     l_y * maxMovementMagnitude,
    //     r_x * 180_deg / 1_s
    // });
  double armSpeed = map.controllers.coDriver.GetRightY();//gets the Y bu on

  if (map.controllers.coDriver.GetXButton()) {
    if (armManualControl) {
      armManualControl = false;//sets the arm to be in manual control
    } else {
      armManualControl = true;//sets the arm to not be in manual control
    }
  }

  if (armManualControl) {
    arm->SetManual(armSpeed);
    std::cout << "arm manual speed" << std::endl;
  } else {
    if (map.controllers.driver.GetAButton())
      arm->SetAngle(0_deg);//sets the angle to 0 degrees
    if (map.controllers.driver.GetYButton())
      arm->SetAngle(45_deg);//sets the angle to 45 degrees
  }
}


void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

/* SIMULATION */

// #include <frc/simulation/DIOSim.h>
// #include <networktables/NetworkTableInstance.h>

// frc::sim::DIOSim *_sim_limit;

// void Robot::SimulationInit() {
//   _sim_limit = new frc::sim::DIOSim(map.arm.limitSwitch);
// }

// units::radian_t _sim_arm_angle{0};

// void Robot::SimulationPeriodic() {
//   _sim_arm_angle += map.arm.gearbox.motor.Speed(90_N * units::math::cos(_sim_arm_angle) * 1_m, map.arm.controller.GetVoltage()) * 20_ms;
  
//   if (_sim_arm_angle <= 0_rad) {
//     _sim_arm_angle = 0_rad;
//     _sim_limit->SetValue(true);
//   } else {
//     _sim_limit->SetValue(false);
//   }

//   if (_sim_arm_angle >= 90_deg) {
//     _sim_arm_angle = 90_deg;
//   }

//   map.arm.encoder.SetTurns(_sim_arm_angle);

//   nt::NetworkTableInstance::GetDefault().GetEntry("arm/angle").SetDouble(_sim_arm_angle.convert<units::degree>().value());
// }