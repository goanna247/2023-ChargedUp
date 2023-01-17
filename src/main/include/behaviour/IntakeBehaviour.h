#pragma once

#include "Intake.h"
#include "behaviour/Behaviour.h"
#include <frc/XboxController.h>

class IntakeBehaviour : behaviour::Behaviour {
 public:
  IntakeBehaviour::IntakeBehaviour(Intake *intake, frc::XboxController *driver, frc::XboxController *coDriver);
  
  void OnTick(units::second_t dt);

 private: 
  Intake *_intake;
  frc::XboxController *_driver;
  frc::XboxController *_coDriver;
};

