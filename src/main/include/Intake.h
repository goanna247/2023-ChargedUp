#pragma once 

#include "behaviour/HasBehaviour.h"
#include "Gearbox.h"

enum class IntakeStates {
  kIdle, 
  kIntaking, 
  kOutaking
};

struct IntakeConfig {
  wom::Gearbox intakeGearbox;
};

class Intake : behaviour::HasBehaviour {
 public:
  Intake(IntakeConfig config);

  void OnUpdate();

  void SetIdle();
  void SetIntaking();
  void SetOutaking();

 private: 
 IntakeConfig _config;
 IntakeStates _state;
};