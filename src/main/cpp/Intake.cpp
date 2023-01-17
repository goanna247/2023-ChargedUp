#include "Intake.h"

Intake::Intake(IntakeConfig config) 
: _config(config) {}

void Intake::OnUpdate() {

}

void Intake::SetIdle() {
  _state = IntakeStates::kIdle;
}

void Intake::SetIntaking() {
  _state = IntakeStates::kIntaking;
}

void Intake::SetOutaking() {
  _state = IntakeStates::kOutaking;
}