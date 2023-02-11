#include "Armavator.h"
#include <units/length.h>

//Armavator configeration
Armavator::Armavator(wom::Gearbox &armGearbox, wom::Gearbox &elevatorGearbox, ArmavatorConfig &config)
: _armGearbox(armGearbox), _elevatorGearbox(elevatorGearbox), _config(config) {
  arm = new wom::Arm(config.arm);
  elevator = new wom::Elevator(config.elevator);
}

Armavator::~Armavator() {
  free(arm);
  free(elevator);
}

//Instructions for when the program updates (seconds delta time)
void Armavator::OnUpdate(units::second_t dt) {
  units::volt_t voltage{0};

  switch(_state) {
    case ArmavatorState::kIdle:
      voltage = 0_V;
      break;
    case ArmavatorState::kPosition: // holds it into that pos
      arm->SetAngle(_setpoint.angle);
      elevator->SetPID(_setpoint.height);
      break;
    case ArmavatorState::kRaw:
      arm->SetRaw(_rawArm);
      elevator->SetManual(_rawElevator);
      break;
    case ArmavatorState::kHE_Calibration:
      units::centimeter_t height;
      if (_config.lowerHE) {
        height = lowerHEHeight;
      } else if (_config.lowerMiddleHE) {
        height = lowerMiddleHEHeight;
      } else if (_config.highMiddleHE) {
        height = highMiddleHEHeight;
      } else if (_config.highHE) {
        height = highHEHeight;
      } else {
        voltage = -5_V;
      }
      double height_in_rotations = double(height*1)/4.9; // get height in rotations based of circum
      units::degree_t height_in_degrees = height_in_rotations * 360_deg; // 
      elevator->GetConfig().gearbox.encoder->SetEncoderPosition(height_in_degrees);
      break;

      
      
    elevator->GetConfig().gearbox.transmission->SetVoltage(voltage);
  }

  arm->OnUpdate(dt);
  elevator->OnUpdate(dt);
}

//Sets the states names
//idle state
void Armavator::SetIdle() {
  _state = ArmavatorState::kIdle;
}

//set positions state
void Armavator::SetPosition(ArmavatorPosition pos) {
  _state = ArmavatorState::kPosition;
  _setpoint = pos;
}

//set elevator to HE sensor pos
// void Armavator::GoToSensor(int sensorNum) {
//   _state = ArmavatorState::kSensorHeight;
//   _sensorNum = sensorNum;
// }

//manual state setup
void Armavator::SetRaw(units::volt_t arm, units::volt_t elevator) {
  _state = ArmavatorState::kRaw;
  _rawArm = arm;
  _rawElevator = elevator;
}

void Armavator::SetHECalibration() {
  _state = ArmavatorState::kHE_Calibration;
}

//returns the current position
ArmavatorPosition Armavator::GetCurrentPosition() const {
  return ArmavatorPosition {
    elevator->GetHeight(),
    arm->GetAngle()
  };
}

//determines if the armavator is stable/done
bool Armavator::IsStable() const {
  return elevator->IsStable() && arm->IsStable();
}

ArmavatorConfig &Armavator::GetConfig() {
  return _config;
}