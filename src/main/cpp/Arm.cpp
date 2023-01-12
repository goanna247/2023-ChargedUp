#include "Arm.h"

using namespace frc;
using namespace wom;

Arm::Arm(ArmConfig config) : _config(config), _pid("arm/pid", config.pidConfig) {   
}

void Arm::OnUpdate(units::second_t dt) {
  units::volt_t voltage = 0_V;
  switch (_state) {
    case ArmState::kIdle:
      std::cout << "arm idle state" << std::endl;
      break;
    case ArmState::kZeroing:
      voltage = -2_V;
      if (_config.bottomLimitSwitch->Get()) {
        _config.gearbox.encoder->ZeroEncoder();
        _state = ArmState::kIdle;
      }
      std::cout << "arm zeroing state" << std::endl;
      break;
    case ArmState::kAngle:
      {
        voltage = -(_pid.Calculate(_config.gearbox.encoder->GetEncoderPosition(), dt));
        // units::radian_t currentAngle = _config.gearbox.encoder->GetEncoderPosition();
        // units::radian_t error = _targetAngle - currentAngle;
        // voltage = 12_V / 20_deg * error;
      }
      std::cout << "arm angle state" << std::endl;
      break;
  }
  _config.gearbox.transmission->SetVoltage(voltage);
}

void Arm::SetIdle() {
  _state = ArmState::kIdle;
}

units::angle::radian_t Arm::GetAngle() {
  return _config.gearbox.encoder->GetEncoderPosition();
}

void Arm::SetZeroing() {
  _state = ArmState::kZeroing;
}

void Arm::SetAngle(units::radian_t angle) {
  _state = ArmState::kAngle;
  _pid.SetSetpoint(angle);
}