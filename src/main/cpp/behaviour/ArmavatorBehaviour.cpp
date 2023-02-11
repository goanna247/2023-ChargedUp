#include "behaviour/ArmavatorBehaviour.h"

//Constructs class
ArmavatorGoToPositionBehaviour::ArmavatorGoToPositionBehaviour(Armavator *armavator, ArmavatorPosition setpoint)
: _armavator(armavator), _setpoint(setpoint) {
  //tells code that the points are controlled (one point at a time) 
  Controls(armavator);
};

// Function for OnStart
void ArmavatorGoToPositionBehaviour::OnStart() {
  std::cout << "On Start" << std::endl;
}

//Function for OnTick
void ArmavatorGoToPositionBehaviour::OnTick(units::second_t dt) {

  if(_setpoint.height < 1_m) {
    if (_setpoint.angle >  0_rad){ // _setpoint.angle.value() < 0
      _setpoint.angle = 0_rad;
    }
    if (_setpoint.angle > 90_rad){
      _setpoint.angle = 90_rad;
    }
  };
  _armavator->SetPosition(_setpoint);

  if (_armavator->IsStable())
    SetDone();
}


// Got to sensor on elevator code

ArmavatorGoToSensor::ArmavatorGoToSensor(Armavator *armavator, ArmavatorPosition setpoint, frc::XboxController &controller) : _armavator(armavator), _setpoint(setpoint), _controller(controller){
  Controls(armavator);
}

void ArmavatorGoToSensor::OnStart() {}

void ArmavatorGoToSensor::OnTick(units::second_t dt) {
  units::centimeter_t height;
  double sensorHeight = 0;
  if (_controller.GetAButton()) {
    height = _armavator->lowerHEHeight;
    sensorHeight = 1;
  } else if (_controller.GetBButton()) {
    height = _armavator->lowerMiddleHEHeight;
    sensorHeight = 2;
  } else if (_controller.GetXButton()) {
    height = _armavator->highMiddleHEHeight;
    sensorHeight = 3;
  } else if (_controller.GetYButton()) {
    height = _armavator->highHEHeight;
    sensorHeight = 4;
  } else {
    _armavator->SetIdle();
    sensorHeight = 0;
  }

  if ((sensorHeight == 1) && (_armavator->GetConfig().lowerHE) && (height != _armavator->elevator->GetHeight())) {
    height = _setpoint.height;
  } else if ((sensorHeight == 2) && (_armavator->GetConfig().lowerMiddleHE) && (height != _armavator->elevator->GetHeight())) {
    height = _setpoint.height;
  } else if ((sensorHeight == 3) && (_armavator->GetConfig().highMiddleHE) && (height != _armavator->elevator->GetHeight())) {
    height = _setpoint.height;
  } else if ((sensorHeight == 4) && (_armavator->GetConfig().highHE) && (height != _armavator->elevator->GetHeight())) {
    height = _setpoint.height;
  }

  _setpoint.height = height; 
  _armavator->SetPosition(_setpoint);

  if (_armavator->IsStable())
    SetDone();
}


// Raw elevator code

ArmavatorRawBehaviour::ArmavatorRawBehaviour(Armavator *armavator)
: _armavator(armavator) {
  //tells code that the points are controlled (one point at a time) 
  _setpoint.height = 0.0_m;
  _setpoint.angle = 0.0_deg;
  Controls(armavator);
};

void ArmavatorRawBehaviour::OnStart() {
}


void ArmavatorRawBehaviour::OnTick(units::second_t dt) {
  //Raw Positioning
  // _setpoint.angle = getCorrectAngle(_setpoint.height);
  // _armavator->SetRaw(
  //   -_tester.GetLeftY() * 9_V,
  //   -_tester.GetRightY() * 9_V
  // );
}


ArmavatorManualBehaviour::ArmavatorManualBehaviour(Armavator *armavator, frc::XboxController &codriver, frc::XboxController &tester) 
  : _armavator(armavator), _codriver(codriver), _tester(tester) {
    Controls(armavator);
}

void ArmavatorManualBehaviour::OnStart() {
  startHeight = _armavator->GetCurrentPosition().height;
  _manualSetpoint = _armavator->GetCurrentPosition();
}

void ArmavatorManualBehaviour::OnTick(units::second_t dt) {

  // if (_codriver.GetBButtonReleased()) {
  //   if (rawControl) {
  //     rawControl = false;
  //   } else {
  //     rawControl = true;
  //   }
  // }
 
  // if (rawControl) {
  //   // _armavator->SetManual(
  //   //   -_codriver.GetLeftY() * 9_V,
  //   //   -_codriver.GetRightY() * 9_V
  //   // );
  // } else {
  //   if (wom::deadzone(_codriver.GetLeftY())) {
  //     _manualSetpoint.angle += (_codriver.GetLeftY() * 1_deg);
  //   }

  //   if (wom::deadzone(_codriver.GetRightY())) {
  //     _manualSetpoint.height += (_codriver.GetRightY() * 1_m);
  //   }

  //   std::cout << _manualSetpoint.angle.convert<units::degree>().value() << std::endl;
  //   std::cout << _manualSetpoint.height.convert<units::meter>().value() << std::endl;

  //   _armavator->SetPosition(_manualSetpoint);
  // }


  if (_tester.GetXButtonPressed()) {
    _heightSetpoint = 0_m;
  } else if (_tester.GetYButtonPressed()) {
    _heightSetpoint = 0.4_m;
  } else if (_tester.GetBButtonPressed()) {
    _heightSetpoint = 0.7_m;
  } else if (_tester.GetAButtonPressed()) {
    _heightSetpoint = 1_m;
  }

  // if (_tester.LeftBumper()) {
  //   _heightSetpoint -= 0.1;
  // } else if (_tester.RightBumper()) {
  //   _heightSetpoint += 0.1;
  // }

  // if (_tester.LeftTrigger()) {
  //   _armSetpoint -= 0.1;
  // } else if (_tester.RightTrigger()) {
  //   _armSetpoint += 0.1;
  // }
  // _armavator->SetPosition()
}