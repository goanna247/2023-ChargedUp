#include "behaviour/SideIntakeBehaviour.h"

#include <iostream>

SideIntakeBehaviour::SideIntakeBehaviour(SideIntake *sideIntake, frc::XboxController &codriver): sideIntake(sideIntake), _codriver(codriver) {
  Controls(sideIntake);
}

void SideIntakeBehaviour::OnStart() {}

void SideIntakeBehaviour::OnTick(units::second_t dt) {
  // if (_codriver.GetLeftTriggerAxis() > 0.05) {
  //   sideIntake->SetIntakingWide();
  // } else if (_codriver.GetRightTriggerAxis() > 0.05) {
  //   sideIntake->SetOutakingWide();
  // } else {
  //   sideIntake->SetWide();
  // }

  if (_codriver.GetBButtonPressed()) {
    if (_intakeReleased) {
      _intakeReleased = false;
    } else {
      _intakeReleased = true;
    }
  }

  if (_codriver.GetLeftBumperPressed()) {
    if (_intakeGrapper) {
      _intakeGrapper = false;
    } else {
      _intakeGrapper = true;
    }
  }

  if (_intakeReleased) {
    sideIntake->SetStow();
  } else {
    sideIntake->SetDeploy();
  }

  if (_intakeGrapper) {
    sideIntake->SetClose();
  } else {
    sideIntake->SetOpen();
  }

  // std::cout << sideIntake->GetState() << std::endl;

  if (wom::deadzone(_codriver.GetXButton())) {
    sideIntake->SetVoltage(1 * -10_V);
  } else if (wom::deadzone(_codriver.GetLeftTriggerAxis())) {
    sideIntake->SetVoltage(_codriver.GetLeftTriggerAxis() * 10_V);
  } else {
    sideIntake->SetVoltage(0_V);
  }

  // if (_codriver.GetAButtonReleased()) {
  //   if (_intakeReleased) {
  //     _intakeReleased = false;
  //   } else {
  //     _intakeReleased = true;
  //   }
  // }

  // if (_codriver.GetXButtonReleased()) {
  //   if (_intakeGrapper) {
  //     _intakeGrapper = false;
  //   } else {
  //     _intakeGrapper = true;
  //   }
  // }

  // if (_intakeReleased) {
  //   if (_intakeGrapper) {
  //     if (_intakeSpeed > 0.05) {
  //       sideIntake->SetIntakingWide();
  //     } else if (_intakeSpeed < -0.05) {
  //       sideIntake->SetOutakingWide();
  //     } else {
  //       sideIntake->SetWide();
  //     }
  //   } else {
  //     if (_intakeSpeed > 0.05) {
  //       sideIntake->SetIntakingClosed();
  //     } else if (_intakeSpeed < -0.05) {
  //       sideIntake->SetOutakingClosed();
  //     } else {
  //       sideIntake->SetClosed();
  //     }
  //   }
  // } else {
  //   sideIntake->SetStowed();
  // }
}