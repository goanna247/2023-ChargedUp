#include "behaviour/VisionBehaviour.h"

// VisionBehaviour::VisionBehaviour(Vision *vision, frc::XboxController *codriver) 
//   : _vision(vision), _codriver(codriver) 
//   {
//     Controls(vision);
//   }

VisionBehaviour::VisionBehaviour(Vision *vision, wom::SwerveDrive *swerveDrivebase, frc::XboxController *codriver, frc::XboxController *driver) : _vision(vision), _swerveDrivebase(swerveDrivebase), _codriver(codriver), _driver(driver)
  {
    Controls(_vision);
  }

void VisionBehaviour::OnTick(units::second_t dt) {
  // units::radian_t tapeXPos = _vision->GetTapePos().convert<units::radian_t>().value();
  // units::radian_t currentAngle = _swerveDrivebase->GetConfig().gyro->GetAngle().convert<units::radian_t>().value();

  // if (_driver.GetYButton()) {
  //   _swerveDrivebase->SetPID(currentAngle + tapeXPos)
  // }
}