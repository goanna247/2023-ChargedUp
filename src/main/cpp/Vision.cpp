// #include "Vision.h"
// #include <networktables/NetworkTableInstance.h>
// #include <wpi/json.h>
// #include <frc/apriltag/AprilTagFields.h>

// Vision::Vision() {}

// void Vision::Update(units::second_t dt) {
//   // photonlib::PhotonPipelineResult result = camera.GetLatestResult();
//   double distance = _visionTable->GetEntry("TargetYaw").GetDouble(0);
//   std::cout << distance << std::endl;
// }

// std::shared_ptr<frc::AprilTagFieldLayout> Get2023Layout() {
//   return std::make_shared<frc::AprilTagFieldLayout>(frc::LoadAprilTagLayoutField(frc::AprilTagField::k2023ChargedUp));
// }