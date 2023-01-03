#pragma once 

#include "Gearbox.h"
#include <frc/SpeedController.h>
#include "behaviour/HasBehaviour.h"
#include "behaviour/Behaviour.h"
#include <frc/interfaces/Gyro.h>
#include "PID.h"
#include <frc/DoubleSolenoid.h>

#include <units/charge.h>
#include <frc/estimator/DifferentialDrivePoseEstimator.h>


namespace wom {

  enum class WaspDriveState {
    kIdle,
    kManualTank,
    kManualWASP,
    kPose
  };

  struct WaspDriveConfig {
    Gearbox &leftDrive;
    Gearbox &rightDrive;
    Gearbox &dropDrive;

    frc::DoubleSolenoid &dropSolenoid;

    frc::Gyro *gyro;

    units::meter_t wheelRadius;
    units::meter_t trackWidth;
  };

  class WaspDrive : public behaviour::HasBehaviour {
   public: 
    WaspDrive(WaspDriveConfig config);

    void OnUpdate(units::second_t dt);

    void SetIdle();
    void SetManualTank();
    void SetManualWASP();
    void SetTargetPose(frc::Pose2d pose);

    WaspDriveConfig &GetConfig() { return _config; }

   private: 
    WaspDriveConfig _config;
    WaspDriveState _state;

    frc::Pose2d _targetPose;

    // PIDController<units::radians, units::volt> _dropVelocityController;
  };

}