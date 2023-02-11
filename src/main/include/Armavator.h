#pragma once

#include "Arm.h"
#include "Elevator.h"
#include "Gearbox.h"
#include "Grid.h"
#include "drivetrain/SwerveDrive.h"

#include <frc/DigitalInput.h>
#include <frc/simulation/DIOSim.h>
#include <frc/simulation/ElevatorSim.h>
#include <units/velocity.h>
#include <ctre/Phoenix.h>
#include <units/math.h>
#include "behaviour/HasBehaviour.h"
#include <frc/DigitalInput.h>

//the config class
struct ArmavatorConfig {
  frc::DigitalInput *lowerHE;
  frc::DigitalInput *lowerMiddleHE;
  frc::DigitalInput *highMiddleHE;
  frc::DigitalInput *highHE;
  using grid_t = wom::DiscretisedOccupancyGrid<units::radian, units::meter>;

  //uses the configs from gthe arm and elevator, as well as includes the grid
  wom::ArmConfig arm;
  wom::ElevatorConfig elevator;
  grid_t grid;
};

//class of info for setting positions
struct ArmavatorPosition {
  units::meter_t height;
  units::radian_t angle;
};

//creates the states used to control the robot
enum class ArmavatorState {
  kIdle,
  kPosition, // holding it in place
  kRaw,
  kHE_Calibration
};

//the behaviour class information
class Armavator : public behaviour::HasBehaviour {
 public:
  Armavator(wom::Gearbox &armGearbox, wom::Gearbox &elevatorGearbox, ArmavatorConfig &config);
  ~Armavator();

  void OnUpdate(units::second_t dt);

  //sets what infomation is needed for the states
  void SetIdle();
  void SetPosition(ArmavatorPosition pos);
  void SetZeroing();
  void SetRaw(units::volt_t arm, units::volt_t elevator);
  void SetHECalibration();

  ArmavatorPosition GetCurrentPosition() const;
  bool IsStable() const;

  ArmavatorState GetState() { return _state; }

  ArmavatorConfig &GetConfig();

  //creates the arm and the elevator
  wom::Arm *arm;
  wom::Elevator *elevator;
  ArmavatorPosition _setpoint;

  const units::centimeter_t lowerHEHeight = 0_m;
  const units::centimeter_t lowerMiddleHEHeight = 0.5_m;
  const units::centimeter_t highMiddleHEHeight = 1_m;
  const units::centimeter_t highHEHeight = 1.5_m;

 private: 
  ArmavatorState _state = ArmavatorState::kIdle;

  units::volt_t _rawArm;
  units::volt_t _rawElevator;


  //creates an instance of the gearboxes and config
  wom::Gearbox &_armGearbox;
  wom::Gearbox &_elevatorGearbox;
  ArmavatorConfig &_config;
};