#pragma once
#include "VoltageController.h"
#include "Arm.h"
#include "Elevator.h"
#include "Armavator.h"
#include "Gyro.h"
// #include "behaviour/SingleSwerveBehaviour.h"
#include "behaviour/ArmavatorBehaviour.h"

#include <ctre/phoenix/motorcontrol/can/WPI_TalonFX.h>
#include <frc/Compressor.h>
#include <frc/XboxController.h>
#include <ctre/Phoenix.h>

#include "drivetrain/SwerveDrive.h"
#include <frc/DoubleSolenoid.h>
// #include "SwerveMod.h"
#include <units/length.h>

#include <iostream>
#include <string>

struct RobotMap {
  struct Controllers {  
    frc::XboxController driver{0};
    frc::XboxController codriver{1};
  };
  Controllers controllers;

  // struct Armavator {
  //   static constexpr units::kilogram_t loadMass = 10_kg;
  //   static constexpr units::kilogram_t armMass = 5_kg;
  //   static constexpr units::kilogram_t carriageMass = 5_kg;

  //   struct Arm {
  //     WPI_TalonSRX motor1{1};
  //     WPI_TalonSRX motor2{2};

  //     wom::MotorVoltageController motor = wom::MotorVoltageController::Group(motor1, motor2);

  //     wom::DigitalEncoder encoder{0, 1, 2048};

  //     wom::Gearbox gearbox {
  //       &motor,
  //       &encoder,
  //       wom::DCMotor::CIM(2).WithReduction(100)
  //     };
  //     wom::ArmConfig config {
  //       "/armavator/arm",
  //       gearbox,
  //       nullptr,
  //       nullptr,
  //       {
  //         "/armavator/arm/pid/config",
  //         12_V / 45_deg
  //       },
  //       armMass, loadMass, 1_m,
  //       -90_deg, 270_deg,
  //       -90_deg
  //     };
  //   };
  //   Arm arm;

  //   struct Elevator {
  //     WPI_TalonSRX motor1{3};
  //     WPI_TalonSRX motor2{4};

  //     wom::MotorVoltageController motor = wom::MotorVoltageController::Group(motor1, motor2);

  //     wom::DigitalEncoder encoder{2, 3, 2048};

  //     wom::Gearbox gearbox {
  //       &motor,
  //       &encoder,
  //       wom::DCMotor::CIM(2).WithReduction(10)
  //     };

  //     wom::ElevatorConfig config {
  //       "/armavator/elevator",
  //       gearbox,
  //       nullptr,
  //       nullptr,
  //       2_in,
  //       armMass + loadMass + carriageMass,
  //       1.5_m,
  //       1_m,
  //       {
  //         "/armavator/elevator/pid/config",
  //         12_V / 1_m
  //       }
  //     };
  //   };
  //   Elevator elevator;

  //   ArmavatorConfig::grid_t occupancyGrid = ArmavatorConfig::grid_t(
  //     arm.config.minAngle, arm.config.maxAngle,
  //     0_m, elevator.config.maxHeight,
  //     50, 50
  //   ).FillF([this](units::radian_t angle, units::meter_t height) {
  //     units::meter_t x = arm.config.armLength * units::math::cos(angle);
  //     units::meter_t y = height + arm.config.armLength * units::math::sin(angle);
  //     return !(y >= 0.1_m && y <= 6_ft);
  //   });

  //   ArmavatorConfig config {
  //     arm.config, elevator.config, occupancyGrid
  //   };
  // };
  // Armavator armavator;

  struct SwerveBase{
    wom::NavX gyro;
    wpi::array<WPI_TalonFX*, 4> turnMotors{
      new WPI_TalonFX(90), new WPI_TalonFX(91), new WPI_TalonFX(92), new WPI_TalonFX(93)
    };
    wpi::array<WPI_TalonFX*, 4> driveMotors{
      new WPI_TalonFX(94), new WPI_TalonFX(95), new WPI_TalonFX(96), new WPI_TalonFX(97)
    };
    //   wpi::array<WPI_TalonFX*, 4> turnMotors{
    //   new WPI_TalonFX(1), new WPI_TalonFX(3), new WPI_TalonFX(4), new WPI_TalonFX(6)
    // };
    // wpi::array<WPI_TalonFX*, 4> driveMotors{
    //   new WPI_TalonFX(5), new WPI_TalonFX(8), new WPI_TalonFX(2), new WPI_TalonFX(7)
    // };
    
    wpi::array<wom::SwerveModuleConfig, 4> moduleConfigs{
      wom::SwerveModuleConfig{ // dimensions are assuming perfect square robot 1m^2 area
        frc::Translation2d(0.5_m, 0.5_m),
        wom::Gearbox{
          new wom::MotorVoltageController(driveMotors[0]),
          new wom::TalonFXEncoder(driveMotors[0], 6.75),
          wom::DCMotor::Falcon500(1).WithReduction(6.75)
        },
        wom::Gearbox{
          new wom::MotorVoltageController(turnMotors[0]),
          new wom::TalonFXEncoder(turnMotors[0], 12.8),
          wom::DCMotor::Falcon500(1).WithReduction(12.8)
        },
        4_in / 2
      },
      wom::SwerveModuleConfig{ // dimensions are assuming perfect square robot 1m^2 area
        frc::Translation2d(0.5_m, -0.5_m),
        wom::Gearbox{
          new wom::MotorVoltageController(driveMotors[1]),
          new wom::TalonFXEncoder(driveMotors[1], 6.75),
          wom::DCMotor::Falcon500(1).WithReduction(6.75)
        },
        wom::Gearbox{
          new wom::MotorVoltageController(turnMotors[1]),
          new wom::TalonFXEncoder(turnMotors[1], 12.8),
          wom::DCMotor::Falcon500(1).WithReduction(12.8)
        },
        4_in / 2
      },
      wom::SwerveModuleConfig{ // dimensions are assuming perfect square robot 1m^2 area
        frc::Translation2d(-0.5_m, 0.5_m),
        wom::Gearbox{
          new wom::MotorVoltageController(driveMotors[2]),
          new wom::TalonFXEncoder(driveMotors[2], 6.75),
          wom::DCMotor::Falcon500(1).WithReduction(6.75)
        },
        wom::Gearbox{
          new wom::MotorVoltageController(turnMotors[2]),
          new wom::TalonFXEncoder(turnMotors[2], 12.8),
          wom::DCMotor::Falcon500(1).WithReduction(12.8)
        },
        4_in / 2
      },
      wom::SwerveModuleConfig{ // dimensions are assuming perfect square robot 1m^2 area
        frc::Translation2d(-0.5_m, -0.5_m),
        wom::Gearbox{
          new wom::MotorVoltageController(driveMotors[3]),
          new wom::TalonFXEncoder(driveMotors[3], 6.75),
          wom::DCMotor::Falcon500(1).WithReduction(6.75)
        },
        wom::Gearbox{
          new wom::MotorVoltageController(turnMotors[3]),
          new wom::TalonFXEncoder(turnMotors[3], 12.8),
          wom::DCMotor::Falcon500(1).WithReduction(12.8)
        },
        4_in / 2
      },
    };

    wom::SwerveModule::angle_pid_conf_t anglePID {
      "/drivetrain/pid/angle/config",
      10.5_V / 180_deg,
      0.75_V / (100_deg * 1_s),
      0_V / (100_deg / 1_s),
      1_deg,
      0.5_deg / 1_s

      // 1_rad
    };
    wom::SwerveModule::velocity_pid_conf_t velocityPID{
      "/drivetrain/pid/velocity/config",
      // -1_V / 8_mps
    };
    wom::SwerveDriveConfig::pose_angle_conf_t poseAnglePID {
      "/drivetrain/pid/pose/angle/config",
      180_deg / 1_s / 45_deg,
      wom::SwerveDriveConfig::pose_angle_conf_t::ki_t{0},
      0_deg / 1_deg,
      1_deg,
      10_deg / 1_s
    };
    wom::SwerveDriveConfig::pose_position_conf_t posePositionPID{
      "/drivetrain/pid/pose/position/config",
      3_mps / 1_m,
      wom::SwerveDriveConfig::pose_position_conf_t::ki_t{0},
      0_m / 1_m,
      5_cm, 
      10_cm / 1_s
    };

    wom::SwerveDriveConfig config{
      "/drivetrain",
      anglePID, velocityPID,
      moduleConfigs,// each module
      &gyro,
      poseAnglePID, 
      posePositionPID,
      10_kg // robot mass (estimate rn)
    };  

    SwerveBase() {
      for (size_t i = 0; i < 4; i++) {
        turnMotors[i]->ConfigSupplyCurrentLimit(SupplyCurrentLimitConfiguration(true, 15, 15, 0));
      }
    }
  };

  struct SwerveGridPoses { // positions to place the items
    frc::Pose2d innerGrid1 = frc::Pose2d(1_m, 1_m, 0_deg); // Closest grid position to the Wall
    frc::Pose2d innerGrid2 = frc::Pose2d(1_m, 2_m, 0_deg); // Middle of Inner Grid
    frc::Pose2d innerGrid3 = frc::Pose2d(1_m, 3_m, 0_deg); // Centremost Inner Grid position
    frc::Pose2d centreGrid1 = frc::Pose2d(1_m, 4_m, 0_deg); // The non central grid on the Inner Grid side
    frc::Pose2d centreGrid2 = frc::Pose2d(1_m, 5_m, 0_deg); // The middle most grid 
    frc::Pose2d centreGrid3 = frc::Pose2d(1_m, 6_m, 0_deg); // The non central grid on the Outer Grid side
    frc::Pose2d outerGrid1 = frc::Pose2d(1_m, 7_m, 0_deg); // Centremost outer grid position
    frc::Pose2d outerGrid2 = frc::Pose2d(1_m, 8_m, 0_deg); // Middle of Outer Grid
    frc::Pose2d outerGrid3 = frc::Pose2d(1_m, 9_m, 0_deg); // Closest grid position to enemy Loading Zone
  };

  // struct SwerveSingleModule{
  //   SwerveSingleModuleConfig config{
  //     WPI_TalonFX(1),
  //     WPI_TalonFX(2)
  //   };
  // };
  
  SwerveBase swerveBase;
  SwerveGridPoses swerveGridPoses;
  //SwerveSingleModule swerveSingleModuleMotors;

  struct IntakeSystem {
    WPI_TalonSRX rightMotor{4};
    WPI_VictorSPX leftMotor{6};

    // wom::MotorVoltageController rightIntake = wom::MotorVoltageController::Group(rightMotor);
    // wom::TalonSr

    // wom::Gearbox intakeGearbox  {
    //   &rightIntake,
    //   &
    // };


    frc::Compressor compressor{1, frc::PneumaticsModuleType::CTREPCM};
    frc::DoubleSolenoid leftSolenoid{1, frc::PneumaticsModuleType::CTREPCM, 0, 1};
    // frc::DoubleSolenoid rightSolenoid{PneumaticsModuleType::CTREPCM, 2};

  }; IntakeSystem intake;
  struct Armavator {
    static constexpr units::kilogram_t loadMass = 10_kg;
    static constexpr units::kilogram_t armMass = 5_kg;
    static constexpr units::kilogram_t carriageMass = 5_kg;

    struct Arm {
      WPI_TalonSRX motor{10};

      wom::MotorVoltageController motorGroup = wom::MotorVoltageController::Group(motor);
      
      wom::DigitalEncoder encoder{0, 1, 2048};

      wom::Gearbox gearbox {
        &motorGroup,
        &encoder,
        wom::DCMotor::CIM(2).WithReduction(100)
      };

      wom::ArmConfig config {
        "/armavator/arm",
        gearbox,
        wom::PIDConfig<units::radian, units::volts>("/armavator/arm/pid/config")
      };
    };
    Arm arm;

    struct Elevator {
      WPI_TalonSRX motor{9};

      wom::MotorVoltageController motorGroup = wom::MotorVoltageController::Group(motor);

      wom::DigitalEncoder encoder{2, 3, 2048};

      wom::Gearbox gearbox {
        &motorGroup,
        &encoder,
        wom::DCMotor::CIM(2).WithReduction(10)
      };

      wom::ElevatorConfig config {
        "/armavator/elevator",
        gearbox,
        nullptr,
        nullptr,
        2_in,
        armMass + loadMass + carriageMass,
        1.5_m,
        1_m,
        {
          "/armavator/elevator/pid/config",
          12_V / 1_m
        }
      };
    };
    Elevator elevator;

    ArmavatorConfig::grid_t occupancyGrid = ArmavatorConfig::grid_t(
      arm.config.minAngle, arm.config.maxAngle,
      0_m, elevator.config.maxHeight,
      50, 50
    ).FillF([this](units::radian_t angle, units::meter_t height) {
      units::meter_t x = arm.config.armLength * units::math::cos(angle);
      units::meter_t y = height + arm.config.armLength * units::math::sin(angle);
      return !(y >= 0.1_m && y <= 6_ft);
    });

    ArmavatorConfig config {
      arm.config, elevator.config, occupancyGrid
    };
  }; Armavator armavator;

  // struct SwerveBase{
  //   wom::NavX gyro;
  //   wpi::array<WPI_TalonFX*, 4> turnMotors{
  //     new WPI_TalonFX(1), new WPI_TalonFX(2), new WPI_TalonFX(3), new WPI_TalonFX(4)
  //   };
  //   wpi::array<WPI_TalonFX*, 4> driveMotors{
  //     new WPI_TalonFX(5), new WPI_TalonFX(6), new WPI_TalonFX(7), new WPI_TalonFX(8)
  //   };
    
  //   wpi::array<wom::SwerveModuleConfig, 4> moduleConfigs{
  //     wom::SwerveModuleConfig{ // dimensions are assuming perfect square robot 1m^2 area
  //       frc::Translation2d(0.5_m, 0.5_m),
  //       wom::Gearbox{
  //         new wom::MotorVoltageController(driveMotors[0]),
  //         new wom::TalonFXEncoder(driveMotors[0]),
  //         wom::DCMotor::Falcon500(1).WithReduction(6.75)
  //       },
  //       wom::Gearbox{
  //         new wom::MotorVoltageController(turnMotors[0]),
  //         new wom::TalonFXEncoder(turnMotors[0]),
  //         wom::DCMotor::Falcon500(1).WithReduction(12.8)
  //       },
  //       4_in / 2
  //     },
  //     wom::SwerveModuleConfig{ // dimensions are assuming perfect square robot 1m^2 area
  //       frc::Translation2d(0.5_m, -0.5_m),
  //       wom::Gearbox{
  //         new wom::MotorVoltageController(driveMotors[1]),
  //         new wom::TalonFXEncoder(driveMotors[1]),
  //         wom::DCMotor::Falcon500(1).WithReduction(6.75)
  //       },
  //       wom::Gearbox{
  //         new wom::MotorVoltageController(turnMotors[1]),
  //         new wom::TalonFXEncoder(turnMotors[1]),
  //         wom::DCMotor::Falcon500(1).WithReduction(12.8)
  //       },
  //       4_in / 2
  //     },
  //     wom::SwerveModuleConfig{ // dimensions are assuming perfect square robot 1m^2 area
  //       frc::Translation2d(-0.5_m, 0.5_m),
  //       wom::Gearbox{
  //         new wom::MotorVoltageController(driveMotors[2]),
  //         new wom::TalonFXEncoder(driveMotors[2]),
  //         wom::DCMotor::Falcon500(1).WithReduction(6.75)
  //       },
  //       wom::Gearbox{
  //         new wom::MotorVoltageController(turnMotors[2]),
  //         new wom::TalonFXEncoder(turnMotors[2]),
  //         wom::DCMotor::Falcon500(1).WithReduction(12.8)
  //       },
  //       4_in / 2
  //     },
  //     wom::SwerveModuleConfig{ // dimensions are assuming perfect square robot 1m^2 area
  //       frc::Translation2d(-0.5_m, -0.5_m),
  //       wom::Gearbox{
  //         new wom::MotorVoltageController(driveMotors[3]),
  //         new wom::TalonFXEncoder(driveMotors[3]),
  //         wom::DCMotor::Falcon500(1).WithReduction(6.75)
  //       },
  //       wom::Gearbox{
  //         new wom::MotorVoltageController(turnMotors[3]),
  //         new wom::TalonFXEncoder(turnMotors[3]),
  //         wom::DCMotor::Falcon500(1).WithReduction(12.8)
  //       },
  //       4_in / 2
  //     },
  //   };

  //   wom::SwerveModule::angle_pid_conf_t anglePID {
  //     "/drivetrain/pid/angle/config",
  //     12_V / 90_deg
  //   };
  //   wom::SwerveModule::velocity_pid_conf_t velocityPID{
  //     "/drivetrain/pid/velocity/config",
  //     12_V / 2_mps
  //   };
  //   wom::SwerveDriveConfig::pose_angle_conf_t poseAnglePID {
  //     "/drivetrain/pid/pose/angle/config",
  //     180_deg / 1_s / 45_deg,
  //     wom::SwerveDriveConfig::pose_angle_conf_t::ki_t{0},
  //     0_deg / 1_deg,
  //     1_deg,
  //     10_deg / 1_s
  //   };
  //   wom::SwerveDriveConfig::pose_position_conf_t posePositionPID{
  //     "/drivetrain/pid/pose/position/config",
  //     3_mps / 1_m,
  //     wom::SwerveDriveConfig::pose_position_conf_t::ki_t{0},
  //     0_m / 1_m,
  //     5_cm, 
  //     10_cm / 1_s
  //   };

  //   wom::SwerveDriveConfig config{
  //     "/drivetrain",
  //     anglePID, velocityPID,
  //     moduleConfigs,// each module
  //     &gyro,
  //     poseAnglePID, 
  //     posePositionPID,
  //     70_kg // robot mass (estimate rn)
  //   }; 

  //   // SwerveBase swerveBase;
  // };
};