#pragma once 

#include <frc/SpeedController.h>
#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>
#include <functional>
#include "motorTools.h"

namespace wom {
  class TalonSrx : public frc::SpeedController {
   public: 
    using Configuration = ctre::phoenix::motorcontrol::can::TalonSRXConfiguration;
    using ControlMode = ctre::phoenix::motorcontrol::ControlMode;

    TalonSrx(tools::Port port, int encoderTicksPerRotation = 2048);
    ~TalonSrx();

    void SetUpdateRate(int hz);

    //Get Port function + get physical port 

    void SetInverted(bool invert) override;

    bool GetInverted() const override;

    void Disable() override;
    //in WML there are 2 stop motor functions, disable and stop motor 

    void Set(double speed) override;

    void Set(ControlMode mode, double value); //what

    double Get() const override;

    virtual double GetCurrent() override; 

    


   private: 
  }
}