// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/Phoenix.h>
#include <frc/DoubleSolenoid.h>
#include <frc/PneumaticsControlModule.h>
#include "Constants.h"
#include <frc/Timer.h>

using namespace IntakeConstants;

class IntakeSubsystem : public frc2::SubsystemBase
{
public:
  IntakeSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */

  void Periodic() override;

  void OpenIntake();

  void CloseIntake();

  void IntakeRollersIn();

  void IntakeRollersOut();

  void IntakeRollerStop();

      private :
      // Components (e.g. motor controllers and sensors) should generally be
      // declared private and exposed only through public methods.

       TalonFX m_intakeMotor{kCANIDIntakeMotor, std::string{kRoboRIOCANBusName}};
      frc::DoubleSolenoid m_intakeSolenoid;

     frc::Timer m_intakeEjectTimer;
};
