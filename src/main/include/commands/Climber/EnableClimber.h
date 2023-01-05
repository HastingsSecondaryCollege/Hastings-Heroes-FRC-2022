// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/ClimberSubsystem.h"
#include "subsystems/DriveSubsystem.h"

/**
 * This command starts the point of no return for climbing.  Once this command runs
 * the arms go forward, releasing the pin holding them down and then they extend above 
 * the max height allowed outside the hanger.
 */
class EnableClimber
    : public frc2::CommandHelper<frc2::CommandBase, EnableClimber> {
 public:
  EnableClimber(ClimberSubsystem * ClimberSUB, DriveSubsystem* DriveSUB);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  private:
  ClimberSubsystem * m_climberSUB;
  DriveSubsystem* m_driveSUB;
};

