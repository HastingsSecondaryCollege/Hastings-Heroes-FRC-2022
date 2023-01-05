// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/ClimberSubsystem.h"

/**
 * This command raises the hooks to max height.  It relies on the climber spindle being in 
 * exactly the right position during power on.
 */
class ExtendClimber
    : public frc2::CommandHelper<frc2::CommandBase, ExtendClimber> {
 public:
  ExtendClimber(ClimberSubsystem* climberSUB);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  private:
  ClimberSubsystem* m_climberSUB;
};
