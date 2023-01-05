// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems\ClimberSubsystem.h"

/**
 * Drives the climb hooks to almost fully down.  This is the position that the climb hooks start in.
 * IE the hhoks nearly fully down with the black line on the cog inline with the polycarb behind it.
 */
class ClimberRestPosition
    : public frc2::CommandHelper<frc2::CommandBase, ClimberRestPosition> {
 public:
  ClimberRestPosition(ClimberSubsystem* ClimbSUB);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  private:
    ClimberSubsystem* m_climbSUB;
};
