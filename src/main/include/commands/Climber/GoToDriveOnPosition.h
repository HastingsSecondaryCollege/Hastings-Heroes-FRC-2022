// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/ClimberSubsystem.h"

/**
 * Drives the climb hooks to the correct height for lining up with the bar at the start of climb
 * sequence.  Also a handy height for being just below max height.
 */
class GoToDriveOnPosition
    : public frc2::CommandHelper<frc2::CommandBase, GoToDriveOnPosition> {
 public:
  GoToDriveOnPosition(ClimberSubsystem * ClimberSUB);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  private:

    ClimberSubsystem *m_climberSUB;
};
