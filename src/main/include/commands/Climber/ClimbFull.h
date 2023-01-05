// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/ClimberSubsystem.h"
/**
 * This is position that climb hooks pull down to when climbing. NB: This command is relying on some
 * tension being on the hooks, because if it run without tension the climb cord bundles loosy around
 * the spindle in the intial part of the retraction and subsequential cause the rope to become very 
 * tight when the spindle has turns the correct number of turns to fully retract the hooks
 */
class ClimbFull
    : public frc2::CommandHelper<frc2::CommandBase, ClimbFull> {
 public:
  ClimbFull(ClimberSubsystem *ClimberSUB);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  private:

  ClimberSubsystem *m_ClimberSUB;  
};
