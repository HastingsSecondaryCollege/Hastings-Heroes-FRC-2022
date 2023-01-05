// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/InstantCommand.h>

#include "subsystems/LimelightSubsystem.h"

class StopShootingSequenceOnNoTarget
    : public frc2::CommandHelper<frc2::CommandBase,
                                 StopShootingSequenceOnNoTarget> {
 public:
  StopShootingSequenceOnNoTarget(LimelightSubsystem* subsystem);

  void Initialize() override;
  
  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

private:
LimelightSubsystem* m_LimelightSubsystem;

};
