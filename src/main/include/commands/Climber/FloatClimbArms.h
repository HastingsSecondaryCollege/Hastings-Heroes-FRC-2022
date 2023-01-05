// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/ClimberSubsystem.h"

/**
 * Takes the air pressure off both sides of the air cyclinders that control the forward climb arms.
 * Without air pressure on either side the arm can freely move forwards and backwards.
 */
class FloatClimbArms
    : public frc2::CommandHelper<frc2::CommandBase, FloatClimbArms> {
 public:
  FloatClimbArms(ClimberSubsystem* climberSUB);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  private:
  ClimberSubsystem* m_climberSUB;
};
