// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/ClimberSubsystem.h"

/**
 * This commands drives the climb hooks to perfect position for them slide off after lowering down with
 * the climb arms hooked onto the next bar up.  The speed that you approach this height is critical as
 * you want the robot to stay attached right up until this position to control the swinging.
 * If the robot releases releases from the bar too early it will swing wildly.  This happens if you motion 
 * magic cruise velocity and acceleration are too high.
 */
class GoToSlideOffPosition
    : public frc2::CommandHelper<frc2::CommandBase, GoToSlideOffPosition> {
 public:
  GoToSlideOffPosition(ClimberSubsystem * ClimberSUB);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  private:

  ClimberSubsystem *m_climberSUB;
};
