// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>
#include "subsystems\ShooterSubsystem.h"
#include "subsystems\StorageSubsystem.h"
#include "commands\SetBottomMotorTargetSpeed.h"
#include "commands\SetTopMotorTargetSpeed.h"
#include "commands\WaitUntilShooterSpeedsOnTarget.h"
#include "commands\SetReadyToShoot.h"
#include "commands\SetFeedMotorSpeed.h"
#include "commands\SetLoaderMotorSpeed.h"
#include "commands\SetLoaderMotorTargetSpeed.h"
#include "commands\WaitUntilBallCountMatchesTarget.h"
#include "commands\WaitUntilLastBallClearofShooter.h"
#include "commands\DisableTopMotor.h"
#include "commands\DisableBottomMotor.h"


class SetShooterSpeedsThenFeedWhenReady
    : public frc2::CommandHelper<frc2::SequentialCommandGroup,
                                 SetShooterSpeedsThenFeedWhenReady> {
 public:
  SetShooterSpeedsThenFeedWhenReady(ShooterSubsystem* ShooterSUB, StorageSubsystem* StorageSUB, std::function<double()> TopTargetSpeedFunction, std::function<double()> BottomTargetSpeedFunction);

  private:
  ShooterSubsystem * m_shooterSUB;
  StorageSubsystem * m_storageSUB;
  std::function<double()> m_topTargetSpeedFunction;  
  std::function<double()> m_bottomTargetSpeedFunction;

};
