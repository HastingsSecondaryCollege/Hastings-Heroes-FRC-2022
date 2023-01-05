// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>
#include "subsystems\ShooterSubsystem.h"
#include "subsystems\StorageSubsystem.h"
#include "subsystems\DriveSubsystem.h"
#include "subsystems\LimelightSubsystem.h"
#include "commands\SetBottomCalculatedTargetRPM.h"
#include "commands\SetTopCalculatedTargetRPM.h"
#include "commands\WaitUntilShooterSpeedsOnTarget.h"
#include "commands\SetReadyToShoot.h"
#include "commands\SetFeedMotorSpeed.h"
#include "commands\SetLoaderMotorSpeed.h"
#include "commands\SetLoaderMotorTargetSpeed.h"
#include "commands\WaitUntilBallCountZero.h"
#include "commands\WaitUntilLastBallClearofShooter.h"
#include "commands\DisableTopMotor.h"
#include "commands\DisableBottomMotor.h"
#include "commands\WaitUntilBallCountMatchesTarget.h"
#include "commands\SetDistanceToTarget.h"


class CalcShoot
    : public frc2::CommandHelper<frc2::SequentialCommandGroup, CalcShoot> {
 public:
  CalcShoot(ShooterSubsystem* ShooterSUB, StorageSubsystem* StorageSUB, DriveSubsystem* DriveSUB); 
 private:
  ShooterSubsystem * m_shooterSUB;
  StorageSubsystem * m_storageSUB;
  DriveSubsystem* m_driveSUB;
  std::function<double()> m_bottomTargetSpeedFunction;
  std::function<double()> m_topTargetSpeedFunction;
};
