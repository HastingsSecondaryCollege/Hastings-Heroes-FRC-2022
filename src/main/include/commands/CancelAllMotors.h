// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>
#include "subsystems/ShooterSubsystem.h"
#include "subsystems/StorageSubsystem.h"
#include "subsystems/IntakeSubsystem.h"
#include "commands/DisableBottomMotor.h"
#include "commands/DisableTopMotor.h"
#include "commands/SetLoaderMotorSpeed.h"
#include "commands/SetFeedMotorSpeed.h"
#include "commands/Intake/IntakeStop.h"

class CancelAllMotors
    : public frc2::CommandHelper<frc2::SequentialCommandGroup,
                                 CancelAllMotors> {
 public:
  CancelAllMotors(ShooterSubsystem* ShooterSUB, StorageSubsystem* StorageSUB, IntakeSubsystem* IntakeSUB);

  private:
    ShooterSubsystem* m_shooterSUB;
    StorageSubsystem* m_storageSUB;
    IntakeSubsystem* m_intakeSUB;
};
