// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/StorageSubsystem.h"
#include "Constants.h"


class EjectIntakeAndLoaderAndFeeder
    : public frc2::CommandHelper<frc2::SequentialCommandGroup,
                                 EjectIntakeAndLoaderAndFeeder> {
 public:
  EjectIntakeAndLoaderAndFeeder(IntakeSubsystem* IntakeSUB, StorageSubsystem* StorageSUB, units::time::second_t WaitTime);

  private:
    IntakeSubsystem * m_intakeSUB;
    StorageSubsystem * m_storageSUB;
    units::time::second_t m_waitTime;
};
