// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/StorageSubsystem.h"
#include "commands/Intake/EjectIntakeAndLoader.h"
#include "commands/Intake/EjectIntakeAndLoaderAndFeeder.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class EjectOneBall
    : public frc2::CommandHelper<frc2::CommandBase, EjectOneBall> {
 public:
  EjectOneBall(IntakeSubsystem* IntakeSUB, StorageSubsystem* StorageSUB, EjectIntakeAndLoader* EjectIntakeAndLoaderCMDGRP, EjectIntakeAndLoaderAndFeeder* EjectIntakeAndLoaderAndFeederCMDGRP);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;
  
  IntakeSubsystem* m_intakeSUB;
  StorageSubsystem* m_storageSUB;

  EjectIntakeAndLoader* m_ejectIntakeAndLoaderCMDGRP;
  EjectIntakeAndLoaderAndFeeder* m_ejectIntakeAndLoaderAndFeederCMDGRP;
};
