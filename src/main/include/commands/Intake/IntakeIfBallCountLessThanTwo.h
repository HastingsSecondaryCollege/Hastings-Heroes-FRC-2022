// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/StorageSubsystem.h"
#include "subsystems/IntakeSubsystem.h"
#include "commands/Intake/IntakeOutAndIntakeDriveInwards.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class IntakeIfBallCountLessThanTwo
    : public frc2::CommandHelper<frc2::CommandBase, IntakeIfBallCountLessThanTwo> {
 public:
  IntakeIfBallCountLessThanTwo(StorageSubsystem* StorageSUB, IntakeSubsystem* IntakeSUB, IntakeOutAndIntakeDriveInwards* IntakeOutAndIntakeDriveInwardsCMDGRP);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  private:

    StorageSubsystem* m_storageSUB;
    IntakeSubsystem* m_intakeSUB;
    IntakeOutAndIntakeDriveInwards* m_intakeOutAndIntakeDriveInwardsCMDGRP;
};
