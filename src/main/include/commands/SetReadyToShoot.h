// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/StorageSubsystem.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class SetReadyToShoot
    : public frc2::CommandHelper<frc2::CommandBase, SetReadyToShoot> {
 public:
  SetReadyToShoot(StorageSubsystem *subsystem, bool ReadyToShoot);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;
private:
  StorageSubsystem *m_subsystem;
  bool m_readyToShoot;
};