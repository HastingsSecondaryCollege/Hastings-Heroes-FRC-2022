// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>
#include "commands/Intake/IntakeOut.h"
#include "commands/Intake/IntakeDriveInwards.h"
#include "subsystems/StorageSubsystem.h"
#include "commands/SetLoaderMotorTargetSpeed.h"

class IntakeOutAndIntakeDriveInwards
    : public frc2::CommandHelper<frc2::SequentialCommandGroup,
                                 IntakeOutAndIntakeDriveInwards> {
 public:
  IntakeOutAndIntakeDriveInwards(IntakeSubsystem* IntakeSUB, StorageSubsystem* StorageSUB, double LoaderTargetSpeed);

 private:
  IntakeSubsystem* m_intakeSUB;
  StorageSubsystem* m_storageSUB;
  double m_loaderTargetSpeed;
};
