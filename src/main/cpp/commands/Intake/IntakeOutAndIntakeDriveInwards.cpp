// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Intake/IntakeOutAndIntakeDriveInwards.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
IntakeOutAndIntakeDriveInwards::IntakeOutAndIntakeDriveInwards(IntakeSubsystem* IntakeSUB, StorageSubsystem* StorageSUB, double LoaderTargetSpeed)
: m_intakeSUB{IntakeSUB},
  m_storageSUB{StorageSUB},
  m_loaderTargetSpeed{LoaderTargetSpeed}
 {
  // Add your commands here, e.g.
  // AddCommands(FooCommand(), BarCommand());
  AddCommands(
    IntakeOut(m_intakeSUB),
    IntakeDriveInwards(m_intakeSUB),
    SetLoaderMotorTargetSpeed(m_storageSUB, m_loaderTargetSpeed)
  );
}
