// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Intake/IntakeInWaitThenStop.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
IntakeInWaitThenStop::IntakeInWaitThenStop(IntakeSubsystem* IntakeSUB, units::time::second_t WaitSeconds, StorageSubsystem* StorageSUB)
: m_intakeSUB{IntakeSUB},
m_waitSeconds{WaitSeconds},
m_storageSUB{StorageSUB}
 {
  // Add your commands here, e.g.
  // AddCommands(FooCommand(), BarCommand());
  AddCommands(
    IntakeIn(m_intakeSUB),
    WaitForNumberOfSeconds(m_waitSeconds),
    IntakeStop(m_intakeSUB),
    SetLoaderMotorSpeed(m_storageSUB, 0)
  );
}
