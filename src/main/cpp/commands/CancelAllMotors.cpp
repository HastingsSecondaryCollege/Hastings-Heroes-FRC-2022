// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/CancelAllMotors.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
CancelAllMotors::CancelAllMotors(ShooterSubsystem* ShooterSUB, StorageSubsystem* StorageSUB, IntakeSubsystem* IntakeSUB)
: m_shooterSUB{ShooterSUB},
  m_storageSUB{StorageSUB},
  m_intakeSUB{IntakeSUB}
 {
  // Add your commands here, e.g.
  // AddCommands(FooCommand(), BarCommand());
  AddCommands(
    DisableBottomMotor(m_shooterSUB),
    DisableTopMotor(m_shooterSUB),
    SetLoaderMotorSpeed(m_storageSUB, 0),
    SetFeedMotorSpeed(m_storageSUB, 0),
    IntakeStop(m_intakeSUB)
  );
}
