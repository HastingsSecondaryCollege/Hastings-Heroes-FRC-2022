// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Intake/EjectOneBall.h"

EjectOneBall::EjectOneBall(IntakeSubsystem* IntakeSUB, StorageSubsystem* StorageSUB, EjectIntakeAndLoader* EjectIntakeAndLoaderCMDGRP, EjectIntakeAndLoaderAndFeeder* EjectIntakeAndLoaderAndFeederCMDGRP)
: m_intakeSUB{IntakeSUB},
  m_storageSUB{StorageSUB},
  m_ejectIntakeAndLoaderCMDGRP{EjectIntakeAndLoaderCMDGRP},
  m_ejectIntakeAndLoaderAndFeederCMDGRP{EjectIntakeAndLoaderAndFeederCMDGRP}

 {
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void EjectOneBall::Initialize() {
  if (m_storageSUB -> GetBallCount() < 2)
  {
    m_ejectIntakeAndLoaderAndFeederCMDGRP->Schedule();
  }
  else
  {
    m_ejectIntakeAndLoaderCMDGRP->Schedule();
  }
}

// Called repeatedly when this Command is scheduled to run
void EjectOneBall::Execute() {}

// Called once the command ends or is interrupted.
void EjectOneBall::End(bool interrupted) {}

// Returns true when the command should end.
bool EjectOneBall::IsFinished() {
  return true;
}
