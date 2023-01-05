// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Intake/IntakeIfBallCountLessThanTwo.h"

IntakeIfBallCountLessThanTwo::IntakeIfBallCountLessThanTwo(StorageSubsystem* StorageSUB, IntakeSubsystem* IntakeSUB, IntakeOutAndIntakeDriveInwards* IntakeOutAndIntakeDriveInwardsCMDGRP)
: m_storageSUB{StorageSUB},
  m_intakeSUB{IntakeSUB},
  m_intakeOutAndIntakeDriveInwardsCMDGRP{IntakeOutAndIntakeDriveInwardsCMDGRP}


 {
  // Use addRequirements() here to declare subsystem dependencies.

}

// Called when the command is initially scheduled.
void IntakeIfBallCountLessThanTwo::Initialize() {
  if (m_storageSUB -> GetBallCount() < 2)
  {
    m_intakeOutAndIntakeDriveInwardsCMDGRP->Schedule();
  }
}

// Called repeatedly when this Command is scheduled to run
void IntakeIfBallCountLessThanTwo::Execute() {}

// Called once the command ends or is interrupted.
void IntakeIfBallCountLessThanTwo::End(bool interrupted) {}

// Returns true when the command should end.
bool IntakeIfBallCountLessThanTwo::IsFinished() {
  return true;
}
