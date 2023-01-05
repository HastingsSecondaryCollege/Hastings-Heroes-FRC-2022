// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/WaitUntilBallCountMatchesTarget.h"

WaitUntilBallCountMatchesTarget::WaitUntilBallCountMatchesTarget(StorageSubsystem *storageSUB)
    : m_storageSUB{storageSUB}
{
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void WaitUntilBallCountMatchesTarget::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void WaitUntilBallCountMatchesTarget::Execute() {}

// Called once the command ends or is interrupted.
void WaitUntilBallCountMatchesTarget::End(bool interrupted) {}

// Returns true when the command should end.
bool WaitUntilBallCountMatchesTarget::IsFinished()
{
  return (m_storageSUB->GetBallCount() == m_storageSUB -> GetTargetBallCount());
}
