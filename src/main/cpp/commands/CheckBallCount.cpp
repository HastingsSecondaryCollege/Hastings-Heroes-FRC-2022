// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/CheckBallCount.h"

CheckBallCount::CheckBallCount(StorageSubsystem *StorageSUB)
: m_StorageSUB{StorageSUB}
 {
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void CheckBallCount::Initialize() {
  m_StorageSUB -> CheckBallCount();
}

// Called repeatedly when this Command is scheduled to run
void CheckBallCount::Execute() {}

// Called once the command ends or is interrupted.
void CheckBallCount::End(bool interrupted) {}

// Returns true when the command should end.
bool CheckBallCount::IsFinished() {
  return true;
}
