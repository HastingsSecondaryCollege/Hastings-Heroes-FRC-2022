// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/SetLoaderMotorTargetPosition.h"

SetLoaderMotorTargetPosition::SetLoaderMotorTargetPosition(StorageSubsystem *StorageSUB) 
  : m_StorageSUB{StorageSUB}
{
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void SetLoaderMotorTargetPosition::Initialize() {
  m_StorageSUB->SetLoaderMotorTargetPosition();
}

// Called repeatedly when this Command is scheduled to run
void SetLoaderMotorTargetPosition::Execute() {}

// Called once the command ends or is interrupted.
void SetLoaderMotorTargetPosition::End(bool interrupted) {}

// Returns true when the command should end.
bool SetLoaderMotorTargetPosition::IsFinished() {
  return true;
}
