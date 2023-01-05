// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/StopTrackToHub.h"

StopTrackToHub::StopTrackToHub(DriveSubsystem* DriveSUB) 
: m_driveSUB{DriveSUB}
{
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void StopTrackToHub::Initialize() {
  m_driveSUB->SetLockedHeadingMode(false);
  m_driveSUB->SetActiveTrackMode(false);
}

// Called repeatedly when this Command is scheduled to run
void StopTrackToHub::Execute() {}

// Called once the command ends or is interrupted.
void StopTrackToHub::End(bool interrupted) {}

// Returns true when the command should end.
bool StopTrackToHub::IsFinished() {
  return true;
}
