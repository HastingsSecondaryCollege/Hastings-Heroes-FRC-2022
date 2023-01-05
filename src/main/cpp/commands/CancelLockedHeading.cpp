// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/CancelLockedHeading.h"

CancelLockedHeading::CancelLockedHeading(DriveSubsystem *subsystem)
    : m_driveSubsystem{subsystem}
{
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void CancelLockedHeading::Initialize()
{
  m_driveSubsystem->SetLockedHeadingMode(false);
    //fmt::print("Init of CancelledLockedHeading\n");
}

// Called repeatedly when this Command is scheduled to run
void CancelLockedHeading::Execute() {}

// Called once the command ends or is interrupted.
void CancelLockedHeading::End(bool interrupted) {}

// Returns true when the command should end.
bool CancelLockedHeading::IsFinished()
{
  return true;
}
