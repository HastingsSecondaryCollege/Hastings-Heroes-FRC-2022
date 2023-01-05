// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Climber/GoToDriveOnPosition.h"

GoToDriveOnPosition::GoToDriveOnPosition(ClimberSubsystem * ClimberSUB)
:m_climberSUB{ClimberSUB} {
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void GoToDriveOnPosition::Initialize() {
    m_climberSUB->MotionMagicToPosition(kStartClimbHookHeightPosition);
}

// Called repeatedly when this Command is scheduled to run
void GoToDriveOnPosition::Execute() {}

// Called once the command ends or is interrupted.
void GoToDriveOnPosition::End(bool interrupted) {}

// Returns true when the command should end.
bool GoToDriveOnPosition::IsFinished() {
    return m_climberSUB->HasMotionMagicFinished();
}
