// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Climber/ClimbFull.h"

ClimbFull::ClimbFull(ClimberSubsystem * ClimberSUB)
:m_ClimberSUB{ClimberSUB}
 {
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void ClimbFull::Initialize() {
  m_ClimberSUB->MotionMagicToPosition(kFullRetractEncoderPosition);
}

// Called repeatedly when this Command is scheduled to run
void ClimbFull::Execute() {}

// Called once the command ends or is interrupted.
void ClimbFull::End(bool interrupted) {}

// Returns true when the command should end.
bool ClimbFull::IsFinished() {
  return m_ClimberSUB->HasMotionMagicFinished();
}
