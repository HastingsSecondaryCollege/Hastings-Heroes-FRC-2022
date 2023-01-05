// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Climber/ExtendClimber.h"

 

ExtendClimber::ExtendClimber(ClimberSubsystem* climberSUB) : m_climberSUB(climberSUB) {
  // Use addRequirements() here to declare subsystem dependencies.
  // AddRequirements(climberSUB);
}

// Called when the command is initially scheduled.
void ExtendClimber::Initialize() {
  m_climberSUB->MotionMagicToPosition(kFullExtendEncoderPosition);
}

// Called repeatedly when this Command is scheduled to run
void ExtendClimber::Execute() {}

// Called once the command ends or is interrupted.
void ExtendClimber::End(bool interrupted) {}

// Returns true when the command should end.
bool ExtendClimber::IsFinished() {
  return m_climberSUB->HasMotionMagicFinished();
}
