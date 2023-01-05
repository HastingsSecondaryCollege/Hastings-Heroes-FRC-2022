// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/IncrementBothMotorsPercentOutput.h"

IncrementBothMotorsPercentOutput::IncrementBothMotorsPercentOutput(ShooterSubsystem *subsystem, double direction, double increment) 
: m_subsystem{subsystem},
  m_direction{direction},
  m_increment{increment} 
{
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void IncrementBothMotorsPercentOutput::Initialize() {
  m_subsystem->IncrementBothShooterMotorsPercentOutput(m_direction, m_increment);
}

// Called repeatedly when this Command is scheduled to run
void IncrementBothMotorsPercentOutput::Execute() {}

// Called once the command ends or is interrupted.
void IncrementBothMotorsPercentOutput::End(bool interrupted) {}

// Returns true when the command should end.
bool IncrementBothMotorsPercentOutput::IsFinished() {
  return true;
}
