// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/IncrementShooterAdjustments.h"

IncrementShooterAdjustments::IncrementShooterAdjustments(ShooterSubsystem* ShooterSUB, double Direction, double Increment) 
: m_shooterSUB{ShooterSUB},
m_direction{Direction},
m_increment{Increment}
{
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void IncrementShooterAdjustments::Initialize() {
  m_shooterSUB->IncrementShooterAdjustments(m_direction, m_increment);
}

// Called repeatedly when this Command is scheduled to run
void IncrementShooterAdjustments::Execute() {}

// Called once the command ends or is interrupted.
void IncrementShooterAdjustments::End(bool interrupted) {}

// Returns true when the command should end.
bool IncrementShooterAdjustments::IsFinished() {
  return true;
}
