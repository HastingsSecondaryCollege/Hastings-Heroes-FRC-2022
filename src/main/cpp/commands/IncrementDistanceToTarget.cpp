// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/IncrementDistanceToTarget.h"

IncrementDistanceToTarget::IncrementDistanceToTarget(ShooterSubsystem *shooterSub, double direction, double increment) 
: m_shooterSUB{shooterSub},
  m_direction{direction},
  m_increment{increment} 
  {
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void IncrementDistanceToTarget::Initialize() {
m_shooterSUB->SetTopMotorState(true);
m_shooterSUB->SetBottomMotorState(true);
m_shooterSUB->IncrementDistanceToTarget(m_direction, m_increment);
//frc::SmartDashboard::PutNumber("Distance Increment Direction", m_direction);
//frc::SmartDashboard::PutNumber("Distance Increment Step", m_increment);
}

// Called repeatedly when this Command is scheduled to run
void IncrementDistanceToTarget::Execute() {}

// Called once the command ends or is interrupted.
void IncrementDistanceToTarget::End(bool interrupted) {}

// Returns true when the command should end.
bool IncrementDistanceToTarget::IsFinished() {
  return true;
}
