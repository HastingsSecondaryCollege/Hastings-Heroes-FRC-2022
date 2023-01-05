// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/IncrementTopMotorTargetSpeed.h"

IncrementTopMotorTargetSpeed::IncrementTopMotorTargetSpeed(ShooterSubsystem *subsystem, double direction, double increment) 
 : m_subsystem{subsystem},
  m_direction{direction},
  m_increment{increment} {
    // AddRequirements({subsystem}); // We will only use AddRequirements statements if required
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void IncrementTopMotorTargetSpeed::Initialize() {
   m_subsystem->SetTopMotorState(true);
m_subsystem->IncrementTopMotorTargetSpeed(m_direction, m_increment);
}

// Called repeatedly when this Command is scheduled to run
void IncrementTopMotorTargetSpeed::Execute() {}

// Called once the command ends or is interrupted.
void IncrementTopMotorTargetSpeed::End(bool interrupted) {}

// Returns true when the command should end.
bool IncrementTopMotorTargetSpeed::IsFinished() {
  return true;
}
