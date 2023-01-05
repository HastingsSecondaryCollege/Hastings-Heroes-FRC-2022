// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/IncrementBottomMotorTargetSpeed.h"


IncrementBottomMotorTargetSpeed::IncrementBottomMotorTargetSpeed(ShooterSubsystem *subsystem, double direction, double increment)
: m_subsystem{subsystem},
  m_direction{direction},
  m_increment{increment} {
  // Use addRequirements() here to declare subsystem dependencies.
  // AddRequirements({subsystem}); // We will only use AddRequirements statements if required
}

// Called when the command is initially scheduled.
void IncrementBottomMotorTargetSpeed::Initialize() {
   m_subsystem->SetBottomMotorState(true);
m_subsystem->IncrementBottomMotorTargetSpeed(m_direction, m_increment);
}

// Called repeatedly when this Command is scheduled to run
void IncrementBottomMotorTargetSpeed::Execute() {}

// Called once the command ends or is interrupted.
void IncrementBottomMotorTargetSpeed::End(bool interrupted) {}

// Returns true when the command should end.
bool IncrementBottomMotorTargetSpeed::IsFinished() {
  return true;
}
