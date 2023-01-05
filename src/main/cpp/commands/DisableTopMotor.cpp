// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/DisableTopMotor.h"

DisableTopMotor::DisableTopMotor(ShooterSubsystem *subsystem)
    : m_subsystem{subsystem}
{
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({m_subsystem}); // This command will interrupt other commans that require the ShooterSubsystem
}

// Called when the command is initially scheduled.
void DisableTopMotor::Initialize()
{
  m_subsystem->SetTopMotorState(false);
  //m_subsystem->SetTopMotorTargetSpeed(0.0);
  m_subsystem->DisableShooter();
}

// Called repeatedly when this Command is scheduled to run
void DisableTopMotor::Execute() {}

// Called once the command ends or is interrupted.
void DisableTopMotor::End(bool interrupted) {}

// Returns true when the command should end.
bool DisableTopMotor::IsFinished()
{
  return true;
}
