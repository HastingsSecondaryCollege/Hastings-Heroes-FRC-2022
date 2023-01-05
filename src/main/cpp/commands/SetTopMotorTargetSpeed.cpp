// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/SetTopMotorTargetSpeed.h"
//#include <frc/smartdashboard/SmartDashboard.h>

SetTopMotorTargetSpeed::SetTopMotorTargetSpeed(ShooterSubsystem *subsystem, std::function<double()> MotorTargetSpeed)
    : m_subsystem{subsystem},
      m_motorTargetSpeedFunction{std::move(MotorTargetSpeed)}
{
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void SetTopMotorTargetSpeed::Initialize()
{
  m_subsystem->SetTopMotorState(true);
}

// Called repeatedly when this Command is scheduled to run
void SetTopMotorTargetSpeed::Execute()
{
  m_subsystem->SetTopMotorTargetSpeed(m_motorTargetSpeedFunction());
}

// Called once the command ends or is interrupted.
void SetTopMotorTargetSpeed::End(bool interrupted) {}

// Returns true when the command should end.
bool SetTopMotorTargetSpeed::IsFinished()
{
  return true;
}
