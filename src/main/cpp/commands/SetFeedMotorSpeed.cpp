// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/smartdashboard/SmartDashboard.h>
#include "commands/SetFeedMotorSpeed.h"

SetFeedMotorSpeed::SetFeedMotorSpeed(StorageSubsystem *subsystem, double MotorSpeed)
   : m_subsystem{subsystem},
      m_feedSpeed{(MotorSpeed)}  {
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void SetFeedMotorSpeed::Initialize() 
{
   m_subsystem->SetFeedMotorPower(m_feedSpeed);
}

// Called repeatedly when this Command is scheduled to run
void SetFeedMotorSpeed::Execute() {}

// Called once the command ends or is interrupted.
void SetFeedMotorSpeed::End(bool interrupted) {}

// Returns true when the command should end.
bool SetFeedMotorSpeed::IsFinished() {
  return true;
}
