// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/WaitForNumberOfSeconds.h"

WaitForNumberOfSeconds::WaitForNumberOfSeconds( units::time::second_t waitSeconds) 
: m_waitSeconds {waitSeconds}
 {
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void WaitForNumberOfSeconds::Initialize() {
  m_waitTimer.Reset();
  m_waitTimer.Start();
}

// Called repeatedly when this Command is scheduled to run
void WaitForNumberOfSeconds::Execute() {}

// Called once the command ends or is interrupted.
void WaitForNumberOfSeconds::End(bool interrupted) {
  m_waitTimer.Stop();
  m_waitTimer.Reset();
}

// Returns true when the command should end.
bool WaitForNumberOfSeconds::IsFinished() {
  return (m_waitTimer.Get() >= m_waitSeconds);
}
