// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Intake/IntakeStop.h"

IntakeStop::IntakeStop(IntakeSubsystem* subsystem) : m_intakeSS(subsystem) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(subsystem);
}

// Called when the command is initially scheduled.
void IntakeStop::Initialize() {
m_intakeSS->IntakeRollerStop();
}

// Called repeatedly when this Command is scheduled to run
void IntakeStop::Execute() {}

// Called once the command ends or is interrupted.
void IntakeStop::End(bool interrupted) {}

// Returns true when the command should end.
bool IntakeStop::IsFinished() {
  return true;
}
