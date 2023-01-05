// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Intake/IntakeDriveOutwards.h"

IntakeDriveOutwards::IntakeDriveOutwards(IntakeSubsystem* subsystem) : m_intakeSS(subsystem) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(subsystem);
}

// Called when the command is initially scheduled.
void IntakeDriveOutwards::Initialize() {
m_intakeSS->IntakeRollersOut();
}

// Called repeatedly when this Command is scheduled to run
void IntakeDriveOutwards::Execute() {}

// Called once the command ends or is interrupted.
void IntakeDriveOutwards::End(bool interrupted) {}

// Returns true when the command should end.
bool IntakeDriveOutwards::IsFinished() {
  return true;
}
