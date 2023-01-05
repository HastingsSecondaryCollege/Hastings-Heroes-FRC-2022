// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Intake/IntakeDriveInwards.h"

IntakeDriveInwards::IntakeDriveInwards(IntakeSubsystem* subsystem) : m_intakeSS(subsystem) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(subsystem);
}

// Called when the command is initially scheduled.
void IntakeDriveInwards::Initialize() {
  //fmt::print("IntakeDriveInwards Command Initialized.\nThis call ->IntakeRollersIn()\n");
m_intakeSS->IntakeRollersIn();
}

// Called repeatedly when this Command is scheduled to run
void IntakeDriveInwards::Execute() {}

// Called once the command ends or is interrupted.
void IntakeDriveInwards::End(bool interrupted) {}

// Returns true when the command should end.
bool IntakeDriveInwards::IsFinished() {
  return true;
}
