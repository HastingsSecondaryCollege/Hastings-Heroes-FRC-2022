// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Intake/IntakeOut.h"

IntakeOut::IntakeOut(IntakeSubsystem* subsystem) : m_intakeSS(subsystem) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(subsystem);
}

// Called when the command is initially scheduled.
void IntakeOut::Initialize() {
  //fmt::print("IntakeOut command initialized.\nThis calls ->OpenIntake() on the IntakeSS.\n");
  m_intakeSS->OpenIntake();
}

// Called repeatedly when this Command is scheduled to run
void IntakeOut::Execute() {}

// Called once the command ends or is interrupted.
void IntakeOut::End(bool interrupted) {}

// Returns true when the command should end.
bool IntakeOut::IsFinished() {
  return true;
}
