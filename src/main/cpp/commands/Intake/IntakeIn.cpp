// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Intake/IntakeIn.h"

IntakeIn::IntakeIn(IntakeSubsystem* subsystem) : m_intakeSS(subsystem) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(subsystem);
  this->SetName("IntakeIn");
}

// Called when the command is initially scheduled.
void IntakeIn::Initialize() {
//fmt::print("INTAKE_IN\n");
m_intakeSS->CloseIntake();
}

// Called repeatedly when this Command is scheduled to run
void IntakeIn::Execute() {}

// Called once the command ends or is interrupted.
void IntakeIn::End(bool interrupted) {}

// Returns true when the command should end.
bool IntakeIn::IsFinished() {
  return true;
}
