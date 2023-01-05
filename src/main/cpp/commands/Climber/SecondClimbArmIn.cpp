// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Climber/SecondClimbArmIn.h"

SecondClimbArmIn::SecondClimbArmIn(ClimberSubsystem * ClimberSUB)
:m_ClimberSUB{ClimberSUB} {
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void SecondClimbArmIn::Initialize() {
  m_ClimberSUB->ArmIn();
}

// Called repeatedly when this Command is scheduled to run
void SecondClimbArmIn::Execute() {}

// Called once the command ends or is interrupted.
void SecondClimbArmIn::End(bool interrupted) {}

// Returns true when the command should end.
bool SecondClimbArmIn::IsFinished() {
  return true;
}
