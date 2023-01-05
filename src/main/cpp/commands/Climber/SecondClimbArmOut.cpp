// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Climber/SecondClimbArmOut.h"

SecondClimbArmOut::SecondClimbArmOut(ClimberSubsystem * ClimberSUB) 
:m_ClimberSUB{ClimberSUB} {
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void SecondClimbArmOut::Initialize() {
  //fmt::print("Arm Out Command");
  m_ClimberSUB->ArmOut();
}

// Called repeatedly when this Command is scheduled to run
void SecondClimbArmOut::Execute() {}

// Called once the command ends or is interrupted.
void SecondClimbArmOut::End(bool interrupted) {}

// Returns true when the command should end.
bool SecondClimbArmOut::IsFinished() {
  return true;
}
