// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Climber/FloatClimbArms.h"

FloatClimbArms::FloatClimbArms(ClimberSubsystem* climberSUB) : m_climberSUB(climberSUB) {
  // Use addRequirements() here to declare subsystem dependencies.
  // AddRequirements(climberSUB);
}

// Called when the command is initially scheduled.
void FloatClimbArms::Initialize() {
  m_climberSUB->ArmInFloat();
  m_climberSUB->ArmOutFloat();
}

// Called repeatedly when this Command is scheduled to run
void FloatClimbArms::Execute() {}

// Called once the command ends or is interrupted.
void FloatClimbArms::End(bool interrupted) {}

// Returns true when the command should end.
bool FloatClimbArms::IsFinished() {
  return true;
}
