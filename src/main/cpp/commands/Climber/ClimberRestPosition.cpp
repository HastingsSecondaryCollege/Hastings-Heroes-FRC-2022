// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Climber/ClimberRestPosition.h"

ClimberRestPosition::ClimberRestPosition(ClimberSubsystem* ClimbSUB)
: m_climbSUB{ClimbSUB}
 {
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void ClimberRestPosition::Initialize() {
  m_climbSUB->MotionMagicToPosition(kClimbStartPosition);
}

// Called repeatedly when this Command is scheduled to run
void ClimberRestPosition::Execute() {}

// Called once the command ends or is interrupted.
void ClimberRestPosition::End(bool interrupted) {}

// Returns true when the command should end.
bool ClimberRestPosition::IsFinished() {
return m_climbSUB->HasMotionMagicFinished();
}
