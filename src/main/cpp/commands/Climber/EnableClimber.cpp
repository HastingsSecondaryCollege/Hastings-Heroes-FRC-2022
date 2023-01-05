// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Climber/EnableClimber.h"

EnableClimber::EnableClimber(ClimberSubsystem * ClimberSUB, DriveSubsystem* DriveSUB)
:m_climberSUB(ClimberSUB),
m_driveSUB{DriveSUB}

{
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void EnableClimber::Initialize() {
//fmt::print("calling m_climberSUB->EnableClimber();");
m_climberSUB->EnableClimber();
//fmt::print("m_driveSUB->SetDriveSpeedMultiplier(kDriveSpeedMultiplierWhileClimbing);");
m_driveSUB->SetDriveSpeedMultiplier(kDriveSpeedMultiplierWhileClimbing);
//fmt::print("Calling m_climberSUB->ArmOut();");
m_climberSUB->ArmOut();

m_climberSUB->MotionMagicToPosition(kStartClimbHookHeightPosition);
// -------FIXME-----need to add pointer to intake and open intake
//-------FIXME-----need to add pointer to drive and scale speed
}

// Called repeatedly when this Command is scheduled to run
void EnableClimber::Execute() {}

// Called once the command ends or is interrupted.
void EnableClimber::End(bool interrupted) {}

// Returns true when the command should end.
bool EnableClimber::IsFinished() {
  return true;
}
