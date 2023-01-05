// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/SetDistanceToTarget.h"

SetDistanceToTarget::SetDistanceToTarget(DriveSubsystem* DriveSUB, ShooterSubsystem* ShooterSUB)
: m_driveSUB{DriveSUB},
  m_shooterSUB{ShooterSUB}
 {
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void SetDistanceToTarget::Initialize() {
  m_shooterSUB->SetDistanceToTarget((m_driveSUB->GetDistanceToHUB()*1000)); // GetDistanceToHUB returns metres, Set DistanceToTarget, takes millimetres
}

// Called repeatedly when this Command is scheduled to run
void SetDistanceToTarget::Execute() {}

// Called once the command ends or is interrupted.
void SetDistanceToTarget::End(bool interrupted) {}

// Returns true when the command should end.
bool SetDistanceToTarget::IsFinished() {
  return true;
}
