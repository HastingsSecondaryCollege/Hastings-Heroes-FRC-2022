// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/WaitUntilShooterSpeedsOnTarget.h"

WaitUntilShooterSpeedsOnTarget::WaitUntilShooterSpeedsOnTarget(ShooterSubsystem *shooterSUB, StorageSubsystem *storageSUB)
    : m_shooterSUB{shooterSUB},
    m_storageSUB{storageSUB}
{
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({shooterSUB}); // DisableBottomMotor and DisableTopMotor commands will interrupt this command
}

// Called when the command is initially scheduled.
void WaitUntilShooterSpeedsOnTarget::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void WaitUntilShooterSpeedsOnTarget::Execute() {}

// Called once the command ends or is interrupted.
void WaitUntilShooterSpeedsOnTarget::End(bool interrupted) {
  if (interrupted) {
    m_storageSUB->SetReadyToShoot(false);
    //frc::SmartDashboard::PutBoolean("Ready To Shoot", false);
  }
  else {
    m_storageSUB->SetReadyToShoot(true);
    //frc::SmartDashboard::PutBoolean("Ready To Shoot", true);
  }
}

// Returns true when the command should end.
bool WaitUntilShooterSpeedsOnTarget::IsFinished()
{
  return m_shooterSUB->ShooterSpeedsOnTarget();
}

