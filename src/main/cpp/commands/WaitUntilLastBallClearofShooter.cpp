// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/WaitUntilLastBallClearofShooter.h"

WaitUntilLastBallClearofShooter::WaitUntilLastBallClearofShooter(ShooterSubsystem* ShooterSUB) 
: m_shooterSUB {ShooterSUB}
{
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void WaitUntilLastBallClearofShooter::Initialize() {
  m_shooterSUB ->SetBottomPositionWhenBallCountZero();
}

// Called repeatedly when this Command is scheduled to run
void WaitUntilLastBallClearofShooter::Execute() {}

// Called once the command ends or is interrupted.
void WaitUntilLastBallClearofShooter::End(bool interrupted) {}

// Returns true when the command should end.
bool WaitUntilLastBallClearofShooter::IsFinished() {
  return (m_shooterSUB ->GetBottomPosition() <= (m_shooterSUB ->GetBottomPositionWhenBallCountZero() - kShooterExitUnitsper100ms)); // Less than or equal to with minus because shooter bottom spins in negative direction
}
