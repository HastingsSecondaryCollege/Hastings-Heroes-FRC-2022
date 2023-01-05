// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/SetTopCalculatedTargetRPM.h"

SetTopCalculatedTargetRPM::SetTopCalculatedTargetRPM(ShooterSubsystem *ShooterSUB, StorageSubsystem* StorageSUB) 
: m_shooterSUB{ShooterSUB}, 
  m_storageSUB{StorageSUB}
{
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void SetTopCalculatedTargetRPM::Initialize() {
  m_shooterSUB->SetTopMotorState(true);
  if(m_storageSUB->IsFirstBallOurAllianceColor())
  {
    m_shooterSUB->SetTopMotorTargetSpeed(m_shooterSUB->CalculateTopMotorTargetSpeed());
  }
  else
  {
    m_shooterSUB->SetTopMotorTargetSpeed(kTopOpponentBallSpeed);
  }
}



// Called repeatedly when this Command is scheduled to run
void SetTopCalculatedTargetRPM::Execute() {}

// Called once the command ends or is interrupted.
void SetTopCalculatedTargetRPM::End(bool interrupted) {}

// Returns true when the command should end.
bool SetTopCalculatedTargetRPM::IsFinished() {
  return true;
}
