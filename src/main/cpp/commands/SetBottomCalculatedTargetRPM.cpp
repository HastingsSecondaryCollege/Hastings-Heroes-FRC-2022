// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/SetBottomCalculatedTargetRPM.h"

SetBottomCalculatedTargetRPM::SetBottomCalculatedTargetRPM(ShooterSubsystem *ShooterSUB, StorageSubsystem *StorageSUB) 
: m_shooterSUB{ShooterSUB},
  m_storageSUB{StorageSUB}
{
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void SetBottomCalculatedTargetRPM::Initialize() {
  m_shooterSUB->SetBottomMotorState(true);
  if(m_storageSUB->IsFirstBallOurAllianceColor())
  {
    m_shooterSUB->SetBottomMotorTargetSpeed(m_shooterSUB->CalculateBottomMotorTargetSpeed());  
    // frc::SmartDashboard::PutString("Opponent Color", "No");
  }
  else
  {
    m_shooterSUB->SetBottomMotorTargetSpeed(kBottomOpponentBallSpeed);
    // frc::SmartDashboard::PutString("Opponent Color", "Yes");
  }
  
}

// Called repeatedly when this Command is scheduled to run
void SetBottomCalculatedTargetRPM::Execute() {}

// Called once the command ends or is interrupted.
void SetBottomCalculatedTargetRPM::End(bool interrupted) {
   if (interrupted) fmt::print("Set BottomCalculatedTarget has been interrupted\n");

}

// Returns true when the command should end.
bool SetBottomCalculatedTargetRPM::IsFinished() {
  return true;
}
