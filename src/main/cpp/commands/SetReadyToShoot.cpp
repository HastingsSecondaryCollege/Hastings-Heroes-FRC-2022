// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/SetReadyToShoot.h"
//#include <frc/smartdashboard/SmartDashboard.h>

SetReadyToShoot::SetReadyToShoot(StorageSubsystem *subsystem, bool ReadyToShoot) 
: m_subsystem{subsystem},
      m_readyToShoot{(ReadyToShoot)}  {
  // Use addRequirements() here to declare subsystem dependencies.
  // AddRequirements({subsystem}); // We will only use AddRequirements statements if required
}

// Called when the command is initially scheduled.
void SetReadyToShoot::Initialize() {
  m_subsystem->SetReadyToShoot(m_readyToShoot);
  //frc::SmartDashboard::PutBoolean("Ready To Shoot", m_readyToShoot);
}

// Called repeatedly when this Command is scheduled to run
void SetReadyToShoot::Execute() {}

// Called once the command ends or is interrupted.
void SetReadyToShoot::End(bool interrupted) {}

// Returns true when the command should end.
bool SetReadyToShoot::IsFinished() {
  return true;
}
