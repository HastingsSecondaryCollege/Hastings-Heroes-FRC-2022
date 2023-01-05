// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/SetLoaderMotorTargetSpeed.h"
#include "subsystems/StorageSubsystem.h"

SetLoaderMotorTargetSpeed::SetLoaderMotorTargetSpeed(StorageSubsystem *StorageSUB, double TargetSpeed) 
  : m_StorageSUB{StorageSUB},
    m_targetSpeed{TargetSpeed}
{
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void SetLoaderMotorTargetSpeed::Initialize() {
  //frc::SmartDashboard::PutBoolean("Mismatched ball colours", m_StorageSUB->AreBallColorsDifferent());
    fmt::print("Inside SetLoaderMotorTargetSpeed Initialize.\n");
  if (!m_StorageSUB->AreBallColorsDifferent()) {
    fmt::print("    Ball colours are same\n");
    m_StorageSUB -> SetTargetBallCount(0);
    m_StorageSUB->SetLoaderMotorTargetSpeed(m_targetSpeed);
  }
  else 
  {
    fmt::print("    Ball colours are different\n");
    m_StorageSUB -> SetTargetBallCount(1);
  }
  
}

// Called repeatedly when this Command is scheduled to run
void SetLoaderMotorTargetSpeed::Execute() {}

// Called once the command ends or is interrupted.
void SetLoaderMotorTargetSpeed::End(bool interrupted) {}

// Returns true when the command should end.
bool SetLoaderMotorTargetSpeed::IsFinished() {
  return true;
}
