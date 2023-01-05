// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/SetLoaderMotorSpeed.h"

SetLoaderMotorSpeed::SetLoaderMotorSpeed(StorageSubsystem *subsystem, double LoaderSpeed)
  : m_subsystem{subsystem},
    m_loaderSpeed{(LoaderSpeed)} {
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void SetLoaderMotorSpeed::Initialize() {
     m_subsystem->SetLoaderMotorPower(m_loaderSpeed);
}


// Called repeatedly when this Command is scheduled to run
void SetLoaderMotorSpeed::Execute() {}

// Called once the command ends or is interrupted.
void SetLoaderMotorSpeed::End(bool interrupted) {}

// Returns true when the command should end.
bool SetLoaderMotorSpeed::IsFinished() {
  return true;
}
