// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/IgnoreColorSensor.h"

IgnoreColorSensor::IgnoreColorSensor(StorageSubsystem* storageSubsystem)
:m_storageSubsystem {storageSubsystem}
 {
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void IgnoreColorSensor::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void IgnoreColorSensor::Execute() {}

// Called once the command ends or is interrupted.
void IgnoreColorSensor::End(bool interrupted) {}

// Returns true when the command should end.
bool IgnoreColorSensor::IsFinished() {
  return false;
}
