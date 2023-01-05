// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/TrackToHub.h"

TrackToHub::TrackToHub(DriveSubsystem* DriveSUB)
: m_driveSUB{DriveSUB}
 {
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void TrackToHub::Initialize() {
  m_driveSUB->ResetThetaController(units::radian_t(m_driveSUB->GetHeading()));
  m_driveSUB->SetActiveTrackMode(true);
  m_driveSUB->SetLockedHeadingMode(true);
  fmt::print("Init of TrackToHub\n");
}

// Called repeatedly when this Command is scheduled to run
void TrackToHub::Execute() {}

// Called once the command ends or is interrupted.
void TrackToHub::End(bool interrupted) {}

// Returns true when the command should end.
bool TrackToHub::IsFinished() {
  return true;
}
