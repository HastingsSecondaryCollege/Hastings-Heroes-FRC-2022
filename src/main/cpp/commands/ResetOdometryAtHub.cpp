// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ResetOdometryAtHub.h"
using namespace DriveConstants;


ResetOdometryAtHub::ResetOdometryAtHub(DriveSubsystem *subsystem) : m_driveSubsystem(subsystem) { 
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void ResetOdometryAtHub::Initialize() {
   //Stop the drivetrain by setting the speed to zero in all axis
  m_driveSubsystem->Drive(units::meters_per_second_t(0),
                          units::meters_per_second_t(0),
                          units::radians_per_second_t(0), false);
  m_driveSubsystem->ForcedResetOdometry(frc::Pose2d(kHubResetX, kHubResetY, frc::Rotation2d(units::angle::degree_t(kHubResetRotation))));
}

// Called repeatedly when this Command is scheduled to run
void ResetOdometryAtHub::Execute() {}

// Called once the command ends or is interrupted.
void ResetOdometryAtHub::End(bool interrupted) {}

// Returns true when the command should end.
bool ResetOdometryAtHub::IsFinished() {
  return true;
}
