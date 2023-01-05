// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/InitializeOdometry.h"

InitializeOdometry::InitializeOdometry(DriveSubsystem *subsystem, frc::Translation2d position, frc::Rotation2d angle)
    : m_driveSubsystem(subsystem),
      m_translation2D(position),
      m_rotation2D(angle)
{
  // Use addRequirements() here to declare subsystem dependencies.
  //AddRequirements({subsystem});
}
/*
InitializeOdometry:: InitializeOdometry(DriveSubsystem* subsystem, pathplanner::PathPlannerTrajectory* pointerToTrajectory)
{
  InitializeOdometry(subsystem,pointerToTrajectory->getInitialState()->pose.Translation(), pointerToTrajectory->getInitialState()->holonomicRotation);
}
*/
// Called when the command is initially scheduled.
void InitializeOdometry::Initialize()
{
  // Stop the drivetrain by setting the speed to zero in all axis
  m_driveSubsystem->Drive(units::meters_per_second_t(0),
                          units::meters_per_second_t(0),
                          units::radians_per_second_t(0), false);
  // First call to Set the gyro, it takes a while for the same value to be return on a Get
  m_driveSubsystem->SetHeading(m_rotation2D.Degrees().value());
}

// Called repeatedly when this Command is scheduled to run
void InitializeOdometry::Execute()
{
  // Keep setting the angle every loop until isfinished thinks it close enough to the same value
  m_driveSubsystem->SetHeading(m_rotation2D.Degrees().value());
}

// Called once the command ends or is interrupted.
void InitializeOdometry::End(bool interrupted) {}

// Returns true when the command should end.
bool InitializeOdometry::IsFinished()
{
  // check the difference between the Angle Set and Angle read is less than 0.1.
  if (abs(m_driveSubsystem->GetHeading().value() - m_rotation2D.Degrees().value()) < 0.1)
  {
    fmt::print("Reseting Position to x: {}, y: {}, theta: {}\n",m_translation2D.X().value(),m_translation2D.Y().value(),m_rotation2D.Degrees().value());
    // Now we can reset the odomemtry because the heading change has gone through
    m_driveSubsystem->ResetOdometry(frc::Pose2d(m_translation2D, m_rotation2D));
    return true;
  }
  else
    return false;
}
