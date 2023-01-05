// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/SCurveDemo.h"
using namespace DriveConstants;
SCurveDemo::SCurveDemo(DriveSubsystem *subsystem) : m_driveSubsystem(subsystem)
{
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({subsystem});
}

// Called when the command is initially scheduled.
void SCurveDemo::Initialize()
{
  //Stop the drivetrain by setting the speed to zero in all axis
  m_driveSubsystem->Drive(units::meters_per_second_t(0),
                          units::meters_per_second_t(0),
                          units::radians_per_second_t(0), false);

  //Reset the odemetry to 0,0,0 deg
  //m_driveSubsystem->ResetOdometry(frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)));
  m_driveSubsystem->ForcedResetOdometry(frc::Pose2d(kStartX, kStartY, 
                    /*frc::Rotation2d(*/kStartRotation/*)*/ ));
}

// Called repeatedly when this Command is scheduled to run
void SCurveDemo::Execute()
{
  //                m_driveSubsystem->Drive(units::meters_per_second_t(0),
  //                          units::meters_per_second_t(0),
  //                          units::radians_per_second_t(0), false);
}

// Called once the command ends or is interrupted.
void SCurveDemo::End(bool interrupted) {}

// Returns true when the command should end.
bool SCurveDemo::IsFinished()
{
  return false;
}
