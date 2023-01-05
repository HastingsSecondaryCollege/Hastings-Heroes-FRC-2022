// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/DriveSubsystem.h"
#include "pathplanner/lib/PathPlannerTrajectory.h"

/**
 * Command to Initialize the odometry to a custom Translation2d (position) 
 * and Rotation2D (Holonomic Angle)
 */
class InitializeOdometry
    : public frc2::CommandHelper<frc2::CommandBase, InitializeOdometry> {
 public:
  InitializeOdometry(DriveSubsystem* subsystem, frc::Translation2d position, frc::Rotation2d angle);
  //InitializeOdometry(DriveSubsystem* subsystem, pathplanner::PathPlannerTrajectory* pointerToTrajectory); 

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
    DriveSubsystem* m_driveSubsystem;
    frc::Translation2d m_translation2D;     // Holds the Position of the robot
    frc::Rotation2d m_rotation2D;           // Holds the holonomic Heading of the robot
};  
