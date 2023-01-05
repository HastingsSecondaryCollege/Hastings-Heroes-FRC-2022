// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/DriveFixedFieldBearing.h"

DriveFixedFieldBearing::DriveFixedFieldBearing(DriveSubsystem* subsystem, units::radian_t fixedHeading) {
  //Set the local member variable to the passed in parameters
  m_DriveSubsystem = subsystem;
  m_fixedRobotHeading = fixedHeading;

  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void DriveFixedFieldBearing::Initialize() {
  m_DriveSubsystem->SetLockedHeadingAngle(m_fixedRobotHeading);
  m_DriveSubsystem->ResetThetaController(units::radian_t(m_DriveSubsystem->GetHeading()));
  m_DriveSubsystem->SetLockedHeadingMode(true);
  ////MDE20220117 migrate 2022 
  //std::cout <<"Initialized DriveFixedFieldBearing " << m_fixedRobotHeading.value() << "\n";
  //fmt::print("Initialized DriveFixedFieldBearing {}\n",m_fixedRobotHeading.value());
}

// Called repeatedly when this Command is scheduled to run
void DriveFixedFieldBearing::Execute() {}

// Called once the command ends or is interrupted.
void DriveFixedFieldBearing::End(bool interrupted) {}

// Returns true when the command should end.
bool DriveFixedFieldBearing::IsFinished() {
  return true;
}
