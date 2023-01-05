// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#include <units/angle.h>
#include "commands/RotateToHub.h"
using namespace DriveConstants;

RotateToHub::RotateToHub(DriveSubsystem *subsystem)
    : m_driveSubsystem{subsystem}
{
  // Use addRequirements() here to declare subsystem dependencies.
  //AddRequirements({subsystem});
}

// Called when the command is initially scheduled.
void RotateToHub::Initialize()
{
   
  //rotateToHeading = m_driveSubsystem->GetAngleToHUB().Degrees().value();
  //fmt::print("*** RotateToHub: {} degrees.  From heading: {}\n",rotateToHeading,m_driveSubsystem->GetPigeonRotation2D().Degrees().value());
  m_driveSubsystem->ResetThetaController(units::radian_t(m_driveSubsystem->GetHeading()));
  m_driveSubsystem->SetActiveTrackMode(true);
  m_driveSubsystem->SetLockedHeadingMode(true);
  fmt::print("Init of RotateToHub\n");
  m_quitTimer.Reset();
  m_quitTimer.Start();
}

// Called repeatedly when this Command is scheduled to run
void RotateToHub::Execute() {
}

// Called once the command ends or is interrupted.
void RotateToHub::End(bool interrupted) {
  m_driveSubsystem->SetLockedHeadingMode(false);
  m_driveSubsystem->SetActiveTrackMode(false);
  m_quitTimer.Stop();
  m_quitTimer.Reset();
}

// Returns true when the command should end.
bool RotateToHub::IsFinished()
{
    if (m_driveSubsystem->IsHubTurnComplete() || 
            (m_quitTimer.Get() >= kQuitSeconds))  
        return true;
    else
        return false;        
}
