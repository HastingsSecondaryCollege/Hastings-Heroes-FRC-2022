// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/SetDistanceToTargetFromLimelight.h"

SetDistanceToTargetFromLimelight::SetDistanceToTargetFromLimelight(LimelightSubsystem* LimelightSUB, ShooterSubsystem* ShooterSUB)
: m_limelightSUB{LimelightSUB},
  m_shooterSUB{ShooterSUB}
 {
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void SetDistanceToTargetFromLimelight::Initialize() {
  if (m_limelightSUB->CheckTarget())
  {
    frc::SmartDashboard::PutNumber("Get Distance For Shooter", (m_limelightSUB->GetDistanceForShooter()*1000));
    m_shooterSUB->SetDistanceToTarget(m_limelightSUB->GetDistanceForShooter()*1000); // GetDistanceForShooter returns metres, SetDistanceToTarget takes millimetres
  }
  else {
    fmt::print("No valid Limelight Target\nLeaving Distance at Previuos Set Value.\n");
  }
  
}

// Called repeatedly when this Command is scheduled to run
void SetDistanceToTargetFromLimelight::Execute() {}

// Called once the command ends or is interrupted.
void SetDistanceToTargetFromLimelight::End(bool interrupted) {
  if (interrupted) fmt::print("Set Distance to target has been interrupted\n");
}

// Returns true when the command should end.
bool SetDistanceToTargetFromLimelight::IsFinished() {
  return true;
}
