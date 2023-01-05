// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ResetOdometryLimelight.h"
#include <math.h>
using namespace DriveConstants;
using namespace LimeConstants;



ResetOdometryLimelight::ResetOdometryLimelight(DriveSubsystem *subsystem,LimelightSubsystem *limelightSUB)
 : m_driveSubsystem(subsystem),
   m_limelightSUB(limelightSUB)
{ 
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void ResetOdometryLimelight::Initialize() {
  m_limelightSUB->m_isTargetCheckFinished = false;
  //m_limelightSUB->LEDState(true);
  fmt::print("Init of ResetOdometryLimelight - rotate to hub based on limelight\n");
  iterationCounter=0;
  m_driveSubsystem->ResetThetaController(units::radian_t(m_driveSubsystem->GetHeading()));
  m_driveSubsystem->SetLimelightAngleToHub(
        frc::Rotation2d(m_driveSubsystem->GetHeading()).Degrees().value() -
        m_limelightSUB->GetHorizontalTargetAngle());
  fmt::print("setting angle to hub; GetHeading(): {} - LLHorizTargetAngle: {} \n",
                    frc::Rotation2d(m_driveSubsystem->GetHeading()).Degrees().value(),
                    m_limelightSUB->GetHorizontalTargetAngle());
  m_driveSubsystem->SetActiveTrackMode(true);
  //fmt::print("About to call SetLockedHeadingMode(true) from inside ResetOdomLimeLight\n");
  m_driveSubsystem->SetLockedHeadingMode(true);
  //fmt::print("About to call SetlimeLightTrackMode(true) from inside ResetOdomLimeLight\n");
  m_driveSubsystem->SetLimelightTrackMode(true);
  //fmt::print("After call to SetlimeLightTrackMode(true) from inside ResetOdomLimeLight\n");
  m_quitLLTimer.Reset();
  m_quitLLTimer.Start();

}

// Called repeatedly when this Command is scheduled to run
void ResetOdometryLimelight::Execute() {
  iterationCounter++;
  if ((iterationCounter%10==0) &&      //every 0.2sec (5x per second)
      (iterationCounter<=50)   &&      //max 5 attempts in 
      m_limelightSUB->CheckTarget()){  //only if we still have target in sight
  
    m_driveSubsystem->SetLimelightAngleToHub(
        frc::Rotation2d(m_driveSubsystem->GetHeading()).Degrees().value() -
        m_limelightSUB->GetHorizontalTargetAngle());
    fmt::print("setting angle to hub; GetHeading(): {} - LLHorizTargetAngle: {} \n",
                    frc::Rotation2d(m_driveSubsystem->GetHeading()).Degrees().value(),
                    m_limelightSUB->GetHorizontalTargetAngle());
    fmt::print("***Limelight track attempt {}.\n",iterationCounter/10); 
    m_driveSubsystem->GetLockedHeadingMode();
    m_driveSubsystem->GetLimelightTrackMode();
  }     
}

// Called once the command ends or is interrupted.
void ResetOdometryLimelight::End(bool interrupted) {
  
  m_driveSubsystem->SetActiveTrackMode(false);
  m_driveSubsystem->SetLockedHeadingMode(false);
  m_driveSubsystem->SetLimelightTrackMode(false);
  m_quitLLTimer.Stop();
  m_quitLLTimer.Reset();
  
  if (m_limelightSUB->CheckTarget() && 
     (abs(m_limelightSUB->GetHorizontalTargetAngle()) < kShootAngleTolerance)) {
       m_limelightSUB->m_isTargetCheckFinished = true;
    frc::Pose2d llPose = m_limelightSUB->GetXYPos();
    auto velocity = m_driveSubsystem->GetVelocity();
    fmt::print("***velocity: {}\n",velocity);
    if ((velocity > -0.5_mps)||(velocity < 0.5_mps)) {
       fmt::print("Limelight Odometry Reset Completed x: {} y: {} rot: {}\n",
                llPose.X().value(),llPose.Y().value(),llPose.Rotation().Degrees().value());
      // m_driveSubsystem->ForcedResetOdometry(llPose);
    } else if ((velocity > -1.0_mps)||(velocity < 1.0_mps)) {  
      fmt::print("Limelight Odometry Reset - Since moving slowly, it resulted in a HIGH trust update to odometry ");
      //m_driveSubsystem->UpdateOdometryVision(llPose,kTrustHIGH);
    } else { 
      fmt::print("Limelight Odometry Reset - Since moving a bit fast, it resulted in a MEDIUM trust update to odometry ");
      //m_driveSubsystem->UpdateOdometryVision(llPose,kTrustMEDIUM);
    }
  }
  //m_limelightSUB->LEDState(false);
}

// Returns true when the command should end.
bool ResetOdometryLimelight::IsFinished() {
 if (m_limelightSUB->CheckTarget() && 
      (abs(m_limelightSUB->GetHorizontalTargetAngle()) < kShootAngleTolerance)){
    fmt::print("Limelight Tracking reached the target.\n");
    return true;
  }

  if (m_quitLLTimer.Get() > kQuitSeconds) {
    fmt::print("TIMEOUT: Limelight tracking took too long to reach target.\n");
    fmt::print("Horizontal Angle still at {}\n",m_limelightSUB->GetHorizontalTargetAngle());
    fmt::print("LimeLight Setpoint to hub at {}\n",m_driveSubsystem->GetSetLimelightAngleToHub());
    return true;
  }

  if (!m_limelightSUB->CheckTarget()) {
    return true;
    fmt::print("Now Target Found in ResetOdometryLimelight\n");
  }
  else
    return false;     
}