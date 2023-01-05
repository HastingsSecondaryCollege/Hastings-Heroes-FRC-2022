// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/SetCalculatedShooterSpeedsThenFeedWhenReady.h"
#include <frc2/command/PrintCommand.h>

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
SetCalculatedShooterSpeedsThenFeedWhenReady::SetCalculatedShooterSpeedsThenFeedWhenReady(ShooterSubsystem* ShooterSUB, StorageSubsystem* StorageSUB, DriveSubsystem* DriveSUB,LimelightSubsystem* LimeSUB)
: m_shooterSUB{ShooterSUB},
  m_storageSUB{StorageSUB},
  m_driveSUB{DriveSUB},
  m_limelightSUB{LimeSUB}
  
  {
  // Add your commands here, e.g.
  // AddCommands(FooCommand(), BarCommand());
  AddCommands(
    //frc2::PrintCommand("calling ResetOdometryLimelight from SetCalShoSpeeds...\n"),
    ResetOdometryLimelight(m_driveSUB,m_limelightSUB),
    // SetDistanceToTarget(m_driveSUB, m_shooterSUB), // Replaced by SetDistanceToTargetFromLimelight
    //frc2::PrintCommand("SetDistanceToTargetFromLimelight"),
    SetDistanceToTargetFromLimelight(m_limelightSUB, m_shooterSUB),
    //frc2::PrintCommand("SetBottomCalculatedTargetRPM"),
    SetBottomCalculatedTargetRPM(m_shooterSUB, m_storageSUB),
    //frc2::PrintCommand("SetTopCalculatedTargetRPM"),
    SetTopCalculatedTargetRPM(m_shooterSUB, m_storageSUB),
    //frc2::PrintCommand("WaitUntilShooterSpeedsOnTarget"),
    WaitUntilShooterSpeedsOnTarget(m_shooterSUB, m_storageSUB),
    // SetReadyToShoot(m_storageSUB, true), // This is done by the WaitUntilShooterSpeedsOnTarget command
    frc2::PrintCommand("SetFeedMotorSpeed"),
    SetFeedMotorSpeed(m_storageSUB, kFeederMotorPower),
    frc2::PrintCommand("SetLoaderMotorTargetSpeed"),
    //SetLoaderMotorSpeed(m_storageSUB, kLoaderMotorPower) // Moving away from percent power to PID control
    SetLoaderMotorTargetSpeed(m_storageSUB, kLoaderMotorTestSpeed),
    frc2::PrintCommand("WaitUntilBallCountMatchesTarget"),
    WaitUntilBallCountMatchesTarget(m_storageSUB),
    frc2::PrintCommand("WaitUntilLastBallClearofShooter"),
    WaitUntilLastBallClearofShooter(m_shooterSUB),
    frc2::PrintCommand("SetReadyToShoot"),
    SetReadyToShoot(m_storageSUB, false),
    frc2::PrintCommand("DisableTopMotor"),
    DisableTopMotor(m_shooterSUB),
    frc2::PrintCommand("DisableBottomMotor"),
    DisableBottomMotor(m_shooterSUB),
    frc2::PrintCommand("SetFeedMotorSpeed"),
    SetFeedMotorSpeed(m_storageSUB, 0.0),
    frc2::PrintCommand("SetLoaderMotorSpeed"),
    SetLoaderMotorSpeed(m_storageSUB, 0.0)
    
  );

}
