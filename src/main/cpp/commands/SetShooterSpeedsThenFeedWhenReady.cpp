// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/SetShooterSpeedsThenFeedWhenReady.h"
#include "frc2/command/PrintCommand.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
SetShooterSpeedsThenFeedWhenReady::SetShooterSpeedsThenFeedWhenReady(ShooterSubsystem* ShooterSUB, StorageSubsystem* StorageSUB, std::function<double()> TopTargetSpeedFunction, std::function<double()> BottomTargetSpeedFunction)
: m_shooterSUB{ShooterSUB},
  m_storageSUB{StorageSUB},
  m_topTargetSpeedFunction{std::move(TopTargetSpeedFunction)}, 
  m_bottomTargetSpeedFunction{std::move(BottomTargetSpeedFunction)} 

  {
  // Add your commands here, e.g.
  // AddCommands(FooCommand(), BarCommand());
  AddCommands(
    //frc2::PrintCommand("About to call SetBottomMotorTargetSpeed"),
    SetBottomMotorTargetSpeed(m_shooterSUB, m_bottomTargetSpeedFunction), // [this] {return kBottomFenderLowSpeed;}) // m_bottomTargetSpeedFunction
      //frc2::PrintCommand("Finished SetBottomMotorTargetSpeed.\n About to Call SetTopMotorTargetSpeed"),
    SetTopMotorTargetSpeed(m_shooterSUB, m_topTargetSpeedFunction), // [this] {return kTopFenderLowSpeed;}) // m_topTargetSpeedFunction
      //frc2::PrintCommand("Finished SetTopMotorTargetSpeed.\n About to call WaitUntilShooterSpeedsOnTarget"),
    WaitUntilShooterSpeedsOnTarget(m_shooterSUB, m_storageSUB),
      //frc2::PrintCommand("Finished WaitUntilShooterSpeedsOnTarget.\n About to call SetFeedMotorSpeed"),
    // SetReadyToShoot(m_storageSUB, true), // This is done by the WaitUntilShooterSpeedsOnTarget command
    SetFeedMotorSpeed(m_storageSUB, kFeederMotorPower),
      //frc2::PrintCommand("Finished SetFeedMotorSpeed. \n About to Call SetLoaderMotorTargetSpeed"),
    //SetLoaderMotorSpeed(m_storageSUB, kLoaderMotorPower) // Moving away from percent power to PID control
    SetLoaderMotorTargetSpeed(m_storageSUB, kLoaderMotorTestSpeed),
      //frc2::PrintCommand("Finished SetLoaderMotorTargetSpeed. \n About to Call WaitUntilBallCountMatchesTarget"),
    WaitUntilBallCountMatchesTarget(m_storageSUB),
      //frc2::PrintCommand("Finished WaitUntilBallCountMatchesTarget. \n About to call WaitUntilLastBallClearofShooter"),
    WaitUntilLastBallClearofShooter(m_shooterSUB),
      //frc2::PrintCommand("Finished WaitUntilLastBallClearofShooter. \n About to call SetReadyToShoot"),
    SetReadyToShoot(m_storageSUB, false),
      //frc2::PrintCommand("Finished SetReadyToShoot. \n About to call DisableTopMotor"),
    DisableTopMotor(m_shooterSUB),
      //frc2::PrintCommand("Finished DisableTopMotor. \n About to call DisableBottomMotor"),
    DisableBottomMotor(m_shooterSUB),
      //frc2::PrintCommand("Finished DisableBottomMotor. \n About to call SetFeedMotorSpeed"),
    SetFeedMotorSpeed(m_storageSUB, 0.0),
      //frc2::PrintCommand("Finished SetFeedMotorSpeed. \n About to call SetLoaderMotorSpeed"),
    SetLoaderMotorSpeed(m_storageSUB, 0.0)
      //frc2::PrintCommand("Finished SetLoaderMotorSpeed. \n Now SetShooterSpeedsThenFeedWhenReady Group should be finished")
  );

}
