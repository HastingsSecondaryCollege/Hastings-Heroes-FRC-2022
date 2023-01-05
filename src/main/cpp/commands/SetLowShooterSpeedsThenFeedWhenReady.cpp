// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/SetLowShooterSpeedsThenFeedWhenReady.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
SetLowShooterSpeedsThenFeedWhenReady::SetLowShooterSpeedsThenFeedWhenReady(ShooterSubsystem* ShooterSUB, StorageSubsystem* StorageSUB) // , std::function<double()> BottomTargetSpeedFunction, std::function<double()> TopTargetSpeedFunction
: m_shooterSUB{ShooterSUB},
  m_storageSUB{StorageSUB}//,
//  m_bottomTargetSpeedFunction{BottomTargetSpeedFunction},
//  m_topTargetSpeedFunction{TopTargetSpeedFunction}

 {
  // Add your commands here, e.g.
  // AddCommands(FooCommand(), BarCommand());
  AddCommands(
    SetBottomMotorTargetSpeed(m_shooterSUB, [this] {return kBottomFenderLowSpeed;}), // [this] {return kBottomFenderLowSpeed;}) // m_bottomTargetSpeedFunction
    SetTopMotorTargetSpeed(m_shooterSUB, [this] {return kTopFenderLowSpeed;}), // [this] {return kTopFenderLowSpeed;}) // m_topTargetSpeedFunction
    WaitUntilShooterSpeedsOnTarget(m_shooterSUB, m_storageSUB),
    // SetReadyToShoot(m_storageSUB, true), // This is done by the WaitUntilShooterSpeedsOnTarget command
    SetFeedMotorSpeed(m_storageSUB, kFeederMotorPower),
    //SetLoaderMotorSpeed(m_storageSUB, kLoaderMotorPower) // Moving away from percent power to PID control
    SetLoaderMotorTargetSpeed(m_storageSUB, kLoaderMotorTestSpeed),
    WaitUntilBallCountMatchesTarget(m_storageSUB),
    WaitUntilLastBallClearofShooter(m_shooterSUB),
    SetReadyToShoot(m_storageSUB, false),
    DisableTopMotor(m_shooterSUB),
    DisableBottomMotor(m_shooterSUB),
    SetFeedMotorSpeed(m_storageSUB, 0.0),
    SetLoaderMotorSpeed(m_storageSUB, 0.0)
    
  );
}
