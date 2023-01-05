// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/Phoenix.h>
#include <frc/DoubleSolenoid.h>
#include <frc/PneumaticsControlModule.h>
#include "Constants.h"
#include <frc/Joystick.h>
#include "subsystems/IntakeSubsystem.h"


using namespace ClimberConstants;
using namespace OIConstants;
  class ClimberSubsystem : public frc2::SubsystemBase
// namespace ClimberConstants

{
public:
  ClimberSubsystem(frc::Joystick * ClimberButtonBoard, IntakeSubsystem* IntakeSUB);

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 void MotionMagicToPosition(int ClimbPosition);
 void KillClimbMotor();
 void ArmOut();
 void ArmOutFloat();
 void ArmIn();
 void ArmInFloat();
 void StopClimbArm();
 void EnableClimber();

bool HasMotionMagicFinished();

void SetClimbMaxVelAndAcc(int MaxVelocity, int MaxAcceleration);

void FastClimbMode();
void SlowClimbMode();
void SetClimbMotorToBrakeMode();
void SetClimbMotorToCoastMode();

void CloseIntake();
void OpenIntake();

private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  TalonFX m_climberMotor{kCANIDClimberMotor, std::string{kRoboRIOCANBusName}};
  frc::DoubleSolenoid m_2ndArmFWD_OutSoleniod; 
  frc::DoubleSolenoid m_2ndArmBKW_InSolenoid;  
  frc::Joystick * m_climbButtonBoard;
  IntakeSubsystem* m_intakeSUB;
  int m_MotionMagicSetPointPostion;
  bool m_climberEnabled=false;
  bool m_jogDownClicked=false;
  bool m_jogUpClicked=false;
  bool m_slowClimbMode=false;
  int m_maxClimbSpeed=kMaxClimbCruiseVelocityInitPreferenceValue; //preference_MaxClimbSpeed;
  int m_maxClimbAcceleration=kMaxClimbAccelerationInitPreferenceValue; //preference_MaxClimbAcceleration;
  };
