// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/IntakeSubsystem.h"

// This method will be called once per scheduler run
void IntakeSubsystem::Periodic() {}

IntakeSubsystem::IntakeSubsystem()
    : m_intakeSolenoid{frc::PneumaticsModuleType::CTREPCM, kPCMPortIntakeForward, kPCMPortIntakeReverse} {
    m_intakeMotor.ConfigFactoryDefault();
    m_intakeMotor.ConfigNeutralDeadband(kNeutralDeadband);
    m_intakeMotor.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrame::Status_1_General_, 255, 0);
    m_intakeMotor.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrame::Status_2_Feedback0_, 255, 0);
    m_intakeMotor.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrame::Status_4_AinTempVbat_, 255, 0);
    m_intakeMotor.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrame::Status_6_Misc_, 255, 0);
    m_intakeMotor.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrame::Status_7_CommStatus_, 255, 0);
    m_intakeMotor.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrame::Status_9_MotProfBuffer_, 255, 0);
    m_intakeMotor.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrame::Status_10_MotionMagic_, 255, 0);
    m_intakeMotor.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrame::Status_10_Targets_, 255, 0);
    m_intakeMotor.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrame::Status_12_Feedback1_, 255, 0);
    m_intakeMotor.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrame::Status_13_Base_PIDF0_, 255, 0);
    m_intakeMotor.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrame::Status_14_Turn_PIDF1_, 255, 0);
    m_intakeMotor.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrame::Status_15_FirmareApiStatus_, 255, 0);
    m_intakeMotor.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrame::Status_17_Targets1_, 255, 0);
    
    }
//  kHatchSolenoidPorts[0], kHatchSolenoidPorts[1]} {}

void IntakeSubsystem::OpenIntake()
{
    m_intakeSolenoid.Set(frc::DoubleSolenoid::kForward);
}

void IntakeSubsystem::CloseIntake()
{
    m_intakeSolenoid.Set(frc::DoubleSolenoid::kReverse);
}

void IntakeSubsystem::IntakeRollersIn()
{
    m_intakeMotor.Set(ControlMode::PercentOutput, kIntakeRollersInPower);
}

void IntakeSubsystem::IntakeRollersOut()
{
    m_intakeMotor.Set(ControlMode::PercentOutput, kIntakeRollersOutPower);
}

void IntakeSubsystem::IntakeRollerStop()
{
    m_intakeMotor.Set(ControlMode::PercentOutput, 0.0);
}