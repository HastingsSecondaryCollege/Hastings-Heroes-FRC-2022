// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ClimberSubsystem.h"
#include <frc/Preferences.h>

// This method will be called once per scheduler run
void ClimberSubsystem::Periodic() {
    if (m_climberEnabled) {
        //Check to see if values have been changed in preferences
        int NewMaxClimbSpeed=frc::Preferences::GetInt(kMaxClimbSpeedKey);
        int NewMaxClimbAcceleration=frc::Preferences::GetInt(kMaxClimbAccelerationKey);
        if ((m_maxClimbSpeed != NewMaxClimbSpeed) || (m_maxClimbAcceleration != NewMaxClimbAcceleration)){
            m_maxClimbSpeed = NewMaxClimbSpeed;
            m_maxClimbAcceleration = NewMaxClimbAcceleration;
            SetClimbMaxVelAndAcc(m_maxClimbSpeed, m_maxClimbAcceleration);
            //fmt::print("Just set Max Climb speed to {}, and Max Climb Acc to {}",m_maxClimbSpeed,m_maxClimbAcceleration);
        }
        if (!m_jogDownClicked){
            if (m_climbButtonBoard->GetY()<-0.9){
                m_jogDownClicked=true;
                m_MotionMagicSetPointPostion+=kClimbIncrementSize;
                m_climberMotor.Set(ControlMode::MotionMagic, m_MotionMagicSetPointPostion);
                //frc::SmartDashboard::PutNumber("Climb SetPoint", m_MotionMagicSetPointPostion);
            }
        }
        else {
            if (m_climbButtonBoard->GetY()>=-0.9){
                m_jogDownClicked=false;
            }
        };


        if (!m_jogUpClicked){
            if (m_climbButtonBoard->GetY()>0.9){
                m_jogUpClicked=true;
                m_MotionMagicSetPointPostion-=kClimbIncrementSize;
                if (m_MotionMagicSetPointPostion<0) m_MotionMagicSetPointPostion=0; //No use going any more as it only starts to retract the hook again if you go negative.
                m_climberMotor.Set(ControlMode::MotionMagic, m_MotionMagicSetPointPostion);
                //frc::SmartDashboard::PutNumber("Climb SetPoint", m_MotionMagicSetPointPostion);
            }
        }
        else {
            if (m_climbButtonBoard->GetY()<=0.9){
                m_jogUpClicked=false;
            }
        };

    }
}
void ClimberSubsystem::KillClimbMotor(){
    m_climberMotor.Set(ControlMode::PercentOutput,0.0);
    //fmt::print("Just set motor to zero percent power\n");
    SetClimbMotorToBrakeMode();
}

ClimberSubsystem::ClimberSubsystem(frc::Joystick * climbButtonBoard, IntakeSubsystem* IntakeSUB)
    : m_2ndArmFWD_OutSoleniod{frc::PneumaticsModuleType::CTREPCM, k2ndArmForwardsOutPort, k2ndArmForwardsOutFloatPort},
      m_2ndArmBKW_InSolenoid{frc::PneumaticsModuleType::CTREPCM, k2ndArmBackwardsInPort, k2ndArmBackwardsInFloatPort},
      m_climbButtonBoard{climbButtonBoard},
      m_intakeSUB{IntakeSUB}
{
    m_climberMotor.ConfigFactoryDefault(); // Always reset the setting incase the motor has been replaced with used motor
    m_climberMotor.ConfigNeutralDeadband(kClimberNeutralDeadband);
    m_climberMotor.ConfigVoltageCompSaturation(12.0);
    m_climberMotor.SetNeutralMode(Brake); // Brake mode so that hook does extend during match and does drop after the buzzer
    m_climberMotor.ConfigSelectedFeedbackSensor(TalonFXFeedbackDevice::IntegratedSensor, kPIDLoopIdx, kTimeoutMs);
    m_climberMotor.SetSelectedSensorPosition(kClimbStartPosition, kPIDLoopIdx, kTimeoutMs);
    // m_climberMotor.SetSelectedSensorPosition(0, kPIDLoopIdx, kTimeoutMs); // Use this line when starting the robot with arm Fully out

    /* Sets the direction that motor runs in for positive values */
    m_climberMotor.SetInverted(TalonFXInvertType::CounterClockwise);

    /* Set relevant frame periods to be at least as fast as periodic rate */
    m_climberMotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 10, kTimeoutMs);  // recommended values from CTRE Example
    m_climberMotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 10, kTimeoutMs); // recommended values from CTRE Example

    /* Set the peak and nominal outputs */
    m_climberMotor.ConfigNominalOutputForward(0, kTimeoutMs);
    m_climberMotor.ConfigNominalOutputReverse(0, kTimeoutMs);
    m_climberMotor.ConfigPeakOutputForward(1.0, kTimeoutMs);
    m_climberMotor.ConfigPeakOutputReverse(-1.0, kTimeoutMs);

    /* Set Motion Magic gains in slot0 - see documentation */
    m_climberMotor.SelectProfileSlot(0, 0);
    m_climberMotor.Config_kF(0, 0.1, kTimeoutMs); // 0.3
    m_climberMotor.Config_kP(0, 0.6, kTimeoutMs); // 0.1
    m_climberMotor.Config_kI(0, 0.0, kTimeoutMs);
    m_climberMotor.Config_kD(0, 0.0, kTimeoutMs);

    /* Set acceleration and vcruise velocity */
    m_climberMotor.ConfigMotionCruiseVelocity(frc::Preferences::GetInt(kMaxClimbSpeedKey), kTimeoutMs); // 75000 is fast, could push up to this value if you want to reduce climb time.
    m_climberMotor.ConfigMotionAcceleration(frc::Preferences::GetInt(kMaxClimbAccelerationKey), kTimeoutMs);   // 35000 is fast, could push up to this value if you want to reduce climb time.
    
    /* Setup Motion magic position to the starting position */
    m_MotionMagicSetPointPostion = kClimbStartPosition;

    /* Make sure the arms are locked in place */
    m_2ndArmFWD_OutSoleniod.Set(frc::DoubleSolenoid::kReverse); // Float out solenoid at start of match
    m_2ndArmBKW_InSolenoid.Set(frc::DoubleSolenoid::kForward); // Set Arm in Solenoid at start of match
}

void ClimberSubsystem::EnableClimber()
{
    m_climberEnabled = true;
    m_intakeSUB->OpenIntake();
    //fmt::print("\n climber enable set to true and open intake\n");
}

void ClimberSubsystem::CloseIntake(){
    m_intakeSUB->CloseIntake();
}

void ClimberSubsystem::OpenIntake(){
    m_intakeSUB->OpenIntake();
}

// Proceedure to drive the arm to a position using motion magic
void ClimberSubsystem::MotionMagicToPosition(int ClimbPosition)
{
    if (m_climberEnabled)
    {
        m_MotionMagicSetPointPostion = ClimbPosition;
        m_climberMotor.Set(ControlMode::MotionMagic, ClimbPosition);
    }
}
bool ClimberSubsystem::HasMotionMagicFinished()
{
    return (m_climberMotor.GetActiveTrajectoryPosition() == m_MotionMagicSetPointPostion);
}
// Proccedure to extend arm out
void ClimberSubsystem::ArmOut()
{   //fmt::print("Inside Arm out, checking if enabled\n");
    if (m_climberEnabled)
    {
        //fmt::print("Inside ClimberSubsystem::ArmOut(), about to Set Soleniod\n");
        ArmInFloat();                                               // Float opposite soleniod
        m_2ndArmFWD_OutSoleniod.Set(frc::DoubleSolenoid::kForward); // set Arm Out soleniod
    }
}
// Proceedure to Float Out Arm Out Soleniod
void ClimberSubsystem::ArmOutFloat()
{
    if (m_climberEnabled)
    {

        m_2ndArmFWD_OutSoleniod.Set(frc::DoubleSolenoid::kReverse); // Float out soleniod
    }
}
// Proceedure to bring arm in
void ClimberSubsystem::ArmIn()
{
    if (m_climberEnabled)
    {
        ArmOutFloat();                                             // Float opposite soleniod
        m_2ndArmBKW_InSolenoid.Set(frc::DoubleSolenoid::kForward); // Set Arm in Soleniod
    }
}

// Proceedure to Float In Arm Soleniod
void ClimberSubsystem::ArmInFloat()
{
    if (m_climberEnabled)
    {
        m_2ndArmBKW_InSolenoid.Set(frc::DoubleSolenoid::kReverse); // Float in solenoid
    }
}

void ClimberSubsystem::StopClimbArm()
{
    if (m_climberEnabled)
    {
        // By putting air pressure on both sides of the ram it doesn't move much
        // NB: Not used in 2022
        m_2ndArmBKW_InSolenoid.Set(frc::DoubleSolenoid::kForward);
        m_2ndArmFWD_OutSoleniod.Set(frc::DoubleSolenoid::kForward);
    }
}


    // Procedure to set the motion magic Max Speed and Max Acceleration
void ClimberSubsystem::SetClimbMaxVelAndAcc(int MaxVelocity, int MaxAcceleration){
    m_climberMotor.ConfigMotionCruiseVelocity(MaxVelocity, kTimeoutMs); 
    m_climberMotor.ConfigMotionAcceleration(MaxAcceleration, kTimeoutMs);   
}

void ClimberSubsystem::SlowClimbMode(){
    m_slowClimbMode=true;
    m_climberMotor.ConfigMotionCruiseVelocity(20000.0, kTimeoutMs); 
    m_climberMotor.ConfigMotionAcceleration(20000.0, kTimeoutMs); 
}

void ClimberSubsystem::FastClimbMode(){
    m_slowClimbMode=false;
    m_climberMotor.ConfigMotionCruiseVelocity(frc::Preferences::GetInt(kMaxClimbSpeedKey), kTimeoutMs); 
    m_climberMotor.ConfigMotionAcceleration(frc::Preferences::GetInt(kMaxClimbAccelerationKey), kTimeoutMs); 
}

void ClimberSubsystem::SetClimbMotorToBrakeMode(){
    m_climberMotor.SetNeutralMode(Brake);
}

void ClimberSubsystem::SetClimbMotorToCoastMode(){
    m_climberMotor.SetNeutralMode(Coast);
}