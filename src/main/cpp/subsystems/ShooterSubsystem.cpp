// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ShooterSubsystem.h"
#include <frc/smartdashboard/SmartDashboard.h>

ShooterSubsystem::ShooterSubsystem()
{
    // Implementation of subsystem constructor goes here.
    m_bottomMotor.ConfigFactoryDefault();
    m_bottomMotorMirror.ConfigFactoryDefault();
    m_topMotor.ConfigFactoryDefault();

    m_bottomMotor.ConfigNeutralDeadband(kNeutralDeadband);
    m_bottomMotorMirror.ConfigNeutralDeadband(kNeutralDeadband);
    m_topMotor.ConfigNeutralDeadband(kNeutralDeadband);
    m_bottomMotor.SetNeutralMode(ctre::phoenix::motorcontrol::Coast);
    m_bottomMotorMirror.SetNeutralMode(ctre::phoenix::motorcontrol::Coast);
    m_topMotor.SetNeutralMode(ctre::phoenix::motorcontrol::Coast);
    m_bottomMotorMirror.Follow(m_bottomMotor);
    m_bottomMotorMirror.SetInverted(true);
    m_bottomMotorMirror.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrame::Status_1_General_, 255, 0);
    m_bottomMotorMirror.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrame::Status_2_Feedback0_, 255, 0);
    m_bottomMotorMirror.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrame::Status_4_AinTempVbat_, 255, 0);
    m_bottomMotorMirror.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrame::Status_6_Misc_, 255, 0);
    m_bottomMotorMirror.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrame::Status_7_CommStatus_, 255, 0);
    m_bottomMotorMirror.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrame::Status_9_MotProfBuffer_, 255, 0);
    m_bottomMotorMirror.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrame::Status_10_MotionMagic_, 255, 0);
    m_bottomMotorMirror.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrame::Status_10_Targets_, 255, 0);
    m_bottomMotorMirror.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrame::Status_12_Feedback1_, 255, 0);
    m_bottomMotorMirror.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrame::Status_13_Base_PIDF0_, 255, 0);
    m_bottomMotorMirror.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrame::Status_14_Turn_PIDF1_, 255, 0);
    m_bottomMotorMirror.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrame::Status_15_FirmareApiStatus_, 255, 0);
    m_bottomMotorMirror.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrame::Status_17_Targets1_, 255, 0);

    m_bottomMotor.ConfigVoltageCompSaturation(12.0);
    m_bottomMotorMirror.ConfigVoltageCompSaturation(12.0);
    m_topMotor.ConfigVoltageCompSaturation(12.0);
    m_bottomMotor.ConfigPeakOutputForward(0); // Never allow Shooter Bottom Motor to spin in the positive direction.

    m_bottomMotor.ConfigSelectedFeedbackSensor(TalonFXFeedbackDevice::IntegratedSensor, kPIDLoopIdx, kTimeoutMs);
    // m_bottomMotorMirror.ConfigSelectedFeedbackSensor(TalonFXFeedbackDevice::IntegratedSensor, kPIDLoopIdx, kTimeoutMs);
    m_topMotor.ConfigSelectedFeedbackSensor(TalonFXFeedbackDevice::IntegratedSensor, kPIDLoopIdx, kTimeoutMs);

    m_bottomMotor.SetSelectedSensorPosition(0, kPIDLoopIdx, kTimeoutMs); // Reset bottom motor encoder position to 0 at robot boot
    m_topMotor.SetSelectedSensorPosition(0, kPIDLoopIdx, kTimeoutMs);    // Reset top motor encoder position to 0 at robot boot

    m_topMotor.Config_kF(kPIDLoopIdx, kFTopMotor, kTimeoutMs);
    m_bottomMotor.Config_kF(kPIDLoopIdx, kFBottomMotor, kTimeoutMs);

    m_topMotor.Config_kP(kPIDLoopIdx, kPTopMotor, kTimeoutMs);
    m_bottomMotor.Config_kP(kPIDLoopIdx, kPBottomMotor, kTimeoutMs);

    m_topMotor.Config_kI(kPIDLoopIdx, kITopMotor, kTimeoutMs);
    m_bottomMotor.Config_kI(kPIDLoopIdx, kIBottomMotor, kTimeoutMs);

    m_topMotor.Config_kD(kPIDLoopIdx, kDTopMotor, kTimeoutMs);
    m_bottomMotor.Config_kD(kPIDLoopIdx, kDBottomMotor, kTimeoutMs);

    m_bottomMotor.Config_IntegralZone(kPIDLoopIdx, kIzoneBottomMotor, kTimeoutMs); // have not tested izone
    m_topMotor.Config_IntegralZone(kPIDLoopIdx, kIzoneTopMotor, kTimeoutMs);       // have not tested izone

    frc::SmartDashboard::PutNumber("Shooter Adjustment Factor", m_bottomLongShotFactor);
}

void ShooterSubsystem::Periodic()
{
#ifdef DO_PROCESSING_TIMERS
    units::time::second_t startTime = frc::Timer::GetFPGATimestamp();
#endif
    // Implementation of subsystem periodic method goes here.

    /*
    if (m_percentOutputMode)
    {
        m_bottomMotor.Set(ControlMode::PercentOutput, m_bothShooterMotorsPercentOutput);
        m_topMotor.Set(ControlMode::PercentOutput, m_bothShooterMotorsPercentOutput);
    }
    else
    {
        if (m_bottomMotorEnabled)
    {
        // m_bottomMotor.Set(ControlMode::Velocity, m_bottomTargetVelocityUnitsPer100ms);

        //m_bottomMotor.Config_kF(kPIDLoopIdx, )
        m_bottomMotor.Set(ControlMode::Velocity, m_bottomTargetSpeedUnitsPer100ms);
    }
    else
    {
        m_bottomMotor.Set(ControlMode::PercentOutput, 0.0);
    }

    if (m_topMotorEnabled)
    {

        m_topMotor.Set(ControlMode::Velocity, m_topTargetSpeedUnitsPer100ms);
    }
    else
    {
        m_topMotor.Set(ControlMode::PercentOutput, 0.0);
    }

    }
    */
    m_topCurrentSpeedRPM = m_topMotor.GetSelectedSensorVelocity(kPIDLoopIdx) / kFalconTicksPerRevolution * k100MillisecondsPerMinute;
    m_bottomCurrentSpeedRPM = m_bottomMotor.GetSelectedSensorVelocity(kPIDLoopIdx) / kFalconTicksPerRevolution * k100MillisecondsPerMinute;
#ifdef DO_SMART_DASHBOARD
    frc::SmartDashboard::PutNumber("Top Motor Velocity", m_topMotor.GetSelectedSensorVelocity(kPIDLoopIdx));
    frc::SmartDashboard::PutNumber("Bottom Motor Velocity", m_bottomMotor.GetSelectedSensorVelocity(kPIDLoopIdx));
    frc::SmartDashboard::PutNumber("Top Motor Percentage Output", m_topMotor.GetMotorOutputPercent());
    frc::SmartDashboard::PutNumber("Bottom Motor Percentage Output", m_bottomMotor.GetMotorOutputPercent());
    frc::SmartDashboard::PutNumber("Top Motor Target Velocity", m_topTargetSpeedUnitsPer100ms);
    frc::SmartDashboard::PutNumber("Bottom Motor Target Velocity", m_bottomTargetSpeedUnitsPer100ms);
    frc::SmartDashboard::PutNumber("Top Motor Velocity Error", m_topMotor.GetClosedLoopError(kPIDLoopIdx));
    frc::SmartDashboard::PutNumber("Bottom Motor Velocity Error", m_bottomMotor.GetClosedLoopError(kPIDLoopIdx));
    frc::SmartDashboard::PutNumber("Top Motor Current RPM", m_topCurrentSpeedRPM);
    frc::SmartDashboard::PutNumber("Bottom Motor Current RPM", m_bottomCurrentSpeedRPM);
    frc::SmartDashboard::PutNumber("Top Motor Target RPM", m_topTargetSpeedRPM);
    frc::SmartDashboard::PutNumber("Bottom Motor Target RPM", m_bottomTargetSpeedRPM);
    frc::SmartDashboard::PutNumber("Distance To Target", m_distanceToTarget);

#endif
    /*  // Block commenting out SmartDashboard lines



    // Code below is temporary test code which will not live in the periodic method
    // m_distanceToTarget = frc::SmartDashboard::GetNumber("Distance To Target", 0); // Was not working when tested
    frc::SmartDashboard::PutNumber("Copy of Distance To Target", m_distanceToTarget); */
    // Block commenting out SmartDashboard lines

    /*double m_topCalculatedTargetPower = (((m_distanceToTarget * kTopMotorDistanceCoefficient) + kTopMotorDistanceYIntercept) / 100);
    double m_topCalculatedTargetRPM = (m_topCalculatedTargetPower * kMaxRPM);
    frc::SmartDashboard::PutNumber("Top Calculated Target RPM", m_topCalculatedTargetRPM);

    //double m_bottomCalculatedTargetPower = (((m_distanceToTarget * kBottomMotorDistanceCoefficient) + kBottomMotorDistanceYIntercept) / 100); // old calculation with motor power formula
    double m_bottomCalculatedTargetRPM = ((m_distanceToTarget * kBottomMotorDistanceCoefficient) + kBottomMotorDistanceYIntercept);
    frc::SmartDashboard::PutNumber("Bottom Calculated Target RPM", m_bottomCalculatedTargetRPM); */

#ifdef DO_PROCESSING_TIMERS
    units::time::second_t periodicRunTime = (frc::Timer::GetFPGATimestamp() - startTime);
    if (periodicRunTime > kRunningTooLong)
    {
        fmt::print("\n\nLimelight run time: {}\n\n\n", periodicRunTime);
    }

    if (periodicRunTime > m_maxRunTime)
    {
        m_maxRunTime = periodicRunTime;
        frc::SmartDashboard::PutNumber("Shooter Periodic Max run time", m_maxRunTime.value());
    }
#endif
}

void ShooterSubsystem::SetTopMotorState(bool MotorState)
{
    m_topMotorEnabled = MotorState;
    // frc::SmartDashboard::PutBoolean("Top Motor State", m_topMotorEnabled);
}

void ShooterSubsystem::SetBottomMotorState(bool MotorState)
{
    m_bottomMotorEnabled = MotorState;
    // frc::SmartDashboard::PutBoolean("Bottom Motor State", m_bottomMotorEnabled);
}

void ShooterSubsystem::SimulationPeriodic()
{
    // Implementation of subsystem simulation periodic method goes here.
}

void ShooterSubsystem::SetTopMotorTargetSpeed(double TopTargetSpeed)
{
    // Set the target speed to the top Motor
    m_topTargetSpeedRPM = TopTargetSpeed;
    m_topTargetSpeedUnitsPer100ms = (m_topTargetSpeedRPM * kFalconTicksPerRevolution) / k100MillisecondsPerMinute; // 13,653.3333333 for test distance 3000mm
    m_topRequiredPowerOutput = ((kFGradientTopMotor * m_topTargetSpeedUnitsPer100ms) + kFInterceptTopMotor);       // 0.60430836 for test distance 3000mm
    m_topRequiredkF = ((m_topRequiredPowerOutput * kFFactor) / m_topTargetSpeedUnitsPer100ms);                     // 0.045278866 for test distance 3000mm
    m_topMotor.Config_kF(kPIDLoopIdx, m_topRequiredkF);
    m_topMotor.Set(ControlMode::Velocity, m_topTargetSpeedUnitsPer100ms);
    // frc::SmartDashboard::PutNumber("Shooter Top Target RPM", m_topTargetSpeedRPM);
}

void ShooterSubsystem::SetBottomMotorTargetSpeed(double BottomTargetSpeed)
{
    // Set the target speed to the bottom Motor
    m_bottomTargetSpeedRPM = BottomTargetSpeed;
    m_bottomTargetSpeedUnitsPer100ms = (m_bottomTargetSpeedRPM * kFalconTicksPerRevolution) / k100MillisecondsPerMinute; // -7708.433 for test distance 3000mm
    m_bottomRequiredPowerOutput = ((kFGradientBottomMotor * m_bottomTargetSpeedUnitsPer100ms) + kFInterceptBottomMotor); // -0.340525256 for test distance 3000mm
    m_bottomRequiredkF = ((m_bottomRequiredPowerOutput * kFFactor) / m_bottomTargetSpeedUnitsPer100ms);                  // .04519172 for test distance 3000mm
    m_bottomMotor.Config_kF(kPIDLoopIdx, m_bottomRequiredkF);
    m_bottomMotor.Set(ControlMode::Velocity, m_bottomTargetSpeedUnitsPer100ms);
    // frc::SmartDashboard::PutNumber("Shooter Bottom Target RPM", m_bottomTargetSpeedRPM);
}

void ShooterSubsystem::IncrementTopMotorTargetSpeed(double Direction, double Increment)
{
    m_topTargetSpeedRPM += Direction * Increment;
    if (m_topTargetSpeedRPM > kMaxRPM)
        m_topTargetSpeedRPM = kMaxRPM;
    if (m_topTargetSpeedRPM < -kMaxRPM)
        m_topTargetSpeedRPM = -kMaxRPM;
    m_topTargetSpeedUnitsPer100ms = (m_topTargetSpeedRPM * kFalconTicksPerRevolution) / k100MillisecondsPerMinute;
    m_topRequiredPowerOutput = ((kFGradientTopMotor * m_topTargetSpeedUnitsPer100ms) + kFInterceptTopMotor);
    m_topRequiredkF = ((m_topRequiredPowerOutput * kFFactor) / m_topTargetSpeedUnitsPer100ms);
    m_topMotor.Config_kF(kPIDLoopIdx, m_topRequiredkF);
    // m_topMotor.Set(ControlMode::Velocity, m_topTargetSpeedUnitsPer100ms); // Commenting out as tuning in a different way, not starting shooter motors until button 3 on joystick 3 is pressed
    // frc::SmartDashboard::PutNumber("Shooter Top Target RPM", m_topTargetSpeedRPM);
}

void ShooterSubsystem::IncrementBottomMotorTargetSpeed(double Direction, double Increment)
{
    m_bottomTargetSpeedRPM += Direction * Increment;
    if (m_bottomTargetSpeedRPM < -kMaxRPM)
        m_bottomTargetSpeedRPM = -kMaxRPM;
    if (m_bottomTargetSpeedRPM > 0)
        m_bottomTargetSpeedRPM = 0;
    m_bottomTargetSpeedUnitsPer100ms = (m_bottomTargetSpeedRPM * kFalconTicksPerRevolution) / k100MillisecondsPerMinute;
    m_bottomRequiredPowerOutput = ((kFGradientBottomMotor * m_bottomTargetSpeedUnitsPer100ms) + kFInterceptBottomMotor);
    m_bottomRequiredkF = ((m_bottomRequiredPowerOutput * kFFactor) / m_bottomTargetSpeedUnitsPer100ms);
    m_bottomMotor.Config_kF(kPIDLoopIdx, m_bottomRequiredkF);
    // m_bottomMotor.Set(ControlMode::Velocity, m_bottomTargetSpeedUnitsPer100ms); // Commenting out as tuning in a different way, not starting shooter motors until button 3 on joystick 3 is pressed
    // frc::SmartDashboard::PutNumber("Shooter Bottom Target RPM", m_bottomTargetSpeedRPM);
}

void ShooterSubsystem::IncrementBothShooterMotorsPercentOutput(double Direction, double Increment)
{
    m_bothShooterMotorsPercentOutput += Direction * Increment;
    if (m_bothShooterMotorsPercentOutput > 1.0)
    {
        m_bothShooterMotorsPercentOutput = 1.0;
    }
    if (m_bothShooterMotorsPercentOutput < -1.0)
    {
        m_bothShooterMotorsPercentOutput = -1.0;
    }
}

bool ShooterSubsystem::ShooterSpeedsOnTarget()
{ // return true if the absolute value of the PID error <= tolerance for both top and bottom shooter motors
    return (abs(m_bottomMotor.GetClosedLoopError(kPIDLoopIdx)) <= kBottomMotorUnitsPer100msTolerance) && (abs(m_topMotor.GetClosedLoopError(kPIDLoopIdx)) <= kTopMotorUnitsPer100msTolerance);
}

double ShooterSubsystem::CalculateTopMotorTargetSpeed()
{ // Moved this code to periodic method of subsystem
    // frc::SmartDashboard::GetNumber("Distance To Target", m_distanceToTarget); // Had trouble getting distance from SmartDashboard
    // double m_topCalculatedTargetPower = (((m_distanceToTarget * kTopMotorDistanceCoefficient) + kTopMotorDistanceYIntercept) / 100); // Old calculation with percent output power
    if (m_distanceToTarget == 0.0)
    {
        m_topCalculatedTargetRPM = kTopOpponentBallSpeed;
    }
    else if (m_distanceToTarget < kFenderLowDistance)
    { // Top speed when robot is against fender shooting for low hub.
        m_topCalculatedTargetRPM = kTopFenderLowSpeed;
    }
    else if (m_distanceToTarget <= kTopMediumDistance)
    { // use standard top motor target RPM formula for distances less than 4750mm
        m_topCalculatedTargetRPM = kTopLowMediumSpeed;
    }
    else
    {
        m_topCalculatedTargetRPM = ((m_distanceToTarget * kTopMotorDistanceCoefficient) + kTopMotorDistanceYIntercept) * m_topMediumShotFactor; // 4000 with test distance 3000mm
    }

    // frc::SmartDashboard::PutNumber("Top Calculated Target RPM", m_topCalculatedTargetRPM);
    return (m_topCalculatedTargetRPM);
}

double ShooterSubsystem::CalculateBottomMotorTargetSpeed()
{ //  Moved this code back from periodic method of subsystem
    // frc::SmartDashboard::GetNumber("Distance To Target", m_distanceToTarget); // Had troubles getting distance from Smart Dashboard
    // double m_bottomCalculatedTargetPower = (((m_distanceToTarget * kBottomMotorDistanceCoefficient) + kBottomMotorDistanceYIntercept) / 100); // Old calculation with percent output power
    if (m_distanceToTarget == 0.0)
    {
        m_bottomCalculatedTargetRPM = kBottomOpponentBallSpeed;
    }
    else if (m_distanceToTarget < kFenderLowDistance)
    { // Bottom speed when robot is against fender shooting for low hub.
        m_bottomCalculatedTargetRPM = kBottomFenderLowSpeed;
    }
    else if (m_distanceToTarget < kBottomMediumDistance)
    { // kBottomUpperHubCloseShot is the lowest bottom motor target RPM to use when shooting for the Upper Hub
        m_bottomCalculatedTargetRPM = kBottomLowMediumSpeed;
    }
    else
    {
        m_bottomCalculatedTargetRPM = ((m_distanceToTarget * kBottomMotorDistanceCoefficient) + kBottomMotorDistanceYIntercept) * m_bottomLongShotFactor; // Use Desmos formula to calculate power if further than 3500mm away

        /* if (m_bottomCalculatedTargetRPM < kMaxTargetRPMBottom)
        { // Commenting out this section 1/4/22 as the current formula will only go faster than 6000 RPM if the distance is around 26 metres, which should not happen
            m_bottomCalculatedTargetRPM = kMaxTargetRPMBottom; // Set bottom shooter speed to -6000 RPM as the fastest target speed.
        } */
    }

    // frc::SmartDashboard::PutNumber("Bottom Calculated Target RPM", m_bottomCalculatedTargetRPM);
    // frc::SmartDashboard::PutNumber("Shooter Distance to Target", m_distanceToTarget);
    return (m_bottomCalculatedTargetRPM);
}

void ShooterSubsystem::SetDistanceToTarget(double Distance)
{
    m_distanceToTarget = Distance;
}

void ShooterSubsystem::IncrementDistanceToTarget(double Direction, double Increment)
{
    m_distanceToTarget += (Direction * Increment);
    if (m_distanceToTarget < 0)
        m_distanceToTarget = 0;
    if (m_bottomTargetSpeedRPM > 20)
        m_bottomTargetSpeedRPM = 20;
}

void ShooterSubsystem::IncrementShooterAdjustments(double Direction, double Increment)
{
    m_topMediumShotFactor += (Direction * Increment);
    m_bottomLongShotFactor += (Direction * Increment);
    if (m_topMediumShotFactor > 1.1)
        m_topMediumShotFactor = 1.1;
    if (m_topMediumShotFactor < 0.75)
        m_topMediumShotFactor = 0.75;
    if (m_bottomLongShotFactor > 1.1)
        m_bottomLongShotFactor = 1.1;
    if (m_bottomLongShotFactor < 0.75)
        m_bottomLongShotFactor = 0.75;
    frc::SmartDashboard::PutNumber("Shooter Adjustment Factor", m_bottomLongShotFactor);
}

void ShooterSubsystem::SetBottomPositionWhenBallCountZero()
{
    m_bottomPositionWhenBallCountZero = m_bottomMotor.GetSelectedSensorPosition();
}

double ShooterSubsystem::GetBottomPosition()
{
    return (m_bottomMotor.GetSelectedSensorPosition());
}

double ShooterSubsystem::GetBottomPositionWhenBallCountZero()
{
    return m_bottomPositionWhenBallCountZero;
}

void ShooterSubsystem::DisableShooter()
{
    m_bottomMotor.Set(ControlMode::PercentOutput, 0);
    m_topMotor.Set(ControlMode::PercentOutput, 0);
}

double ShooterSubsystem::GetBottomShooterTargetRPM()
{
    return m_bottomTargetSpeedRPM;
}

double ShooterSubsystem::GetTopShooterTargetRPM()
{
    return m_topTargetSpeedRPM;
}