// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SwerveModule.h"

#include <frc/geometry/Rotation2d.h>
#include <wpi/numbers>
//#include <frc/smartdashboard/SmartDashboard.h>
#include "Constants.h"
using namespace DriveConstants;

SwerveModule::SwerveModule(int driveMotorCANBusID,
                           int turningMotorCANBusID,
                           int AbsoluteTurnEncoderID,
                           double offsetDegrees,
                           std::string moduleName) //WDR: Used by Smartdashboard puts.
    : m_driveMotor(driveMotorCANBusID),
      m_turningMotor(turningMotorCANBusID),
      m_absoluteTurnEncoder(AbsoluteTurnEncoderID)
{
  // Setup the CANCoder on the Swerve Module for absolute steering reading.
  m_absoluteTurnEncoder.ConfigSensorInitializationStrategy(ctre::phoenix::sensors::SensorInitializationStrategy::BootToAbsolutePosition, kTimeoutMs);
  m_absoluteTurnEncoder.ConfigAbsoluteSensorRange(ctre::phoenix::sensors::AbsoluteSensorRange::Signed_PlusMinus180, kTimeoutMs);
  m_absoluteTurnEncoder.ConfigMagnetOffset(offsetDegrees, kTimeoutMs);
  m_absoluteTurnEncoder.SetStatusFramePeriod(ctre::phoenix::sensors::CANCoderStatusFrame_VbatAndFaults, 255, kTimeoutMs);
  m_absoluteTurnEncoder.SetStatusFramePeriod(ctre::phoenix::sensors::CANCoderStatusFrame_SensorData, 255, kTimeoutMs);

  //Need to initialize the Steering Motor encoder to match the absolute Angle of the module
  //First get the absolute Angle of the module in degrees from the CANcoder
  double powerOnAbsoluteSteerAngle = m_absoluteTurnEncoder.GetAbsolutePosition();

  //Then Scales this up to match units of the integrated Sensor on the Steering Motor
  #ifdef SWERVY
    m_turningMotor.SetSelectedSensorPosition(-powerOnAbsoluteSteerAngle / 360.0 * kSteeringRatio * 2048.0);
  #else
    m_turningMotor.SetSelectedSensorPosition(powerOnAbsoluteSteerAngle / 360.0 * kSteeringRatio * 2048.0);
  #endif

  //frc::SmartDashboard::PutNumber(m_name + "/Initial SelectedSensorPosition: ", m_turningMotor.GetSelectedSensorPosition());

  //Initialise the last module heading value to the heading at the time the swerve modules are created (ie Powerup)
  m_prevModuleAngleDegrees = m_absoluteTurnEncoder.GetAbsolutePosition();
  //std::cout << "Set m_prevModuleAngleDegrees on " << moduleName << " to " << m_prevModuleAngleDegrees << "\n";

  //Initialize the encoder windup value
  m_encoderWindUpRevolutions = 0;

  //name the module from the name passed into the constructor
  m_name = moduleName;

  // Setup the Drive FalconFX

  // Factory reset the FalconFX Drive Motor to make sure there is no settings left there from a previous life.
  m_driveMotor.ConfigFactoryDefault();

  m_driveMotor.ConfigVoltageCompSaturation(11.0);

  /* set the peak and nominal outputs */
  m_driveMotor.ConfigNominalOutputForward(0, kTimeoutMs);
  m_driveMotor.ConfigNominalOutputReverse(0, kTimeoutMs);
  m_driveMotor.ConfigPeakOutputForward(1, kTimeoutMs);
  m_driveMotor.ConfigPeakOutputReverse(-1, kTimeoutMs);

  /* set closed loop gains in slot0 */
  m_driveMotor.Config_kF(kPIDLoopIdx, 0.1097, kTimeoutMs); //WDR:TODO need to check these for velocity mode. Was initially 0.1097
  m_driveMotor.Config_kP(kPIDLoopIdx, kDrive_kP, kTimeoutMs);    // WDR:TODO Need to check these numbers Initially 0.22
  m_driveMotor.Config_kI(kPIDLoopIdx, 0.0, kTimeoutMs);
  m_driveMotor.Config_kD(kPIDLoopIdx, 0.0, kTimeoutMs);

  // use the internal sensor for velocity feedback
  m_driveMotor.ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::IntegratedSensor, 0, 0);
  m_driveMotor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Coast);
  

  //Setup the steering FalconFX

  // Factory reset the FalconFX to make sure there is no settings left there from a previous life.
  //m_turningMotor.ConfigFactoryDefault();

  m_turningMotor.ConfigVoltageCompSaturation(11.0);

  /* set the peak and nominal outputs */
  m_turningMotor.ConfigNominalOutputForward(0, kTimeoutMs);
  m_turningMotor.ConfigNominalOutputReverse(0, kTimeoutMs);
  m_turningMotor.ConfigPeakOutputForward(1.0, kTimeoutMs);  //WDR these were 1.0, trying smaller values to see if less violent
  m_turningMotor.ConfigPeakOutputReverse(-1.0, kTimeoutMs); //WDR these were 1.0, trying smaller values to see if less violent
 
  /* set closed loop gains in slot0 */
  m_turningMotor.Config_kF(kPIDLoopIdx, 0.0, kTimeoutMs);  //WARNING this did have a kF of 0.1097, was set to 0 as position mode
  m_turningMotor.Config_kP(kPIDLoopIdx, kTurning_kP, kTimeoutMs); //0.22 
  m_turningMotor.Config_kI(kPIDLoopIdx, 0.0, kTimeoutMs);
  m_turningMotor.Config_kD(kPIDLoopIdx, 0.0, kTimeoutMs);

  //Use the internal sensor for position feedback on the PID position loop
  m_turningMotor.ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::IntegratedSensor, 0, 0);
  m_turningMotor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Coast);
  
  #ifdef SWERVY
    // 2022_swervy robot
    m_turningMotor.SetInverted(true);
    m_turningMotor.SetSensorPhase(true);
  #else
    m_turningMotor.SetInverted(false);
    m_turningMotor.SetSensorPhase(false);
  #endif
} //Swerve Module Constructor

frc::SwerveModuleState SwerveModule::GetState()
{
  // Returns the velocity of the module in m/s and the Angle in Rotation2D
  return {units::meters_per_second_t{m_driveMotor.GetSelectedSensorVelocity() * 10.0 * ModuleConstants::kDriveEncoderMetresPerPulse},
          frc::Rotation2d(units::radian_t(remainder((m_turningMotor.GetSelectedSensorPosition() * ModuleConstants::kTurningEncoderRadiansPerPulse), 360.0)))};
}

void SwerveModule::SetDesiredState(const frc::SwerveModuleState &referenceState)
{
  //Takes a reference state feeds two talon PID controler loops for speed and angle.
  // Optimize the reference state to avoid spinning further than 90 degrees
  const auto state = frc::SwerveModuleState::Optimize(
      referenceState, frc::Rotation2d(GetSwerveTurningFalconInternalRadians()));
  // Need velocity in units per 100ms,
  double speedInMetresPerSecond = state.speed.value();
  double speedInUnitsPer100mS = speedInMetresPerSecond / 10.0 / ModuleConstants::kDriveEncoderMetresPerPulse;

  m_driveMotor.Set(ControlMode::Velocity, speedInUnitsPer100mS);

  // New Code added by WDR to handle the 180 to -180 and -180 t0 180 boundry condition
 if (m_prevModuleAngleDegrees > 90)
  {
    //Our previous angle was in an area where a wrap could oocur (SW Corner)
    if (state.angle.Degrees().value() < -90)
    {
      //If we are here it means that target setpoint needs to be adjusted to prevent the
      // module from taking the long way to reach the target in the SE Corner.
      m_encoderWindUpRevolutions++;
    }
  }
  if (m_prevModuleAngleDegrees < -90)
  {
    //Our previous angle was in an area where a wrap could oocur(SE Corner)
    if (state.angle.Degrees().value() > 90)
    {
      //If we are here it mean that target setpoint needs to be adjusted to prevent the
      // module from taking the long way to reach the target in the SW Corner.
      m_encoderWindUpRevolutions--;
    }
  }
  //Update the last setpoint to the current setpoint
  m_prevModuleAngleDegrees = state.angle.Degrees().value();

  // Calculate the setpoint to turn to
  double steeringModuleNativeUnitsSetPoint = (state.angle.Degrees().value() / 360 + m_encoderWindUpRevolutions) * 2048.0 * kSteeringRatio;
  m_turningMotor.Set(ControlMode::Position, steeringModuleNativeUnitsSetPoint);
}  //SetDiresedState

void SwerveModule::TurningPidOFF()
{
   m_turningMotor.Config_kP(kPIDLoopIdx, 0.0, kTimeoutMs); 
}

void SwerveModule::TurningPidON()
{
   m_turningMotor.Config_kP(kPIDLoopIdx, kTurning_kP, kTimeoutMs); 
}

void SwerveModule::DrivePidOFF()
{
   m_driveMotor.Config_kP(kPIDLoopIdx, 0.0, kTimeoutMs); 
}

void SwerveModule::DrivePidON()
{
   m_driveMotor.Config_kP(kPIDLoopIdx, kDrive_kP, kTimeoutMs); 
}

void SwerveModule::ResetEncoders()
{
  m_driveMotor.SetSelectedSensorPosition(0.0, kPIDLoopIdx, kTimeoutMs);
  //  m_driveEncoder.Reset();
  // Dont want to reset the turning encoders as that will break all the steering code.
  // could implement code to to reset to the value returned by the cancoder.
  //m_turningEncoder.Reset();
}

units::radian_t SwerveModule::GetSwerveTurningFalconInternalRadians()
{
  //frc::SmartDashboard::PutNumber(m_name + "/m_turningMotor.GetSelectedSensorPosition() :", m_turningMotor.GetSelectedSensorPosition());
  //Gets the current angle of the Swerve module from the internal encoder on the Falcon
  return units::radian_t(remainder((m_turningMotor.GetSelectedSensorPosition() * ModuleConstants::kTurningEncoderRadiansPerPulse), 360.0));
}