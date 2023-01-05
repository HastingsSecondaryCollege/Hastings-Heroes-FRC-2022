// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

//#include <frc/Encoder.h>
//#include <frc/motorcontrol/Spark.h>
#include "ctre/Phoenix.h" // Needed for Falcons and Pigeon
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <wpi/numbers>
#include <units/angle.h>

#include "Constants.h"

using namespace OIConstants;
//#include <frc/WPILib.h>
class SwerveModule
{
    using radians_per_second_squared_t =
        units::compound_unit<units::radians,
                             units::inverse<units::squared<units::second>>>;

public:
    SwerveModule(int driveMotorCANBusID,
                 int turningMotorCANBusID,
                 int absoluteTurnEncoderID,
                 double offsetDegrees,
                 std::string moduleName
    );

    frc::SwerveModuleState GetState();
    void SetDesiredState(const frc::SwerveModuleState &state);
    void TurningPidOFF();
    void TurningPidON();
    void DrivePidOFF();
    void DrivePidON();
    void ResetEncoders();
    units::radian_t GetSwerveTurningFalconInternalRadians();
    std::string m_name;

private:
    // We have to use meters here instead of radians due to the fact that
    // ProfiledPIDController's constraints only take in meters per second and
    // meters per second squared.

    /*static constexpr units::radians_per_second_t kModuleMaxAngularVelocity =
        units::radians_per_second_t(wpi::numbers::pi); // radians per second
    static constexpr units::unit_t<radians_per_second_squared_t>
        kModuleMaxAngularAcceleration =
            units::unit_t<radians_per_second_squared_t>(
                wpi::numbers::pi * 2.0); // radians per second squared
*/
    WPI_TalonFX m_driveMotor;
    WPI_TalonFX m_turningMotor;
    CANCoder m_absoluteTurnEncoder; // This will be used to initialise the Falcons Built-in relative encoder 

    // Variable to hold the number of times the swerve module has turned through a full revolution 
    // Increments when the desired heading changes from somewhere in the range -90 to -180 to +90 to +180
    // Decrements when the desired heading changes from somewhere in the range +90 to +180 to -90 to -180
    int m_encoderWindUpRevolutions=0;

    //Variable used to remember the last heading of the swerve module to calculate if a wrapping condition has occured
    double m_prevModuleAngleDegrees;

};
