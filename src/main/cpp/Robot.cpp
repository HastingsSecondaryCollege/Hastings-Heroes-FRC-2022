// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandScheduler.h>
#include <frc/DriverStation.h>
#include "Constants.h"
#include <frc/Preferences.h>
#include <cameraserver/CameraServer.h>


void Robot::RobotInit() {
frc::CameraServer::StartAutomaticCapture();
using namespace ClimberConstants;

  //Setup Robot preferences for the climber to make changing constants values easier
  //Once the preference key has been created on the rio, this code will never run again.
      if (!frc::Preferences::ContainsKey(kMaxClimbSpeedKey)) {
      frc::Preferences::SetDouble(kMaxClimbSpeedKey, kMaxClimbCruiseVelocityInitPreferenceValue);   // NB 20000 was the values used at the wollongong Regional. Safe, but to slow.
                                                                                                    // This value will be over written by setting a robot preference
    }
    if (!frc::Preferences::ContainsKey(kMaxClimbAccelerationKey)) {
      frc::Preferences::SetDouble(kMaxClimbAccelerationKey, kMaxClimbAccelerationInitPreferenceValue);  // NB 10000 was the value used at the wollongong Regional.
                                                                                                        // This value will be over written by setting a robot preference
    }
  }//RobotInit

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want to run during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic()
{
  frc2::CommandScheduler::GetInstance().Run();
}

/**
 * This function is called once each time the robot enters Disabled mode. You
 * can use it to reset any subsystem information you want to clear when the
 * robot is disabled.
 */
void Robot::DisabledInit()
{
  m_container.m_driverStationMode=DISABLEDMODE;
  fmt::print("Just entered Disabled Mode\n");
  #ifdef SWERVY
    m_container.TurnFanOff();
  #endif
}

void Robot::DisabledPeriodic() {}

/**
 * This autonomous runs the autonomous command selected by your {@link
 * RobotContainer} class.
 */

void Robot::AutonomousInit()
{
  m_container.m_driverStationMode=AUTOMODE;
  fmt::print("Just entered Auto Mode\n");
  #ifdef SWERVY
    m_container.CancelAllMotorsMethod();
    m_container.StorageInitialisePreloadedBallColour();
  #endif
  
  m_autonomousCommand = m_container.GetAutonomousCommand();
  if (m_autonomousCommand != nullptr)
  {
    m_autonomousCommand->Schedule(false);
  }
}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit()
{
  // This makes sure that the autonomous stops running when
  // teleop starts running. If you want the autonomous to
  // continue until interrupted by another command, remove
  // this line or comment it out.
  #ifdef SWERVY
    m_container.CancelAllMotorsMethod();  //  Safety feature used when test at Port.
    m_container.StorageCheckBallCount();  //  Not needed for match play ,but used when going straight to teleOP
    m_container.TurnFanOn();
 
  #endif

  if (m_autonomousCommand != nullptr)
  {
    m_autonomousCommand->Cancel();
    m_autonomousCommand = nullptr;
  }
  fmt::print("Just entered TeleOp Mode\n");
  m_container.m_driverStationMode=TELEOPMODE;
 }

/**
 * This function is called periodically during operator control.
 */
void Robot::TeleopPeriodic() {}

/**
 * This function is called periodically during test mode.
 */
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
