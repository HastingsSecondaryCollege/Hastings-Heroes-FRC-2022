// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "Constants.h"
#include <ctre/phoenix.h>

using namespace ShooterConstants;
using namespace OIConstants;

class ShooterSubsystem : public frc2::SubsystemBase
{
public:
  ShooterSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  /**
   * Will be called periodically whenever the CommandScheduler runs during
   * simulation.
   */
  void SimulationPeriodic() override;

  /**
   * Sets top motor percentage power.
   */
  void SetTopMotorTargetSpeed(double TopTargetSpeed);

  /**
   * Sets Bottom motor percentage power.
   */
  void SetBottomMotorTargetSpeed(double BottomTargetSpeed);

  /**
   * Sets top motor enabled state.
   */
  void SetTopMotorState(bool MotorState);

  /**
   * Sets bottom motor enabled state.
   */
  void SetBottomMotorState(bool MotorState);

  void IncrementTopMotorTargetSpeed(double Direction, double Increment);
  /**
   * Changes Top Motor Speed by 5% in direction of sign of Direction
   */

  void IncrementBottomMotorTargetSpeed(double Direction, double Increment);
  /**
   * Changes Top Motor Speed by 5% in direction of sign of Direction
   */

void IncrementBothShooterMotorsPercentOutput(double Direction, double Increment);

  double CalculateTopMotorTargetSpeed(); // Calculates TopMotorTargetSpeed from distance entered on SmartDashboard

  double CalculateBottomMotorTargetSpeed(); // Calculates BottomMotorTargetSpeed from distance entered on SmartDashboard

  void SetDistanceToTarget(double Distance); // Set distance to target. Plan to do this based on odometry data

  void IncrementDistanceToTarget(double Direction, double Increment); // Increment or decrement distance to target

  void IncrementShooterAdjustments(double Direction, double Increment); 

  bool ShooterSpeedsOnTarget(); // Return true of Shooter top and bottom motor speeds within tolerance range of targets

  void SetBottomPositionWhenBallCountZero(); // Store position of shooter bottom motor when ball count drops to zero

  double GetBottomPosition(); //return current position of ShooterBottom motor

  double GetBottomPositionWhenBallCountZero(); //Return position ShooterBottom motor when ball count drops to zero

  void DisableShooter(); // Disable shooter but remember target speeds

  double GetBottomShooterTargetRPM(); // Return the target RPM of the shooter bottom motor

  double GetTopShooterTargetRPM(); // Return the target RPM of the shooter top motor

private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  TalonFX m_bottomMotor{kCANIDShooterBottomMotor, std::string{kCANivoreCANBusName}};
  TalonFX m_bottomMotorMirror{kCANIDShooterBottomMirrorMotor, std::string{kCANivoreCANBusName}};
  TalonFX m_topMotor{kCANIDShooterTopMotor, std::string{kCANivoreCANBusName}};

  double m_topTargetSpeedUnitsPer100ms = 0.0;
  double m_bottomTargetSpeedUnitsPer100ms = 0.0;
  double m_topTargetSpeedRPM = 0.0;
  double m_topCurrentSpeedRPM = 0.0;
  double m_bottomTargetSpeedRPM = 0.0;
  double m_bottomCurrentSpeedRPM = 0.0;
  double m_distanceToTarget = 3000; // Test default value of 3000
  double m_topCalculatedTargetPower = 0.0; // (((m_distanceToTarget * kTopMotorDistanceCoefficient) + kTopMotorDistanceYIntercept)/100);
  double m_topCalculatedTargetRPM = 0.0; // (m_topCalculatedTargetPower * kMaxRPM);
  double m_bottomCalculatedTargetPower = 0.0; // (((m_distanceToTarget * kBottomMotorDistanceCoefficient) + kBottomMotorDistanceYIntercept)/100);
  double m_bottomCalculatedTargetRPM = 0.0; // (m_bottomCalculatedTargetPower * kMaxRPM);
  double m_bottomPositionWhenBallCountZero = 0; // Position of Shooter bottom motor when ball is deindexed at last sensor
  double m_bothShooterMotorsPercentOutput = 0;
  double m_topRequiredPowerOutput = 0;
  double m_topRequiredkF = 0;
  double m_bottomRequiredPowerOutput = 0;
  double m_bottomRequiredkF = 0;
  double m_bottomLongShotFactor = 1.0; // updated 1/4/22 f4om 0.85; // Implemented 13/12/22 7am.
  double m_topMediumShotFactor = 1.0; // updated 31/3/22 from 0.85; // Implemented 13/12/22 7am.


  bool m_bottomMotorEnabled = false;
  bool m_topMotorEnabled = false;
  bool m_percentOutputMode = false; // Shooter does not function normally when this is true

  units::time::second_t m_maxRunTime = units::time::second_t(0);
};
