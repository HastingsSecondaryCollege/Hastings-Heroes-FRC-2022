// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "frc/smartdashboard/Smartdashboard.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h"
#include "wpi/span.h"
#include "subsystems/DriveSubsystem.h"

class LimelightSubsystem : public frc2::SubsystemBase {
 public:
  LimelightSubsystem(DriveSubsystem* DriveSUB);

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  /**
   * Returns the position of the bot using the limelight and pigeon
   * 
   * @return Pose2d coordinate
   */
  frc::Pose2d GetXYPos();

  /**
   * Activates (& Deactivates) limelight LED and starts getting values form the tables
   * 
   */
  //void LEDState(bool LEDOnFlag);

  /**
   * Get the distance to the targets
   * 
   * @return double in mm
   */
  double GetDistance();

  /**
   * Check if target estimate is valid
   * 
   * @return true 
   * @return false 
   */
 bool CheckTarget();
 bool m_isTargetCheckFinished = false;


 double GetHorizontalTargetAngle();

 double GetDistanceForShooter();

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
    double targetOffsetAngle_Horizontal;
    double targetOffsetAngle_Vertical;
    double targetArea;
    double targetSkew;
    double targetValid;
    double calcAngle;
    double calcHeight;
    double calcDistance;
    double targetHubReflectorDistance;
    double targetAngledBotToHubCentre;
    double targetGroundBotToHubCentre;
    double calcTrueBearing;
    double xFieldPosition;
    double yFieldPosition;
    double m_distanceForShooter = 0.0; // The distance that the limelight will give to the shooter subsytem to calculate required shooter motor speeds

    bool isAccurateDistance;
    
    frc::Pose2d fieldPos;

    DriveSubsystem* m_driveSUB;

    units::time::second_t m_maxRunTime = units::time::second_t(0);
};
