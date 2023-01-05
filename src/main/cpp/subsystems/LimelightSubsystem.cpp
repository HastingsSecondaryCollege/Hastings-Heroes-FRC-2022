// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/LimelightSubsystem.h"
//#include <iostream>
#include <cmath>
#include "Constants.h"

#define PI 3.14159265

using namespace nt;
using namespace LimeConstants;
using namespace DriveConstants;


LimelightSubsystem::LimelightSubsystem(DriveSubsystem* DriveSUB)      // = default;
: m_driveSUB{DriveSUB}
{
  //LEDState(false);
  targetValid = 0;
}

// This method will be called once per scheduler run
void LimelightSubsystem::Periodic() {
  #ifdef DO_PROCESSING_TIMERS
  units::time::second_t startTime = frc::Timer::GetFPGATimestamp();
  #endif
  std::shared_ptr<NetworkTable> table = NetworkTableInstance::GetDefault().GetTable("limelight");
  targetOffsetAngle_Horizontal = table->GetNumber("tx",0.0);
  targetOffsetAngle_Vertical = table->GetNumber("ty",0.0);
  targetArea = table->GetNumber("ta",0.0);
  targetSkew = table->GetNumber("ts",0.0);
  targetValid = table->GetNumber("tv",0.0);
  frc::SmartDashboard::PutBoolean("Limelight target", CheckTarget()); // Constantly update boolean on SmartDashboard to show whether limelight has a valid target
      
  if (targetValid == 1){
    calcHeight = kTargetHeight-kCamHeight;
    calcAngle = kCamFixedAngle+targetOffsetAngle_Vertical;
    calcDistance = calcHeight/tan(calcAngle*(PI/180)); // tan() uses radians. calc distance (adjacent.)=calc height (opposite)/tan angle
    
        //--------------------------------------------------------------//
        /* WDR 24-4-22 Diagnosing Code after competition failures.
            There is no way the next line of code can give accurate distances
            as calcDistance is a double in Metres and the other variable are doubles in mm.
            This means GetXYPos would have been returning bad data.
        */
    targetGroundBotToHubCentre = calcDistance - kLimelightToBotCentre + kHubReflectorToCentre;
    
    m_distanceForShooter = ((kHubRadius + calcDistance) / cos(targetOffsetAngle_Horizontal*(PI/180))); // Distance calculation for shooter updated 31/3/22 AD
    #ifdef DO_UPDATE_ODOMETRY_VISION
      m_driveSUB->UpdateOdometryVision(fieldPos,kTrustLOW);
      auto odomCounters = m_driveSUB->GetOdometryUpdateCounters();
      frc::SmartDashboard::PutNumber("LL odometryUpdateCounter[kTrustLOW]: ",odomCounters[kTrustLOW]);
      frc::SmartDashboard::PutNumber("LL odometryUpdateCounter[kTrustMEDIUM]: ",odomCounters[kTrustMEDIUM]);
      frc::SmartDashboard::PutNumber("LL odometryUpdateCounter[kTrustHIGH]: ",odomCounters[kTrustHIGH]);
    #endif
    #ifdef DO_SMART_DASHBOARD
      fieldPos = GetXYPos();
      targetHubReflectorDistance = calcDistance/cos(calcAngle*(PI/180)); //target dist (hypot.).= calc distance (adjacent.) x cos angle
      targetAngledBotToHubCentre = targetHubReflectorDistance - kLimelightToBotCentre/cos(calcAngle*(PI/180)) + kHubReflectorToCentre/cos(calcAngle*(PI/180));
  
      frc::SmartDashboard::PutNumber("LL Target Horizontal Offset Angle: ", targetOffsetAngle_Horizontal);
      frc::SmartDashboard::PutNumber("LL Target Vertical Offset Angle: ", targetOffsetAngle_Vertical);
      frc::SmartDashboard::PutNumber("LL Target Area: ", targetArea);
      frc::SmartDashboard::PutNumber("LL Target Skew: ", targetSkew);
      frc::SmartDashboard::PutNumber("LL Target Distance: ", targetHubReflectorDistance);
      frc::SmartDashboard::PutNumber("LL Angled Bot To Hub Centre Distance: ", targetAngledBotToHubCentre);
      frc::SmartDashboard::PutNumber("LL Ground Bot To Hub Centre Distance: ", targetGroundBotToHubCentre); //most useful for field positioning and more convenient to test and measure with shooter
      frc::SmartDashboard::PutNumber("LL X Field Position: ", fieldPos.X().value());
      frc::SmartDashboard::PutNumber("LL Y Field Position: ", fieldPos.Y().value());
      frc::SmartDashboard::PutNumber("LL Field Rotation: ", fieldPos.Rotation().Degrees().value());
    #endif
    // The five lines below have been added for limelight distance checking on 28/3/22
      frc::SmartDashboard::PutNumber("LL Target Horizontal Offset Angle: ", targetOffsetAngle_Horizontal);
      frc::SmartDashboard::PutNumber("LL Target Vertical Offset Angle: ", targetOffsetAngle_Vertical);
      frc::SmartDashboard::PutNumber("New LL Ground Bot To Hub Centre Distance: ", m_distanceForShooter);
      }
    
  
  #ifdef DO_PROCESSING_TIMERS
  units::time::second_t periodicRunTime = (frc::Timer::GetFPGATimestamp() - startTime);
  if (periodicRunTime > kRunningTooLong)
  {
    fmt::print("\n\nLimelight run time: {}\n\n\n", periodicRunTime);
  }

  if (periodicRunTime > m_maxRunTime) 
  {
    m_maxRunTime = periodicRunTime;
    frc::SmartDashboard::PutNumber("Limelight Periodic Max run time", m_maxRunTime.value());
  }
  #endif
}

frc::Pose2d LimelightSubsystem::GetXYPos()
{
  /*calcTrueBearing = m_driveSUB->GetPigeonRotation2D().Degrees().value() - 180;
  xFieldPosition = targetDistance*cos(calcTrueBearing*(PI/180)) + kXTowerPos;
  yFieldPosition = targetDistance*sin(calcTrueBearing*(PI/180)) + kYTowerPos;*/
  
  //Rotate by +90 so that the angle that the robot is facing when is
  // see the hub gives correct offsets and correct Field Positioning.
  //LEDState(true);
  /*for (int x;x<10000;x++) {
    if (targetValid){
      fmt::print("Waited {} iterations for LED On/targetValid\n",x);
      break;
    }  
  }*/

    /** 24-4-22  WDR Code Analysis,  The following could never have worked because:-
     * targetValid is 1 whenever the target is visible by the limelight.  IE the target 
     * could be 20 degrees to the right in the Limelight and targetvalid would still be 1.
     * The Calculation of the position only looks at the PigeonRotation2D (heading of 
     * the robot) and doesn't compensate for the fact that the target is not in the 
     * centre of the limelight vision. If this updated during a path following, it would
     * break the path following code.
     * The other issue is that the values in targetGoundToHubCentre are invalid from where 
     * they are calculated in the limelight periodic because of a mixture of 
     * mm and metres in the formula to calculate the distance.
    */
  if (targetValid == 1){
    frc::Rotation2d bearing2d = m_driveSUB->GetPigeonRotation2D();
    calcTrueBearing = bearing2d.RotateBy(frc::Rotation2d(units::degree_t(180))).Degrees().value();
    xFieldPosition = kXTowerPos - targetGroundBotToHubCentre*cos(calcTrueBearing*(PI/180));
    yFieldPosition = kYTowerPos - targetGroundBotToHubCentre*sin(calcTrueBearing*(PI/180));
    return frc::Pose2d(frc::Translation2d(units::meter_t(xFieldPosition/1000),
                                        units::meter_t(yFieldPosition/1000)), 
                                        bearing2d);
  } else {
    return m_driveSUB->GetPose(); //no limelight so return last known
  }
  //LEDState(false);
}

double LimelightSubsystem::GetDistance()
{
  /** 24-4-22  WDR Code Analysis, multiple issues here. The variable that it returns is
   * set inside a #ifdef, which means that it doesn't work if that bit of code is #defined out
   * Also the calculation of this value is completed wrong.  Thankfully it appears that this was
   * never used.
  **/
  return targetHubReflectorDistance;
}

bool LimelightSubsystem::CheckTarget()
{
  if(targetValid == 1)
    return true;
  else
    return false;
}

double LimelightSubsystem::GetHorizontalTargetAngle() {
   return targetOffsetAngle_Horizontal;
}

double LimelightSubsystem::GetDistanceForShooter()
{
  return m_distanceForShooter;
}
