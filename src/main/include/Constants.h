// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include <wpi/numbers>
#include <string.h>

#include <frc/TimedRobot.h>
//#include <frc/smartdashboard/smartdashboard.h>

#include "rev/ColorSensorV3.h"
#define SWERVY
//#define DEBUG
#pragma once
//odometry updates might be delaying the limelight run 
//#define DO_UPDATE_ODOMETRY_VISION
//#define DEBUG_WITH_GLASS
//#define DO_SMART_DASHBOARD
#define DO_BUTTONBOARDS
//#define DO_PROCESSING_TIMERS

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or bool constants.  This should not be used for any other purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

// Global constants start here
constexpr units::time::second_t kRunningTooLong = units::time::second_t(.004);
constexpr int kTimeoutMs = 10;
constexpr int kPIDLoopIdx = 0;
constexpr int kSecondaryPIDLoopIdx = 1;
constexpr double kFFactor = 1023; 
constexpr double kNeutralDeadband = 0.001;
constexpr std::string_view kRoboRIOCANBusName = "rio";
constexpr std::string_view kCANivoreCANBusName = "CANivore1";
enum DriverStationMode {AUTOMODE=1,TELEOPMODE=2,DISABLEDMODE=3,TESTMODE=4};
// Global constants end here

namespace DriveConstants
{
    constexpr int kPigeonID = 5;

    constexpr int kFrontLeftDriveMotorID = 22;
    constexpr int kRearLeftDriveMotorID = 42;
    constexpr int kFrontRightDriveMotorID = 12;
    constexpr int kRearRightDriveMotorID = 32;

    constexpr int kFrontLeftTurningMotorID = 21;
    constexpr int kRearLeftTurningMotorID = 41;
    constexpr int kFrontRightTurningMotorID = 11;
    constexpr int kRearRightTurningMotorID = 31;

    constexpr int kFrontLeftAbsoluteTurningEncoderID = 20;
    constexpr int kRearLeftAbsoluteTurningEncoderID = 40;
    constexpr int kFrontRightAbsoluteTurningEncoderID = 10;
    constexpr int kRearRightAbsoluteTurningEncoderID = 30;

    #ifdef SWERVY
        //2022_swervy robot 
        constexpr double kTurning_kP = 0.22;
    #else
        //spo_robot
        constexpr double kTurning_kP = 0.22;
    #endif
    constexpr double kDrive_kP = 0.1;
    
/***
 * 
 * Get offsets:
         1) put Magnet Offset (deg) = 0 by setting all the k*AngleOffset = 0 below
         2) face each wheel forward (consistent bevel pos.)
         3) power cycle the cancoders (dont Enable robot)
         4) go to pheonix tuner and Self Test Snapshot (each CANCoder device)
         5) read off the absolute position, negate and store in all the k*AngleOffset below
*/
    #ifdef SWERVY
      //2022_swervy robot DURING REPLACE
      constexpr double kFrontleftAngleOffset = 159.609;
      constexpr double kFrontRightAngleOffset = 25.3;
      constexpr double kRearleftAngleOffset = -45.9;
      constexpr double kRearRightAngleOffset = 71.6;
      
      /*swervy BEFORE FRONT LEFT SWWERVE DRIVE REPLACE
      constexpr double kFrontleftAngleOffset = 148.8;
      constexpr double kFrontRightAngleOffset = 25.3;
      constexpr double kRearleftAngleOffset = -45.9;
      constexpr double kRearRightAngleOffset = 71.6;
      */
      /*constexpr double kFrontleftAngleOffset = 8.086;
      constexpr double kFrontRightAngleOffset = 100.635;
      constexpr double kRearleftAngleOffset = 145.986;
      constexpr double kRearRightAngleOffset = 43.682;*/
    #else
      //spo_robot
      constexpr double kFrontleftAngleOffset = -295.4;
      constexpr double kFrontRightAngleOffset = -131.2;
      constexpr double kRearleftAngleOffset = -62.2;
      constexpr double kRearRightAngleOffset = -224.4;
    #endif

    //calc steering redution ratio: 60/10*50/14 = belt cog / belt drv cog * reduction gear / motor pinion cog
    #ifdef SWERVY
      constexpr double kSteeringRatio = (60.0 / 10.0) * (50.0 / 14.0);
    #else
      constexpr double kSteeringRatio = (60.0 / 10.0) * (32.0 / 15.0);    
    #endif
   
    // Example value only - as above, this must be tuned for your drive!
    constexpr double kPFrontLeftVel = 0.5;
    constexpr double kPRearLeftVel = 0.5;
    constexpr double kPFrontRightVel = 0.5;
    constexpr double kPRearRightVel = 0.5;

    /*  
    *MAX speed of the robot in TeleOp set here
    */
    constexpr auto kMaxTeleOpSpeed = units::meters_per_second_t(5); 
    constexpr auto kMaxDrivingRotation = units::radians_per_second_t(2.5); // 5/3/22 changed from 1 to 3 then 2.5
    constexpr double kShootAngleTolerance = 2.0; //degrees
    constexpr auto kQuitSeconds = units::time::second_t(2.0); //timer back up to quit Rotate to heading after this many seconds
    constexpr double kHubX = 8.05; //center of hub is kHub meteres along x-axis 
    constexpr double kHubY = 4.12; //center of hub is kHub meteres along y-axis
    constexpr auto kStartX = units::meter_t(0);//5.68498);//kHubX-2.36052); // 7.59 for auto test run. 5.68498 for IA corridor. measured offset to centre of bot at start position nearest to fence (fence = 0.0 Y-axis) 
    constexpr auto kStartY = units::meter_t(0);//3.887);//kHubY-0.233);  // 1.77 for auto test run. 3.887 for IA corridormeasured offset to centre of bot at start position nearest to fence (fence = 0.0 Y-axis) 
    constexpr auto kStartRotation = units::degree_t(0);//-178.5;//-178.5; // -90 for auto test run. -178.5 for IA corridor
    constexpr auto kHubResetX=units::meter_t(7.64);
    constexpr auto kHubResetY=units::meter_t(3.17);
    constexpr auto kHubResetRotation = -109.5;
    constexpr double kDriveSpeedMultiplierWhileClimbing = 0.25; // Constant to be passed to the DriveSubsystem::SetDriveSpeedMultiplier method when climbing

    // vision trust standard deviations
    constexpr int kTrustHIGH = 10;
    const wpi::array<double, 3U> kTrustHIGHStdDevs {0.01,0.01,0.01};
    constexpr int kTrustMEDIUM = 5;
    const wpi::array<double, 3U>kTrustMEDIUMStdDevs {0.05,0.05,0.05};
    constexpr int kTrustLOW = 1;
    const wpi::array<double, 3U> kTrustLOWStdDevs {0.1,0.1,0.1};


} // namespace DriveConstants

namespace ModuleConstants
{
    constexpr int kEncoderCPR = 2048; //Falcon500 CPR
    constexpr double kWheelDiameterMeters = .10033;
    constexpr double kDriveGearRatio = (50.0 / 14.0) * (17 / 27.0) * (45.0 / 15.0); //L2 Drive gear ratio

    #ifdef SWERVY
      //2022_swervy robot 

      constexpr double kTurnGearRatio = (50.0 / 14.0) * (60.0 / 10.0);                //L2 Turn gear ratio
    #else
      //spo_robot 
      constexpr double kTurnGearRatio = (32.0 / 15.0) * (60.0 / 10.0);                //L2 Turn gear ratio
    #endif

    constexpr double kDriveEncoderMetresPerPulse =
        // using encoder inside Falcon so need to divide circumference by gear ratio and by CPR
        (kWheelDiameterMeters * wpi::numbers::pi) / static_cast<double>(kEncoderCPR) / kDriveGearRatio;

    constexpr double kTurningEncoderRadiansPerPulse =
        // using encoder inside Falcon so need to divide 2 pi radians by gear ratio and by CPR
        (wpi::numbers::pi * 2) / static_cast<double>(kEncoderCPR) / kTurnGearRatio;

    //constexpr double kPModuleTurningController = 0.1; //WDR Not used as we use onboard PID
    //constexpr double kPModuleDriveController = 0.1;  //WDR Not used as we use onboard PID
} // namespace ModuleConstants

namespace AutoConstants
{
    using radians_per_second_squared_t =
        units::compound_unit<units::radians,
                             units::inverse<units::squared<units::second>>>;

    constexpr auto kMaxSpeed = units::meters_per_second_t(5);  //Was set to 3
    constexpr auto kMaxAcceleration = units::meters_per_second_squared_t(3); //Was set to 3
    constexpr auto kMaxAngularSpeed = units::radians_per_second_t(10);//(3.142);  //Was set to 3.142, 135 deg run was 6
    constexpr auto kMaxAngularAcceleration =
        units::unit_t<radians_per_second_squared_t>(10);//(3.142);  //was set to 3.142, 135 deg run was 6

    constexpr double kPXController = 2.0;       //Was set to 0.5
    constexpr double kPYController = 2.0;       //Was set to 0.5
    constexpr double kPThetaController = 10.0;//4.5;   //Was set to 0.5 ,135 deg run was 10

    const char kTrajectoryFilename[] = "/home/lvuser/wayfiles/autonomous"; //-1.txt  etc

    constexpr auto kAutoTrajectoryStartVelocity = units::meters_per_second_t(0); //don't start at 0 mps at start of Trajectory
    constexpr auto kAutoTrajectoryEndVelocity = units::meters_per_second_t(0); //don't stop at end of Trajectory
    constexpr double kCommenceFinalPoseDistance = 1.5; //max distance (required to turn 180 degrees), in meters, to complete turn to final pose (after hub aiming).

    
 
    constexpr double AirTime[8][2] = {{2.5,0.63},{3.0,1.2},{3.5,1.25},{4.0,1.33},{4.5,1.41},{5.0,1.33},{5.5,1.4},{6.0,1.45}};
    
    constexpr int kMaxTrajectories=20;
    constexpr int kMaxCommands=20;
    constexpr int kMaxCommandLen=50;
    constexpr int kMaxWayfiles = 15;

    extern const frc::TrapezoidProfile<units::radians>::Constraints
        kThetaControllerConstraints;
    
    constexpr units::time::second_t kAutoRHSOneThenTwoFirstWaitUntil = units::time::second_t(14.0); // When 14.0 seconds remains in Autonomous, we should have turned and be ready to shoot
    constexpr units::time::second_t kAutoRHSOneThenTwoSecondWaitUntil = units::time::second_t(5.0); // When 5.0 seconds remains in Autonomous, we should have picked up two balls and be ready to close intake
    constexpr units::time::second_t kAutoRHSOneThenTwoThirdWaitUntil = units::time::second_t(4.0); // When 4.0 seconds remains in Autonomous, we should have turned and be ready to shoot

    constexpr units::time::second_t kAutoRHSOneThenTwoTeleOpFirstWaitUntil = units::time::second_t(134.0); // When 134.0 seconds remains in TeleOp, we should have turned and be ready to shoot
    constexpr units::time::second_t kAutoRHSOneThenTwoTeleOpSecondWaitUntil = units::time::second_t(125.0); // When 125.0 seconds remains in TeleOpAutonomus, we should have picked up two balls and be ready to close intake
    constexpr units::time::second_t kAutoRHSOneThenTwoTeleOpThirdWaitUntil = units::time::second_t(124.0); // When 124.0 seconds remains in TeleOp, we should have turned and be ready to shoot

    constexpr units::time::second_t kAutoRHSTwoThenOneFirstWaitUntil = units::time::second_t(11.0); // When 11.0 seconds remains in Autonomous, we should have picked up one ball
    constexpr units::time::second_t kAutoRHSTwoThenOneSecondWaitUntil = units::time::second_t(10.0); // When 10.0 seconds remains in Autonomous, we should have turned and be ready to shoot
    constexpr units::time::second_t kAutoRHSTwoThenOneThirdWaitUntil = units::time::second_t(3.0); // When 3.0 seconds remains in Autonomous, we should have picked up one ball
    constexpr units::time::second_t kAutoRHSTwoThenOneFourthWaitUntil = units::time::second_t(2.0); // When 2.0 seconds remains in Autonomous, we should have turned and be ready to shoot

    constexpr units::time::second_t kAutoRHSTwoThenOneTeleOpFirstWaitUntil = units::time::second_t(131.0); // When 131.0 seconds remains in TeleOp, we should have picked up one ball
    constexpr units::time::second_t kAutoRHSTwoThenOneTeleOpSecondWaitUntil = units::time::second_t(130.0); // When 130.0 seconds remains in TeleOp, we should have turned and be ready to shoot
    constexpr units::time::second_t kAutoRHSTwoThenOneTeleOpThirdWaitUntil = units::time::second_t(123.0); // When 123.0 seconds remains in TeleOp, we should have picked up one ball
    constexpr units::time::second_t kAutoRHSTwoThenOneTeleOpFourthWaitUntil = units::time::second_t(122.0); // When 122.0 seconds remains in TeleOp, we should have turned and be ready to shoot

    constexpr units::time::second_t kAutoRHSOneThenTwoThenTwoLowFirstWaitUntil = units::time::second_t(10.4085); // intake When 10.0 seconds remains in Autonomous, we should have picked up two balls
    constexpr units::time::second_t kAutoRHSOneThenTwoThenTwoLowSecondWaitUntil = units::time::second_t(9.821794); // shoot When 8.0 seconds remains in Autonomous, we should have moved and be ready to shoot
    constexpr units::time::second_t kAutoRHSOneThenTwoThenTwoLowThirdWaitUntil = units::time::second_t(7.621426); // intake down/spin When 3.0 seconds remains in Autonomous, we should be approaching the last two balls
    constexpr units::time::second_t kAutoRHSOneThenTwoThenTwoLowFourthWaitUntil = units::time::second_t(4.205843); // When 2.0 seconds remains in Autonomous, we should have picked up last two balls
    constexpr units::time::second_t kAutoRHSOneThenTwoThenTwoLowFifthWaitUntil = units::time::second_t(1.62168); // When 0.5 seconds remains in Autonomous, we should have moveds and be ready to shoot

    constexpr units::meter_t kStartingPosition6x = units::meter_t(6.73);
    constexpr units::meter_t kStartingPosition6y = units::meter_t(5.91);
    //constexpr frc::Rotation2d(units::angle::degree_t(136.5)) kStartingRotation6 = frc::Rotation2d(units::angle::degree_t(136.5))
    constexpr units::time::second_t kAutoCountdown = units::time::second_t(15);

} // namespace AutoConstants

namespace OIConstants
{
    constexpr int kDriverControllerPort = 0;
    constexpr int kButtonBoardUSB1 = 1;
    constexpr int kButtonBoardUSB2 = 2;
    constexpr int kShooterBothControllerPort = 3;
    constexpr int kShooterBottomControllerPort = 4;
    
} // namespace OIConstants




namespace LimeConstants
{
    constexpr double kTargetHeight = 2.587; // 1/4/22 changed from 2646;//top of tape is 2646 in test env.  orig 2580;
    constexpr double kLimelightToBotCentre = 118; //back of limelight lens is 118mm back from center of bot
    constexpr double kHubReflectorToCentre = 684; //front of reflector to center of hub in mm
    constexpr double kHubRadius = 0.643764; // Radius of Hub in metres. Used in distance calculation. 

    constexpr double kXTowerPos = DriveConstants::kHubX*1000.0; // in mm
    constexpr double kYTowerPos = DriveConstants::kHubY*1000.0; // in mm
    
    // update cam values when cam is moved
    // 8-4-22 Horizontal Calibration corrected to 0 from - 0.083583 and Vertical Calibration left at 0.032543 becuase didn't want retake all values.
    constexpr double kCamHeight = 0.843254; // 8-4-22 Changed from 0.868128 to 0.843254 after centering Cross hair // 1/4/22 changed from 853;//900; // in mm   853mm as measured on SWERVY bot mid lens
    constexpr double kCamFixedAngle = 26.62; // 8-4-22 Changed from 26.234 to 26.62 after centering Cross hair // 1/4/22 changed from 27.637; // in degrees
    
    //limelight tracking adjusting rotation drive kP and min adjust
    constexpr double kPLL = -0.1f;
    constexpr double kMinLL = 0.05f;
} // namespace LimeConstants


namespace ShooterConstants
{

    constexpr double kMaxVelocity = 21700; // 21700 encoder ticks per 100ms
    constexpr double kFalconTicksPerRevolution = 2048;
    constexpr double k100MillisecondsPerMinute = 600;
    constexpr double kMaxRPM = (kMaxVelocity / kFalconTicksPerRevolution) * k100MillisecondsPerMinute; // Revolutions per minute
    constexpr double kMaxTargetRPMBottom = -6000.0; // Do not set a target speed faster than this
    constexpr double kFMultiplierTopMotor = 0.99255; // Modifying kF based on recorded RPM figures during tuning
    constexpr double kFGradientTopMotor = 0.0000444656; // first value 0.0000435291;
    constexpr double kFInterceptTopMotor = -0.0027953; //first value -0.00326099;
    constexpr double kFGradientBottomMotor = 0.0000451124;
    constexpr double kFInterceptBottomMotor = 0.00722066;
    constexpr double kFTopMotor = (kFFactor / kMaxVelocity) * kFMultiplierTopMotor; // Sample code was 1023.0/20660.0
    constexpr double kFMultiplierBottomMotor = 0.9476; // Modifying kF based on recorded RPM figures during tuning
    constexpr double kFBottomMotor = (kFFactor / kMaxVelocity) * kFMultiplierBottomMotor; // Sample code was 1023.0/20660.0
    constexpr double kPTopMotor = 0.25;//25; // Sample code was 0.1. Prototype code was 0.25
    // 1.0 bad, 0.5 bad
    constexpr double kPBottomMotor = 0.175; // 0.175 prototype value Sample code was 0.1. .125 pretty good, could go higher. .2 too high. .175 good results. .185 settled, but took a few seconds with no D. .19 still a few seconds
    // 1/3/22. 0.2 OK, 1.0 bad, 0.6 bad, 0.4 bad, 0.3 still oscillates. 0.175 still pretty god. 0.2 oscillating a bit, even with D
    constexpr double kITopMotor = 0.001;//01; // Sample code was 0.001. Prototype code was 0.001
    constexpr double kIBottomMotor = 0.001;//01; // Sample code was 0.001. Prototype code was 0.001
    constexpr double kDTopMotor = 7.5;//7.5; //Sample code was 5. Prototype code was 7.5
    constexpr double kDBottomMotor = 5.0;//5.0; // prototype value 5.0. sample code was 5. 3.5 still overcorrected. 10.5 corrected well. Test with lower D. 7 corrected well. and centred well. 5.0 and 7.0 to be retested with fresh battery
    constexpr double kSmallIncrement = 20; // A small increment To set speed
    constexpr double kSmallIncrementTop = 100; 
    constexpr double kLargeIncrement = 200; //A large increment to set speed for shooter
    constexpr double kIncrementPercentOutput = 0.05;
    constexpr double kIZoneRPMBottomMotor = 50;
    constexpr double kIZoneRPMTopMotor = 50;
    constexpr double kIzoneBottomMotor = (kIZoneRPMBottomMotor * kFalconTicksPerRevolution) / k100MillisecondsPerMinute;
    constexpr double kIzoneTopMotor = (kIZoneRPMTopMotor * kFalconTicksPerRevolution) / k100MillisecondsPerMinute;
    constexpr int kCANIDShooterBottomMotor = 1;
    constexpr int kCANIDShooterBottomMirrorMotor = 2;
    constexpr int kCANIDShooterTopMotor = 3;
    constexpr double kTopMotorTestSpeed = 2500; //RPM
    constexpr double kBottomMotorTestSpeed = -2500; // RPM, This should always be negative
    constexpr double kTopMotorRPMTolerance = 10; // Within 10 RPM of target speed will be considered ready to shoot
    constexpr double kTopMotorUnitsPer100msTolerance = (kTopMotorRPMTolerance * kFalconTicksPerRevolution) / k100MillisecondsPerMinute; // Tolerance in encoder ticks per 100ms
    constexpr double kBottomMotorRPMTolerance = 10; // Within 10 RPM of target speed will be considered ready to shoot
    constexpr double kBottomMotorUnitsPer100msTolerance = (kBottomMotorRPMTolerance * kFalconTicksPerRevolution) / k100MillisecondsPerMinute; // Tolerance in encoder ticks per 100ms
    constexpr double kTopMotorDistanceCoefficient = 0.185; // Updated 31/3/22 from -0.9452; // updated 5/3/22 from -1; // Multiplier of distance in formula. Original value was -18 for motor power with distance in metres
    constexpr double kTopMotorDistanceYIntercept = 1202; // updated 31/3/22 from 6879.8; // updated 5/3/22 from 7000; // Added to distance formula. Original value was 98 for motor power with distance in metres
    constexpr double kTopMotorDistanceForZeroSpeed = 5250; // Any distance greater than or equal to this value will trigger a top motor speed of zero. 
    constexpr double kTopMotorLowDistanceforLowSpeed = 4750; // Lowest distance at which kTOPMotorLowSpeed is applied
    constexpr double kTopMotorHighDistanceforLowSpeed = 4750; // Highest distance at which kTOPMotorLowSpeed is applied
    constexpr double kTOPMotorLowSpeed = 500; // Shooter top motor low speed for distances that don't match the main calculation
    constexpr double kBottomMotorDistanceCoefficient = -0.185; // updated 31/3/22 from -1.03857;// updated 5/3/22 from -1.05; // Multiplier of distance in formula. This value is delibarately negative. Original value was -14 for motor power with distance in metres
    constexpr double kBottomMotorDistanceYIntercept = -1202; // updated 31/3/22 from 1007.29;// updated 5/3/22 from 891.667;// Added to distance formula. Original value was 9 for motor power with distance in metres
    constexpr double kShooterDistanceIncrement = 0.5; // 0.5 metres is distance increment level for testing
    constexpr double kShooterExitRotations = 5; // Number of rotations we want Shooter Bottom Motor to complete after last sensor loses ball before stopping motors
    constexpr double kShooterExitUnitsper100ms = (kShooterExitRotations * kFalconTicksPerRevolution) / k100MillisecondsPerMinute;
    constexpr double kBottomOpponentBallSpeed = -700; //Slow speed to shoot opponent ball out for bottom motor, is always negative
    constexpr double kTopOpponentBallSpeed = 700; //Slow speed to shoot opponent ball out for top motor
    constexpr double kBottomFenderLowSpeed = -1000; // Bottom motor speed when robot bumper against fender shooting for low hub
    constexpr double kTopFenderLowSpeed = 1000; // Top motor speed when robot bumper against fender shooting for low hub
    constexpr double kBottomManualUpperSpeed = -2400; // Bottom motor speed when robot centre on TARMAC line.
    constexpr double kTopManualUpperSpeed = 4233.24; // Top motor speed when robot centre on TARMAC line, calculated from 2800mm distance. Used by manual shoot button
    constexpr double kBottomUpperHubCloseShot = -2000; // changed from -2400 13/3/22 7am; // 7/3/22 changed from -2500 to -2400. Not yet tested. Lowest power to use when shooting for Upper Hub
    constexpr double kFenderLowDistance = 2800; // Max distance to do low close shot
    constexpr double kTopMediumDistance = 3200; // Max distance for low hub medium shot
    constexpr double kBottomMediumDistance = 3200; // Max distance for low hub medium shot
    constexpr double kTopMediumHighDistance = 4750;  // Use 1500 top motor target RPM if distance less than kTopMediumHighDistance 500 top motor target RPM if distance greater than kTopMediumHighDistance
    constexpr double kTopMediumHighSpeed = 1500;
    constexpr double kTopLowMediumSpeed = 1140;
    constexpr double kBottomLowMediumSpeed = -1140;
    // constexpr double kBottomLongShotFactor = 0.9; // Implemented 13/12/22 7am. Changing to a member variable
    // constexpr double kTopMediumShotFactor = 0.9; // Implemented 13/12/22 7am. Changing to a member variable
    constexpr double kShooterAdjustmentStep = 0.01; // Implemented 13/3/22 8am

    constexpr double kTopManualUpperSpeedH1 = 4000.0;
    constexpr double kBottomManualUpperSpeedH1 = -2000.0;
    constexpr double kTopManualUpperSpeedH2 = 3400.0;
    constexpr double kBottomManualUpperSpeedH2 = -2000.0;
    constexpr double kTopManualUpperSpeedH3 = 3400.0;
    constexpr double kBottomManualUpperSpeedH3 = -2200.0;


    
} // namespace ShooterConstants

namespace StorageConstants
{
    constexpr int kDIOPortFirstBallStorage = 1;
    constexpr int kDIOPortSecondBallStorage = 0;
    constexpr int kDIOPortFirstBallPastFeeder = 2;
    constexpr int kDIOPortTest = 3;
    constexpr double kFeederMotorPower = 0.75; // Changed from 1.0 to 0.75 5/3/22
    constexpr double kLoaderMotorPower = 0.4;
    constexpr double kFeederMotorReversePower = -kFeederMotorPower; // Power to use when ejecting ball out intake
    constexpr double kLoaderMotorReversePower = -kLoaderMotorPower; // Power to use when ejecting ball out intake
    constexpr int kCANIDFeedMotor = 4;
    constexpr int kCANIDLoaderMotor = 6;
    constexpr double kLoaderMotorBallDistance = 20000.0;// Distance loader motor moves to load 1 ball
    constexpr double kLoaderToFeederBallDistance = 100000.0; // if loader motor moves this far and feeder sensor stil doesn't see the ball, something is wrong
    constexpr double kLoaderMaxVelocity = 21700; // 21700 encoder ticks per 100ms
    constexpr double kFLoaderMotor = (kFFactor / kLoaderMaxVelocity); // Sample code was 1023.0/20660.0
    constexpr double kPLoaderMotor = 0.0; // Value copied from tuning of shooter top motor. Sample code was 0.1
    constexpr double kILoaderMotor = 0.0; // Value copied from tuning of shooter top motor. Sample code was 0.001
    constexpr double kDLoaderMotor = 7.5; // Value copied from tuning of shooter top motor. Sample code was 5
    constexpr double kLoaderMotorTestSpeed = 2500; //RPM
    constexpr double kLoaderMotorReverseTestSpeed = -kLoaderMotorTestSpeed; //RPM
    constexpr double kFLoaderMotorPosition = 0.0;
    constexpr double kPLoaderMotorPosition = 0.1;
    constexpr double kILoaderMotorPosition = 0.0;
    constexpr double kDLoaderMotorPosition = 0.0;
    constexpr double kFeederMotorBallDistance = 20000.0; // Need to check this distance for feeder motor
    constexpr double kLoaderMotorSecondBallPositionModifier = 2000; // 6500 was when using color sensor for proximity

    /**
   * Note: Any example colors should be calibrated as the user needs, these
   * are here as a basic example.
   */
    constexpr frc::Color kBlueTarget = frc::Color(0.21, 0.46, 0.30); //Sample Code: 0.143, 0.427, 0.429
    constexpr frc::Color kRedTarget = frc::Color(0.37, 0.43, 0.19); //Sample Code:0.561, 0.232, 0.114
    constexpr double kHueThreshold = 94.0; // A hue value lower than this indicates a red ball. A hue value higher than this indicates a blue ball

      /**
   * Change the I2C port below to match the connection of your color sensor
   */
    constexpr auto kI2CPort = frc::I2C::Port::kMXP; // kOnboard or kMXP
    constexpr uint32_t kMinProximity = 700; // The color sensor will wait until m_colorSensor.GetProximity() >= kMinProximity before checking the colour of a ball
    constexpr int kMaxBallCount = 3;

}  // namespace StorageConstants

namespace IntakeConstants
{
    constexpr int kCANIDIntakeMotor = 7; //Need to set CANID to 7
    constexpr int kPCMPortIntakeForward = 0;
    constexpr int kPCMPortIntakeReverse = 1;
    constexpr double kIntakeRollersInPower = 0.6; // 0.25 too slow. 5/3/22 chagned from .6 to .3 to reduce breakages
    constexpr double kIntakeRollersOutPower = -kIntakeRollersInPower;
    constexpr  units::time::second_t kEjectTimeWhenAutoClosing = units::time::second_t(1.0); // Number of seconds intake wheels will ejected while intake closers when ball count gets to 2
    constexpr  units::time::second_t kEjectTimeSecondBall = units::time::second_t(0.5); // Number of seconds intake wheels will eject while intake closes when ball count >= 2
    constexpr  units::time::second_t kEjectTimeFirstBall = units::time::second_t(1.0); // Number of seconds intake wheels will eject while intake closes when ball count <= 1
    constexpr  units::time::second_t kIntakeTimeOnTriggerRelease = units::time::second_t(1.0); // Number of seconds intake wheels will intake while intake closes when driver releases joystick trigger button
}

namespace ClimberConstants
{
constexpr int kCANIDClimberMotor = 8;
constexpr int k2ndArmForwardsOutPort = 6;
constexpr int k2ndArmForwardsOutFloatPort = 5;
constexpr int k2ndArmBackwardsInPort = 4;
constexpr int k2ndArmBackwardsInFloatPort =7;
constexpr int kFullExtendEncoderPosition = 0;  

static constexpr std::string_view kMaxClimbSpeedKey = "Max Climb Speed";
static constexpr std::string_view kMaxClimbAccelerationKey = "Max Climb Acceleration";

constexpr int kMaxClimbCruiseVelocityInitPreferenceValue = 20000;  // NB This is value is only used once to create the preference key, from that point one the value saved in preferences will be `used
constexpr int kMaxClimbAccelerationInitPreferenceValue = 10000;  // NB This is value is only used once to create the preference key, from that point one the value saved in preferences will be `used
                                               
// When the hook is fully out/up the encoder should be zero. Because the robot needs to start with the hook down the 
// encoder needs to be set to a value that it would be with the hook in a partially retracted position.
// Initially this will be set by winding the hook down manually to a known position (mark on the cog) before robot bootup.  Then the encoder will 
// be set to the value in kClimbStartPosition.  Given time, this should be replaced with a calcaluted value from the absolution position of through bore encoders.  
// Tricky part is that these shafts turn 7 full turns and we can only find an absolute position of one revolution.  
constexpr int kClimbStartPosition = 246318;    //HSC 246318 Encoder Count when climber wound down and mark on cog lines up with Polycarb, see Drew if Warren not around.
constexpr int kFullRetractEncoderPosition = 258000;  // HSC 25800, This is tight rope value. If spindle winds with loose cord this will get very tight at the end.
constexpr int kStartClimbHookHeightPosition = 27800;  // HSC 27800 was the height of the bar at HSC, could need calibrating at Comp
constexpr int kDisconnectFromBarWithPoweredHookPosition = 80000;  //HSC 80,000 Point were the powered hook successfully lets go of its bar when hanging from the next bar
constexpr int kClimbIncrementSize = 3000;

constexpr double kClimberNeutralDeadband = 0.001;

} // namespace ClimberConstants



