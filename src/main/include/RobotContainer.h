// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.



#pragma once

#include <utility>
#include <units/angle.h>
#include <units/velocity.h>
#include <frc/PowerDistribution.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/Joystick.h>
#include <frc/controller/PIDController.h>
#include <frc/geometry/Translation2d.h>
//#include <frc/trajectory/Trajectory.h>
//#include <frc/trajectory/TrajectoryGenerator.h>

#include <frc2/command/Command.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/PIDCommand.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/SelectCommand.h>
#include <frc2/command/ConditionalCommand.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/button/POVButton.h>
#include <frc2/command/ParallelRaceGroup.h>


#include "Constants.h"
//#include "SwerveControllerCommand6508-Matt.h" //Renamed Matts version to -Matt
#include "SwerveControllerCommand6508.h" // Replaced with version that takes Path Planner Trajectories instead of WPILib Trajectories
#include "pathplanner/lib/PathPlanner.h"

// Subsystems
#include "subsystems/DriveSubsystem.h"
#include "subsystems/LimelightSubsystem.h"
#include "subsystems/ShooterSubsystem.h"
#include "subsystems/StorageSubsystem.h"
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/ClimberSubsystem.h"

// Commands
//#include "commands/SCurveDemo.h"    //Was used in Robotcontainer.inc
#include "commands/DefaultDrive.h"
#include "commands/ResetOdometryAtHub.h"
#include "commands/ResetOdometryLimelight.h"
#include "commands/DriveFixedFieldBearing.h"
#include "commands/CancelLockedHeading.h"
#include "commands/IncrementBottomMotorTargetSpeed.h"
#include "commands/IncrementTopMotorTargetSpeed.h"
#include "commands/SetBottomMotorTargetSpeed.h"
#include "commands/SetFeedMotorSpeed.h"
#include "commands/SetLoaderMotorSpeed.h"
#include "commands/SetReadyToShoot.h"
#include "commands/SetTopMotorTargetSpeed.h"
#include "commands/ZeroBallCount.h"
#include "commands/DisableBottomMotor.h"
#include "commands/DisableTopMotor.h"
#include "commands/SetShooterSpeedsThenFeedWhenReady.h"
#include "commands/SetCalculatedShooterSpeedsThenFeedWhenReady.h"
#include "commands/CalcShoot.h"
#include "commands/SetLoaderMotorTargetSpeed.h"
#include "commands/Intake/IntakeIfBallCountLessThanTwo.h"
#include "commands/IncrementDistanceToTarget.h"
#include "commands/intake/EjectIntakeAndLoaderAndFeeder.h"
#include "commands/Intake/IntakeOutAndIntakeDriveInwards.h"
#include "commands/Intake/IntakeInWaitThenStop.h"
#include "commands/CheckBallCount.h"
#include "commands/Intake/EjectOneBall.h"
#include "commands/IncrementBothMotorsPercentOutput.h"
#include "commands/TrackToHub.h"
#include "commands/StopTrackToHub.h"
#include "commands/RotateToHub.h"
#include "commands/IncrementShooterAdjustments.h"
#include "commands/CancelAllMotors.h"
#include "commands/ShootOnValidTarget.h"
#include "commands/SetLowShooterSpeedsThenFeedWhenReady.h"
#include "commands/SetDribbleShooterSpeedsThenFeedWhenReady.h"


  //Climber Commands includes
#include "commands/Climber/EnableClimber.h"
#include "commands/Climber/ClimberRestPosition.h"
#include "commands/Climber/ClimbFull.h"
#include "commands/Climber/ExtendClimber.h"
#include "commands/Climber/GoToSlideOffPosition.h"
#include "commands/Climber/GoToDriveOnPosition.h"
#include "commands/Climber/FloatClimbArms.h"
#include "commands/Climber/SecondClimbArmIn.h"
#include "commands/Climber/SecondClimbArmOut.h"
#include "commands/Climber/StopClimberSequence.h"

  // Intake Commands
#include "commands/Intake/IntakeDriveInwards.h"
#include "commands/Intake/IntakeDriveOutwards.h"
#include "commands/Intake/IntakeIn.h"
#include "commands/Intake/IntakeOut.h"
#include "commands/Intake/IntakeStop.h"
#include "commands/intake/EjectIntakeAndLoader.h"

  // Auto Commands
//#include "commands/auto/AutoRHSOneThenTwo.h"
//#include "commands/auto/AutoRHSOneThenTwoLauncher.h"
//#include "commands/auto/AutoRHSTwoThenOne.h"
// #include "commands/auto/AutoRHSTwoThenOneLauncher.h"
// #include "commands/auto/AutoRHSOneThenTwoThenTwoLowLauncher.h"
// #include "commands/auto/AutoWaypoints.h"
// #include "commands/auto/AutoControls.h"
// #include "commands/auto/AutoLauncher.h"

using namespace AutoConstants;

class RobotContainer
{
public:
  RobotContainer();

  frc2::Command *GetAutonomousCommand();

  //--------------Commented out in simple Auto Version
  //double hubAimTrajectory(int wayfileIndex, int trajectoryIndex, double curTime);

  /* Takes a double value (usually from a joystick axis) and makes a deadband region the size of
   * deadband either side of 0, then rescales joystick so max value is still 1 or -1
   */
  double ApplyDeadband(double joystickValue, double deadband);

  void CancelAllMotorsMethod();
  void TurnFanOn();
  void TurnFanOff();
  void StorageInitialisePreloadedBallColour();
  void StorageCheckBallCount();
  int GetAutoChooser();
  void LoadAndBuildAutoTrajectories();
  
  DriverStationMode m_driverStationMode = DISABLEDMODE;

private:

  // Drivermode

  // The chooser for the autonomous routines
  frc::SendableChooser<int> m_autoChooser;

  void ConfigureButtonBindings();

    //PDH Object
  frc::PowerDistribution m_powerDistribution{1,frc::PowerDistribution::ModuleType::kRev};
  
    // Joysticks and Button Boards
  frc::Joystick m_driverController{OIConstants::kDriverControllerPort};
  frc::Joystick m_shooterBothController{OIConstants::kShooterBothControllerPort}; // shooter tuning joystick
  
  //frc::XboxController m_xboxController{3};
 
 #ifdef DO_BUTTONBOARDS 
  frc::Joystick m_buttonBoardUSB1{OIConstants::kButtonBoardUSB1};
  frc::Joystick m_buttonBoardUSB2{OIConstants::kButtonBoardUSB2};
 #endif
 
  // The robot's subsystems
  
  DriveSubsystem m_drive;

#ifdef SWERVY
  LimelightSubsystem m_limelightSUB{&m_drive}; 
  IntakeSubsystem m_intakeSUB;
  StorageSubsystem m_storageSUB{&m_intakeSUB};
  ShooterSubsystem m_shooterSUB;
  ClimberSubsystem m_climberSUB{&m_buttonBoardUSB1, &m_intakeSUB};

  // The robot's commands
  //  ------------------------------Some of these could be inlined to reduce the number of files in the project----------------------------------
  EjectIntakeAndLoader m_ejectIntakeAndLoaderCMDGRP{&m_intakeSUB, &m_storageSUB, kEjectTimeSecondBall};
  IntakeEjectAndClose m_intakeEjectAndCloseCMDGRP{&m_intakeSUB, kEjectTimeWhenAutoClosing};
  EjectIntakeAndLoaderAndFeeder m_ejectIntakeAndLoaderAndFeederCMDGRP{&m_intakeSUB, &m_storageSUB, kEjectTimeFirstBall};
  IntakeOutAndIntakeDriveInwards m_intakeOutAndIntakeDriveInwardsCMDGRP{&m_intakeSUB, &m_storageSUB, kLoaderMotorTestSpeed};
  IntakeInWaitThenStop m_intakeInWaitThenStopCMDGRP{&m_intakeSUB, kIntakeTimeOnTriggerRelease, &m_storageSUB};
  SetCalculatedShooterSpeedsThenFeedWhenReady m_setCalculatedShooterSpeedsThenFeedWhenReadyCMDGRP{&m_shooterSUB, &m_storageSUB, &m_drive, &m_limelightSUB};
  CancelAllMotors m_cancelAllMotorsCMDGRP{&m_shooterSUB, &m_storageSUB, &m_intakeSUB};
  CalcShoot m_aimShoot{&m_shooterSUB, &m_storageSUB, &m_drive};
  SetShooterSpeedsThenFeedWhenReady m_setShooterSpeedsThenFeedWhenReadyManualUpperCMDGRP{&m_shooterSUB, &m_storageSUB, [this] {return kTopManualUpperSpeed;}, [this] {return kBottomManualUpperSpeed;}};
  SetShooterSpeedsThenFeedWhenReady m_setShooterSpeedsThenFeedWhenReadyTuningCMDGRP{&m_shooterSUB, &m_storageSUB, [this] {return m_shooterSUB.GetTopShooterTargetRPM();}, [this] {return m_shooterSUB.GetBottomShooterTargetRPM();}};
  SetLowShooterSpeedsThenFeedWhenReady m_setLowShooterSpeedsThenFeedWhenReadyCMDGRP{&m_shooterSUB, &m_storageSUB};
  SetDribbleShooterSpeedsThenFeedWhenReady m_setDribbleShooterSpeedsThenFeedWhenReadyCMDGRP{&m_shooterSUB, &m_storageSUB};
  IntakeIn m_intakeInCMD{&m_intakeSUB};
  IntakeOut m_intakeOutCMD{&m_intakeSUB};
  IntakeDriveInwards m_intakeDrvInwardsCMD{&m_intakeSUB};
  IntakeDriveOutwards m_intakeDrvOutwardsCMD{&m_intakeSUB};
  IntakeStop m_intakeDrvStop{&m_intakeSUB};
  DisableTopMotor m_disableTopMotorCMD{&m_shooterSUB};
  DisableBottomMotor m_disableBottomMotorCMD{&m_shooterSUB};
  SetLoaderMotorSpeed m_setLoaderMotorSpeedStopCMD{&m_storageSUB, 0};
  SetLoaderMotorTargetSpeed m_setLoaderMotorReverseSpeedCMD{&m_storageSUB, kLoaderMotorReversePower};
  CheckBallCount m_checkBallCountCMD{&m_storageSUB};
  SetFeedMotorSpeed m_setFeedMotorSpeedStopCMD{&m_storageSUB, 0};
  SetFeedMotorSpeed m_setFeedMotorSpeedForward{&m_storageSUB, kFeederMotorPower};
  ResetOdometryAtHub m_resetOdometryAtHub{&m_drive};
  ResetOdometryLimelight m_resetOdometryLimelight{&m_drive,&m_limelightSUB};
  EjectOneBall m_ejectOneBall{&m_intakeSUB, &m_storageSUB, &m_ejectIntakeAndLoaderCMDGRP, &m_ejectIntakeAndLoaderAndFeederCMDGRP};
  IncrementBothMotorsPercentOutput m_incrementBothMotorsPercentOutputIncreaseCMD{&m_shooterSUB, 1.0, kIncrementPercentOutput};
  IncrementBothMotorsPercentOutput m_incrementBothMotorsPercentOutputDecreaseCMD{&m_shooterSUB, -1.0, kIncrementPercentOutput};
  IncrementTopMotorTargetSpeed m_incrementTopMotorTargetSpeedIncreaseLargeCMD{&m_shooterSUB, 1.0, kLargeIncrement};
  IncrementTopMotorTargetSpeed m_incrementTopMotorTargetSpeedDecreaseLargeCMD{&m_shooterSUB, -1.0, kLargeIncrement};
  IncrementBottomMotorTargetSpeed m_incrementBottomMotorTargetSpeedIncreaseLargeCMD{&m_shooterSUB, 1.0, kLargeIncrement};
  IncrementBottomMotorTargetSpeed m_incrementBottomMotorTargetSpeedDecreaseLargeCMD{&m_shooterSUB, -1.0, kLargeIncrement};
  IncrementTopMotorTargetSpeed m_incrementTopMotorTargetSpeedIncreaseSmallCMD{&m_shooterSUB, 1.0, kSmallIncrement};
  IncrementTopMotorTargetSpeed m_incrementTopMotorTargetSpeedDecreaseSmallCMD{&m_shooterSUB, -1.0, kSmallIncrement};
  IncrementBottomMotorTargetSpeed m_incrementBottomMotorTargetSpeedIncreaseSmallCMD{&m_shooterSUB, 1.0, kSmallIncrement};
  IncrementBottomMotorTargetSpeed m_incrementBottomMotorTargetSpeedDecreaseSmallCMD{&m_shooterSUB, -1.0, kSmallIncrement};
  TrackToHub m_trackToHubCMD{&m_drive};
  StopTrackToHub m_stopTrackToHubCMD{&m_drive};
  RotateToHub m_rotateToHubCMD{&m_drive};
  IncrementShooterAdjustments m_incrementShooterAdjustmentsIncrease{&m_shooterSUB, kShooterAdjustmentStep, 1.0};
  IncrementShooterAdjustments m_incrementShooterAdjustmentsDecrease{&m_shooterSUB, kShooterAdjustmentStep, -1.0};
  SetBottomMotorTargetSpeed m_setBottomMotorTargetSpeedTuningCMD{&m_shooterSUB, [this] {return m_shooterSUB.GetBottomShooterTargetRPM();}};
  SetTopMotorTargetSpeed m_setTopMotorTargetSpeedTuningCMD{&m_shooterSUB, [this] {return m_shooterSUB.GetBottomShooterTargetRPM();}};
  IntakeIfBallCountLessThanTwo m_intakeIfBallCountLessThanTwoCMD{&m_storageSUB, &m_intakeSUB, &m_intakeOutAndIntakeDriveInwardsCMDGRP};
  ShootOnValidTarget m_shotOnValidTargetCMDGRP{&m_shooterSUB, &m_storageSUB, &m_drive, &m_limelightSUB};

  
    // Climber Commands
  EnableClimber             m_climberEnable{&m_climberSUB, &m_drive};
  GoToDriveOnPosition       m_ClimbStep0{&m_climberSUB};
  ClimbFull                 m_climbStep1{&m_climberSUB};
  SecondClimbArmIn          m_climbStep2{&m_climberSUB};
  GoToSlideOffPosition      m_climbStep3{&m_climberSUB};
  ExtendClimber             m_climbStep4{&m_climberSUB};
  SecondClimbArmOut         m_climbStep5{&m_climberSUB};
  GoToSlideOffPosition      m_climbStep6{&m_climberSUB};
  ClimbFull                 m_climbStep7{&m_climberSUB};
  SecondClimbArmIn          m_climbStep8{&m_climberSUB};
  GoToSlideOffPosition      m_climbStep9{&m_climberSUB};
  ClimberRestPosition       m_climbStep10{&m_climberSUB};
  StopClimberSequence       m_stopClimbSequence{&m_buttonBoardUSB2,&m_climberSUB};

    // Climber Command Groups
    // clang-format off
  frc2::ParallelRaceGroup m_automattedClimbCMDGRP{m_stopClimbSequence,
    frc2::SequentialCommandGroup{ ClimbFull(&m_climberSUB),
                                                        SecondClimbArmIn(&m_climberSUB),
                                                        frc2::WaitCommand(0.5_s),
                                                        GoToSlideOffPosition(&m_climberSUB),
                                                        //SecondClimbArmIn(&m_climberSUB),
                                                        frc2::WaitCommand(1.0_s),
                                                        ExtendClimber(&m_climberSUB),
                                                        SecondClimbArmOut(&m_climberSUB),
                                                        frc2::WaitCommand(1.0_s),
                                                        FloatClimbArms(&m_climberSUB),
                                                        GoToSlideOffPosition(&m_climberSUB),
                                                        SecondClimbArmOut(&m_climberSUB)
      }
  };// m_automattedClimbCMDGRP
  frc2::ParallelRaceGroup m_automattedClimbSlowCMDGRP{m_stopClimbSequence,
    frc2::SequentialCommandGroup{ ClimbFull(&m_climberSUB),
                                                        frc2::InstantCommand([this]{ m_climberSUB.OpenIntake(); }, {&m_climberSUB}),
                                                        SecondClimbArmIn(&m_climberSUB),
                                                        frc2::InstantCommand([this] { m_climberSUB.CloseIntake(); }, {&m_climberSUB}),
                                                        frc2::WaitCommand(1.5_s),
                                                        frc2::InstantCommand([this] {m_climberSUB.SlowClimbMode(); }, {&m_climberSUB}),
                                                        GoToSlideOffPosition(&m_climberSUB),
                                                        //SecondClimbArmIn(&m_climberSUB),
                                                        frc2::WaitCommand(1.0_s),
                                                        ExtendClimber(&m_climberSUB),
                                                        SecondClimbArmOut(&m_climberSUB),
                                                        frc2::WaitCommand(1.0_s),
                                                        FloatClimbArms(&m_climberSUB),
                                                        ClimberRestPosition(&m_climberSUB)
    }
  };// m_automattedClimbSlowCMDGRP

#endif
  // WDR -------------------------This code will be converted to command in the new simpleAuto Implementation
    // Instant command to rezero the Gyro
  frc2::InstantCommand m_ZeroHeadingCommand{[this] { m_drive.ZeroHeading(); },{}};

/*--------------Commented out for Simple Auto Setup------------------------------------
                          // trajectory and swervecontroller arrays for autonomous routines
                          frc::TrajectoryConfig *trajectoryConfigs[kMaxWayfiles][kMaxTrajectories];
                          frc::Trajectory trajectories[kMaxWayfiles][kMaxTrajectories];
                          std::vector<frc::Translation2d> translations[kMaxWayfiles];
                          int translationIndex[kMaxWayfiles];
                          // frc2::SwerveControllerCommand<4>* swerveControllerCommands[kMaxTrajectories];
                          frc2::SwerveControllerCommand<4> *swerveControllerCommands[kMaxWayfiles][kMaxTrajectories];
                          int trajectoryCount[kMaxWayfiles];
                          int hubAimState[kMaxWayfiles];
                          int lastHubAimState[kMaxWayfiles];
                          const int kHubAimStateDEFAULT = 0;
                          const int kHubAimStateHUB_AIM = 0;
                          const int kHubAimStateHUB_AIM_SECS = 0;
                          const int kHubAimStateHUB_OFFSET_AIM_SECS = 0;
                          const int kHubAimStateFINAL_POSE = 0;
                          frc::Pose2d autoStartPose[kMaxWayfiles];
                          frc::Rotation2d rotation2d[kMaxWayfiles];
                          char commands[kMaxWayfiles][kMaxCommands][kMaxCommandLen];    // Command List
                          int commandBeforeTrajectoryIndex[kMaxWayfiles][kMaxCommands]; // Commands are sequenced in relation to Trajectories
                          int commandCount[kMaxWayfiles];
                          int wayfileCount;

                          void MakeTrajectories(int wayFileIndex);
*/
    // Load trajectories from file system
 
    //H2_H1_H2 5 Ball Auto Part Paths
  pathplanner::PathPlannerTrajectory Path_H2_H1_H2_Part1 = pathplanner::PathPlanner::loadPath("2-1-2-Ball-RER-PartA"/*, 1_mps, 1.5_mps_sq*/);
  pathplanner::PathPlannerTrajectory Path_H2_H1_H2_Part2 = pathplanner::PathPlanner::loadPath("2-1-2-Ball-RER-PartB"/*, 4_mps, 2.5_mps_sq*/);
  pathplanner::PathPlannerTrajectory Path_H2_H1_H2_Part3 = pathplanner::PathPlanner::loadPath("2-1-2-Ball-RER-PartC"/*, 4_mps, 2.5_mps_sq*/);
  pathplanner::PathPlannerTrajectory Path_H2_H1_H2_Part4 = pathplanner::PathPlanner::loadPath("2-1-2-Ball-RER-PartD"/*, 4_mps, 2.5_mps_sq*/);

    //L1_H2_H2 5 Ball Auto Parts Paths
  pathplanner::PathPlannerTrajectory Path_L1_H2_H2_Part1 = pathplanner::PathPlanner::loadPath("5-Ball-L1-H2-H2-Part1"/*, 4_mps, 2_mps_sq*/);  //4,2.5 = 3.4sec    3,1.5 = 4.36
  pathplanner::PathPlannerTrajectory Path_L1_H2_H2_Part2 = pathplanner::PathPlanner::loadPath("5-Ball-L1-H2-H2-Part2"/*, 4_mps, 2.5_mps_sq*/);  //4,2.5 = 4.42sec   3,1.5 = 5.7sec
  pathplanner::PathPlannerTrajectory Path_L1_H2_H2_Part2B = pathplanner::PathPlanner::loadPath("5-Ball-L1-H2-H2-Part2B");
  pathplanner::PathPlannerTrajectory Path_L1_H2_H2_Part3 = pathplanner::PathPlanner::loadPath("5-Ball-180-Spin"/*, 0.5_mps, 1_mps_sq*/);  

    //H2_H2 4 Ball Auto Paths
  pathplanner::PathPlannerTrajectory Path_H2_H2_Part1 = pathplanner::PathPlanner::loadPath("4-Ball-Part1"/*, 1_mps, 1_mps_sq*/);
  pathplanner::PathPlannerTrajectory Path_H2_H2_Part2 = pathplanner::PathPlanner::loadPath("4-Ball-Part2"/*, 4_mps, 1_mps_sq*/);
  pathplanner::PathPlannerTrajectory Path_4BallReset =  pathplanner::PathPlanner::loadPath("4-Ball-Return"/*,1_mps, 1_mps_sq*/);

    //H2 2Ball Left Side LEL
  pathplanner::PathPlannerTrajectory Path_H2_Part1 = pathplanner::PathPlanner::loadPath("2-Ball-Left-Part1"/*, 2_mps, 1_mps_sq*/);
  pathplanner::PathPlannerTrajectory Path_H2_Part2 = pathplanner::PathPlanner::loadPath("2-Ball-Left-Part2"/*, 2_mps, 1_mps_sq*/);

    //H2 2Ball Right Side REL
  pathplanner::PathPlannerTrajectory Path_H2_REL = pathplanner::PathPlanner::loadPath("4-Ball-Part1"/*, 1_mps, 1_mps_sq*/);

    //5 Ball Reset
  pathplanner::PathPlannerTrajectory Path_5BallReset = pathplanner::PathPlanner::loadPath("5ballReturn"/*, 0.5_mps, 0.5_mps_sq*/);

    //Test Path Move 1M to the right
  pathplanner::PathPlannerTrajectory Path_1MTestPath = pathplanner::PathPlanner::loadPath("Test-Path"/*, 1_mps, 1_mps_sq*/);
  

  pathplanner::PathPlannerTrajectory Path2_3BallRightEdge = pathplanner::PathPlanner::loadPath("3-Ball-Right-Edge"/*, 1_mps, 1_mps_sq*/);
 

  frc2::Command* m_autos[20];
  // clang-format off

};//RobotConatiner Class Definition


