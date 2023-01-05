// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include <utility>
#include <frc/controller/PIDController.h>
#include <frc/geometry/Translation2d.h>
#include <frc/shuffleboard/Shuffleboard.h>
//#include <frc/trajectory/Trajectory.h>
//#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/button/POVButton.h>
#include <units/angle.h>
#include <units/velocity.h>
#include <fstream>
#include <cmath>

#include "Constants.h"
#include "SwerveControllerCommand6508.h"  

// Subsystems
#include "subsystems/DriveSubsystem.h"

// Commands
#include "commands/SCurveDemo.h"
#include "commands/DefaultDrive.h"
#include "commands/DriveFixedFieldBearing.h"
#include "commands/CancelLockedHeading.h"
#include "commands/RotateToHub.h"
#include "commands/SetLoaderMotorTargetPosition.h"

using namespace DriveConstants;
using namespace AutoConstants;

RobotContainer::RobotContainer()
{
  // Initialize all of your commands and subsystems here

    // m_autoChooser.SetDefaultOption("Right Five Ball Low", 5);
  m_autoChooser.AddOption("4Ball:H2_H2 Setup REL", 0);
  m_autoChooser.AddOption("2Ball:H2 Setup LEL", 1);
  m_autoChooser.AddOption("2Ball:H2 Setup REL", 5);
  m_autoChooser.AddOption("5Ball:H2_H1_H2 Setup RER",2);
  m_autoChooser.SetDefaultOption("5Ball:L1_H2_H2 Setup RER",3);
  m_autoChooser.AddOption("3Ball:L1_H2 Setup RER",4);

    // Add the chooser to the dashboard
  frc::SmartDashboard::PutData("Autonomous Run", &m_autoChooser);

    // WDR:12-4-22 Hoping this will now put a Gyro on the dashboard that is tied to the Pigeon values
    // since a custom initSendable has now been written for the DriveSubsystem
  frc::SmartDashboard::PutData("Drive",&m_drive);
  
    // Configure the button bindings
  ConfigureButtonBindings();

  // Read in Path Planner Paths and build trajectories  NB: The Code for this is in RobotContainerSimple.inc (included at the end of this file)
  // We want to do this in the constructor for the RobotContainer because it takes some time to do this, and if it delays the command scheduler
  // we want the scheduler to completely catch up to running normal again before we start Auto or TeleOp
  LoadAndBuildAutoTrajectories();

  // Set up default drive command
  // clang-format off
    // m_drive.SetDefaultCommand(DefaultDrive(
    //   &m_drive,[this]{ return ApplyDeadband(-m_xboxController.GetLeftY(), 0.1) * (1 - m_driverController.GetThrottle()) / 2 * 5.0; }, // WDR:Need to implement constants for deadband and maxspeed
    //   [this]{ return ApplyDeadband(-m_xboxController.GetLeftX(), 0.1) * (1 - m_driverController.GetThrottle()) / 2 * 5.0; }, // WDR:Need to implement constants for deadband and maxspeed
    //   [this]{ return ApplyDeadband(-m_driverController.GetZ(), 0.2) * kMaxDrivingRotation.value(); } // WDR:Need to implement constants for deadband and max rotation
    // ));
    m_drive.SetDefaultCommand(DefaultDrive(
      &m_drive,[this]{ return ApplyDeadband(-m_driverController.GetY(), 0.1) * (1 - m_driverController.GetThrottle()) / 2 * 5.0; }, // WDR:Need to implement constants for deadband and maxspeed
      [this]{ return ApplyDeadband(-m_driverController.GetX(), 0.1) * (1 - m_driverController.GetThrottle()) / 2 * 5.0; }, // WDR:Need to implement constants for deadband and maxspeed
      [this]{ return ApplyDeadband(-m_driverController.GetZ(), 0.2) * kMaxDrivingRotation.value(); } // WDR:Need to implement constants for deadband and max rotation
    ));
  // clang-format on    

  //  Name some events
    frc::Shuffleboard::AddEventMarker("Test Event Marker Title Normal Importance", "Test Event Marker Test",frc::ShuffleboardEventImportance::kNormal);
    m_intakeOutCMD.SetName("m_intakeOUTCMD");
/*
    frc2::CommandScheduler::GetInstance().OnCommandInitialize(
      [](const frc2::Command& command) {
        frc::::Shuffleboard::AddEventMarker(
            "Command Initialized", command.GetName(),
            frc::ShuffleboardEventImportance::kNormal);
      });

    frc2::CommandScheduler::GetInstance().OnCommandInterrupt(
      [](const frc2::Command& command) {
        frc::Shuffleboard::AddEventMarker(
            "Command Interrupted", command.GetName(),
            frc::ShuffleboardEventImportance::kNormal);
      });

    frc2::CommandScheduler::GetInstance().OnCommandFinish(
      [](const frc2::Command& command) {
        frc::Shuffleboard::AddEventMarker(
            "Command Finished", command.GetName(),
            frc::ShuffleboardEventImportance::kNormal);
      });  
 */
}


#ifdef SWERVY
void RobotContainer::CancelAllMotorsMethod(){
  m_disableTopMotorCMD.Schedule();
  m_disableBottomMotorCMD.Schedule();
  m_setLoaderMotorSpeedStopCMD.Schedule();
  m_setFeedMotorSpeedStopCMD.Schedule();
  m_intakeDrvStop.Schedule();
}

void RobotContainer::TurnFanOn(){
  m_powerDistribution.SetSwitchableChannel(true);
}

void RobotContainer::TurnFanOff(){
  m_powerDistribution.SetSwitchableChannel(false);
}
#endif

//   -------------------Maybe replaced with a different auto chooser setup ie SelectCommand
int RobotContainer::GetAutoChooser()
{
  return (m_autoChooser.GetSelected());
}

void RobotContainer::ConfigureButtonBindings() {
  #ifdef SWERVY

    frc2::JoystickButton(&m_driverController, 1).WhenPressed(&m_intakeIfBallCountLessThanTwoCMD);
    frc2::JoystickButton(&m_driverController, 1).WhenReleased(&m_intakeInWaitThenStopCMDGRP);
    frc2::JoystickButton(&m_driverController, 2).WhenHeld(new SCurveDemo(&m_drive));
    //frc2::JoystickButton(&m_driverController, 3).WhenPressed(&m_setCalculatedShooterSpeedsThenFeedWhenReadyCMDGRP);
    frc2::JoystickButton(&m_driverController, 3).WhenPressed(&m_shotOnValidTargetCMDGRP);
    frc2::JoystickButton(&m_driverController, 5).WhenPressed(&m_resetOdometryAtHub);
    frc2::JoystickButton(&m_driverController, 7).WhenPressed(frc2::InstantCommand([this] {m_storageSUB.SetIgnoreBallColorStatus(false);})); 
    frc2::JoystickButton(&m_driverController, 8).WhenPressed(frc2::InstantCommand([this] {m_storageSUB.SetIgnoreBallColorStatus(true);})); 
    frc2::JoystickButton(&m_driverController, 9).WhenPressed(&m_incrementShooterAdjustmentsDecrease); 
    frc2::JoystickButton(&m_driverController, 10).WhenPressed(&m_incrementShooterAdjustmentsIncrease); 
    frc2::JoystickButton(&m_driverController, 11).WhenPressed(&m_trackToHubCMD); 
    frc2::JoystickButton(&m_driverController, 11).WhenReleased(&m_stopTrackToHubCMD);
    frc2::JoystickButton(&m_driverController, 12).WhenPressed(&m_trackToHubCMD); 
    frc2::JoystickButton(&m_driverController, 12).WhenReleased(&m_stopTrackToHubCMD);


    // Shooter Tuning test joystick buttons
    /* 
    frc2::JoystickButton(&m_shooterBothController, 1).WhenPressed(&m_intakeIfBallCountLessThanTwoCMD);
    frc2::JoystickButton(&m_shooterBothController, 1).WhenReleased(&m_intakeInWaitThenStopCMDGRP);
    frc2::JoystickButton(&m_shooterBothController, 3).WhenPressed(&m_setShooterSpeedsThenFeedWhenReadyTuningCMDGRP);
    frc2::JoystickButton(&m_shooterBothController, 5).WhenPressed(&m_resetOdometryAtHub);
    frc2::JoystickButton(&m_shooterBothController, 6).WhenPressed(&m_resetOdometryLimelight);
    frc2::JoystickButton(&m_shooterBothController, 7).WhenPressed(&m_incrementBottomMotorTargetSpeedIncreaseLargeCMD);
    frc2::JoystickButton(&m_shooterBothController, 7).WhenPressed(&m_incrementTopMotorTargetSpeedDecreaseLargeCMD);
    frc2::JoystickButton(&m_shooterBothController, 8).WhenPressed(&m_incrementBottomMotorTargetSpeedDecreaseLargeCMD);
    frc2::JoystickButton(&m_shooterBothController, 8).WhenPressed(&m_incrementTopMotorTargetSpeedIncreaseLargeCMD);
    frc2::JoystickButton(&m_shooterBothController, 9).WhenPressed(&m_incrementBottomMotorTargetSpeedIncreaseSmallCMD);
    frc2::JoystickButton(&m_shooterBothController, 9).WhenPressed(&m_incrementTopMotorTargetSpeedDecreaseSmallCMD);
    frc2::JoystickButton(&m_shooterBothController, 10).WhenPressed(&m_incrementBottomMotorTargetSpeedDecreaseSmallCMD);
    frc2::JoystickButton(&m_shooterBothController, 10).WhenPressed(&m_incrementTopMotorTargetSpeedIncreaseSmallCMD);
    frc2::JoystickButton(&m_shooterBothController , 11).WhenPressed(&m_checkBallCountCMD);
    frc2::JoystickButton(&m_shooterBothController , 12).WhenPressed(&m_ejectOneBall); 
  */

  // Button Board USB 1 buttons start here
  #ifdef DO_BUTTONBOARDS
        //Match play buttons
    frc2::JoystickButton(&m_buttonBoardUSB2, 1).WhenPressed(&m_setLowShooterSpeedsThenFeedWhenReadyCMDGRP);  
    frc2::JoystickButton(&m_buttonBoardUSB2, 2).WhenPressed(&m_setShooterSpeedsThenFeedWhenReadyManualUpperCMDGRP);  
    frc2::JoystickButton(&m_buttonBoardUSB2, 3).WhenPressed(&m_checkBallCountCMD);  
    frc2::JoystickButton(&m_buttonBoardUSB2, 4).WhenPressed(&m_resetOdometryLimelight);
    frc2::JoystickButton(&m_buttonBoardUSB2, 5).WhenPressed(&m_cancelAllMotorsCMDGRP);
    frc2::JoystickButton(&m_buttonBoardUSB2, 6).WhenPressed(&m_ejectOneBall);  

    frc2::JoystickButton(&m_buttonBoardUSB1, 6).WhenPressed(&m_incrementShooterAdjustmentsIncrease);
    frc2::JoystickButton(&m_buttonBoardUSB1, 7).WhenPressed(&m_incrementShooterAdjustmentsDecrease);
    frc2::JoystickButton(&m_buttonBoardUSB1, 5).WhenPressed(&m_setDribbleShooterSpeedsThenFeedWhenReadyCMDGRP);
    frc2::JoystickButton(&m_buttonBoardUSB1, 8).WhenPressed(frc2::InstantCommand([this] {m_storageSUB.SetIgnoreBallColorStatus(false);})); 
    frc2::JoystickButton(&m_buttonBoardUSB1, 9).WhenPressed(frc2::InstantCommand([this] {m_storageSUB.SetIgnoreBallColorStatus(true);})); 


        //End Games Buttons
    frc2::JoystickButton(&m_buttonBoardUSB1, 1).WhenPressed(&m_automattedClimbCMDGRP);   //Test Automatted Sequence.
    frc2::JoystickButton(&m_buttonBoardUSB1, 2).WhenPressed(&m_automattedClimbSlowCMDGRP);   


  
        //-----------------WDR: to clean up buttons 3-10
    //frc2::JoystickButton(&m_buttonBoardUSB1, 1).WhenPressed(&m_climbStep1);   //Pull Hook Down all the way. End of motion magic indicates its finished
    //frc2::JoystickButton(&m_buttonBoardUSB1, 2).WhenPressed(&m_climbStep2);   //Send arms back, this pulls them against the bar.
                                                                              //Maybe 0.5 sec delay added to the end to make sure they are pressing against the bar before moving onto next step
  
    //frc2::JoystickButton(&m_buttonBoardUSB1, 3).WhenPressed(&m_climbStep3);   //Lower down/ extend climb hooks until climb hooks fall off (once arms are taking most of the weight)
                                                                              //You could put the motor into coast mode as the start of this so that is the match timer finished it might
                                                                              //continue to lower until coming off.  -----Need to Test this --------
                                                                              //End of this movement could be detected by end of motion magic move.
                                                                              //Climb arm will come back quickly which effectivitly pushes climb hooks forward under bar.  Need to 
                                                                              //check limit switches on both arms to make sure they are all the way back before moving on. 
                                                                              //ie Climb hooks have passed under the bar before moving to step 4
                                                                              //Also put the climb motor back into brake mode before moving on.
    // frc2::JoystickButton(&m_buttonBoardUSB1, 4).WhenPressed(&m_climbStep4);   //Raise climb hooks to fully extended, End of command can be detected by end of motion magic move. 
    // frc2::JoystickButton(&m_buttonBoardUSB1, 5).WhenPressed(&m_climbStep5);   //This step was send climb arms foward/out, but this puts tension on the hooks and
                                                                              //prevents them from coming off. Needs to be changed to float climb arms instead
    // frc2::JoystickButton(&m_buttonBoardUSB1, 6).WhenPressed(&m_climbStep6);   //Pulls climb hooks down slightly (to same position as Step 3)
                                                                              //Because Step 5 is being changed to float. Hooks now need to be sent forward here. Could 
                                                                              //customised so that position is checked and as it passes a point the hooks are sent forward automatically.
                                                                              //Need to make sure hooks are fully forward (via limit switch read) before going past a point 
                                                                              //where the arms will be on the wrong side of the next bar if we move on too early.
    // frc2::JoystickButton(&m_buttonBoardUSB1, 7).WhenPressed(&m_climbStep7);   //Pull climb hooks all the way down, this is effectively being back at Step 1
                                                                              //You can detect the end of this step by the end of the motion magic move.
    // frc2::JoystickButton(&m_buttonBoardUSB1, 8).WhenPressed(&m_climbStep8);   //This is the same as Step 2, you are pulling the arms back in against the bar,
                                                                              //Maybe add a 0.5 second delay to the end to make sure they are back against the back before moving on.
    // frc2::JoystickButton(&m_buttonBoardUSB1, 9).WhenPressed(&m_climbStep9);   //Same as Step3. Lower back down until hooks fall off.
                                                                              //You could potentially put the climb motor into coast here incase the match timer 
                                                                              //expires in the middle of this move.  This way it may continue to drop until the point where is falls 
                                                                              //off after the buzzer, which would give you point for the next level.
  
    frc2::JoystickButton(&m_buttonBoardUSB1, 10).WhenPressed(&m_climbStep10); //This one is only used to rest the climb hook back the initial retracted position.  Also good to use instead of Step 1 
                                                                              //When you are practicing climbs without hooking onto the bar, because it does pull down quiet as hard as step 1
   
    frc2::JoystickButton(&m_buttonBoardUSB1, 12).WhenPressed(&m_climberEnable);//This button is associated with the missle switch to the toggle switch


  #endif  //Button Boards
#endif //Swervy
}   //ConfigureButtonBindings


double RobotContainer::ApplyDeadband(double joystickValue, double deadband)
{
  if (joystickValue < -deadband)
    return (joystickValue + deadband) / (1.0 - deadband);

  if (joystickValue > deadband)
    return (joystickValue - deadband) / (1.0 - deadband);

  return 0.0;
}

void RobotContainer::StorageInitialisePreloadedBallColour()
{
  m_storageSUB.InitialisePreloadedBallColour();
}

void RobotContainer::StorageCheckBallCount()
{
  m_storageSUB.CheckBallCount();
}

//#include "RobotContainer.inc"             // Trying a simple auto approach to train the students
#include "RobotContainerSimple.inc"         // The simple old fashion auto Chooser stuff with predefined command groups is in here.

