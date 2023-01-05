#pragma once
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <utility>

#include <frc/controller/PIDController.h>
#include <frc/geometry/Translation2d.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
//#include <frc2/command/SwerveControllerCommand.h>
// MDE202210117 migrate 2022 (now SwerveControllerCommand.h works fine)
//#include "SwerveControllerCommand6508.h"  //Had to implement our own version becuase the wpilib v2021.3.1 had bugs
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/button/POVButton.h>
#include <units/angle.h>
#include <units/velocity.h>
//#include <cstdlib>
#include <fstream>
#include <cmath>

#include "Constants.h"

// Subsystems
#include "subsystems/DriveSubsystem.h"


// Commands
#include "commands/SCurveDemo.h"
#include "commands/DefaultDrive.h"
#include "commands/DriveFixedFieldBearing.h"
#include "commands/CancelLockedHeading.h"
#include "commands/RotateToHub.h"
#include <frc/shuffleboard/Shuffleboard.h>
#include "commands/SetLoaderMotorTargetPosition.h"

#include "commands/Intake/IntakeOutAndIntakeDriveInwards.h"
#include "commands/WaitForNumberOfSeconds.h"
#include "commands/Intake/IntakeIn.h"
#include "commands/Intake/IntakeStop.h"
#include "commands/SetCalculatedShooterSpeedsThenFeedWhenReady.h"
#include "commands/auto/WaitUntilGetMatchTime.h"
#include "commands/SetLowShooterSpeedsThenFeedWhenReady.h"
#include "commands/auto/AutoRHSOneThenTwoThenTwoLowLauncher.h"
#include "commands/auto/AutoControls.h"

using namespace DriveConstants;
using namespace AutoConstants;
#define MAXC  128

  // WDR: NB-----------------Moved from Constructor of Robot Container to here -----------------------------
  //make trajectories from all autonomous-<n>.txt files that we find (upto 15)
                        char filename_ca[120];
                        for (int x=1;x<16;x++) {
                          sprintf(filename_ca,"%s-%d.txt",kTrajectoryFilename,x);
                          std::fstream f(filename_ca);
                          if (f.is_open()) { 
                            f.close();
                            MakeTrajectories(x);
                            hubAimState[x] = kHubAimStateDEFAULT;
                          } 
                        }
                        
void RobotContainer::MakeTrajectories(int wayFileIndex) {
  //char fileName[] {"/home/lvuser/wayfiles/autonomous.txt"};
  double a {0.0},b {0.0},c {0.0};
  int n {0},trajIdx {0};
  char buf[MAXC];
  char *field_ptr;
  char *comma_ptr;
  auto X = units::length::meter_t(0);
  auto Y = units::length::meter_t(0);
  auto deg = units::angle::degree_t(0);
  auto lastX = units::length::meter_t(0);
  auto lastY = units::length::meter_t(0);
  auto lastDeg = units::angle::degree_t(0);
  frc::Rotation2d launchDeg;
  char filename_ca[120];
  sprintf(filename_ca,"%s-%d.txt",kTrajectoryFilename,wayFileIndex);
  std::fstream f(filename_ca);
  double maxVelocityFactor=1.0;
  double maxAccelerationFactor=1.0;
  bool reversedFlag=false;

  trajectoryCount[wayFileIndex]=0;
  commandCount[wayFileIndex]=0;

  if(f.is_open())
  {
    fmt::print("*** Making Trajectories from: {}\n",filename_ca);

    /*
     * CSV Processing
     * 1) Typically Waypoints of the form x,y,degrees example: 
     *   12.23,45.0,23.0  (Waypoint with 3 fields)
     * 2) Custom Commands can be sent by starting the line with a #
     *   #command
     */
    while (f.getline(buf, MAXC)) { /* read the whole line up to \n */
      
      /*
       * Check if this line is a... 
       *  '#' - Comment line eh #This is a comment
       *  '^' - Command line eg ^HUB_OFFSET_AIM_SECS;1.5;2.2;0.5 or HUB_AIM_SECS;0.4
       *  else - Trajectory Way Point with End Heading eg 12.8,3.5,78.4 (need at least 2 of these)
       *         Trajectory Via eg 14.2,4.7 
       *           (Via's need to start and end with a Trajectory Way Point with End Heading)
       *         - Two sequential Trajectories will launch at an angle, that the line between the 
       *           two points make to the x-axis, by default.  ^Commands can modify this behaviour.  
       */
      switch (buf[0]) { //Check for a Comment '#' or Command '^'
        case '#': //Comment -ignore the line
          fmt::print("{}\n",buf);
          continue;
        case '^': //Command - we will save the Commands, and process later.
          std::strcpy(commands[wayFileIndex][commandCount[wayFileIndex]],&buf[1]);
          fmt::print("^{} (before trajectory {}.)\n",commands[wayFileIndex][commandCount[wayFileIndex]],trajIdx);
          commandBeforeTrajectoryIndex[wayFileIndex][commandCount[wayFileIndex]]=trajIdx;
          if ((std::strlen(commands[wayFileIndex][commandCount[wayFileIndex]])>=8) && (std::strncmp(commands[wayFileIndex][commandCount[wayFileIndex]],"REVERSED",8)==0))
            reversedFlag = true;
          if ((std::strlen(commands[wayFileIndex][commandCount[wayFileIndex]])>8) && (std::strncmp(commands[wayFileIndex][commandCount[wayFileIndex]],"VELOCITY",8)==0))
            maxVelocityFactor = std::stod(&commands[wayFileIndex][commandCount[wayFileIndex]][9]);
          if ((std::strlen(commands[wayFileIndex][commandCount[wayFileIndex]])>12) && (std::strncmp(commands[wayFileIndex][commandCount[wayFileIndex]],"ACCELERATION",12)==0))
            maxAccelerationFactor = std::stod(&commands[wayFileIndex][commandCount[wayFileIndex]][13]);
          commandCount[wayFileIndex]++;

          continue;  // process another line without effecting any trajectory counters
        default: // No, this one is a waypoint, field 1
          field_ptr=buf;
      }

      // while there is another field in the line.
      while (((comma_ptr = strchr(field_ptr, ',')) != nullptr) || //if there is a comma
             ((comma_ptr == nullptr) && (strlen(field_ptr) > 0))) { //or it's the last field in line
           
        if (comma_ptr != nullptr)
          *comma_ptr=0; //replace comma with a null terminator, now field_ptr is considered a string.

        switch (n) {    /* coordinate read based on field number */
          case 0: //field 1 (of 3) Trajectory With Heading
            a = std::stod (field_ptr);
            //fmt::print("a: {}\n",a);
            break;
          case 1: //field 2 (of 3) Trajectory With Heading or field 2 of Trajectory Via eg 5.6,4.2
            b = std::stod (field_ptr);  
            //fmt::print("b: {}\n",b);

            if (comma_ptr==nullptr) { //if there is no field 3, then this is a Via 
              translations[wayFileIndex].push_back(frc::Translation2d(units::length::meter_t(a),units::length::meter_t(b)));
              fmt::print("***Via Added -- X: {} Y: {}\n",a,b);
            }
            break; 
          case 2: //field 3 - Trajectory With Heading  eg 12.0,27.0,90.5
            c = std::stod (field_ptr); //field 3
            //fmt::print("c: {}\n",c);
            fmt::print("x: {} y: {} rot: {}\n",a,b,c);

            
            
            // Set Max Speed and Max acceleration 
            trajectoryConfigs[wayFileIndex][trajIdx] = new frc::TrajectoryConfig(maxVelocityFactor*AutoConstants::kMaxSpeed,
                              maxAccelerationFactor*AutoConstants::kMaxAcceleration);

            trajectoryConfigs[wayFileIndex][trajIdx]->SetReversed(reversedFlag);
                
            // Add kinematics to ensure max speed is actually obeyed
            trajectoryConfigs[wayFileIndex][trajIdx]->SetKinematics(m_drive.kDriveKinematics);
            // Set trajectory Start and End velocity 
            // nb: >0 requires more corrections betweeen trajectories, 
            // might be OK for use between via's.
            trajectoryConfigs[wayFileIndex][trajIdx]->SetStartVelocity(kAutoTrajectoryStartVelocity);
            trajectoryConfigs[wayFileIndex][trajIdx]->SetEndVelocity(kAutoTrajectoryEndVelocity);

            X = units::length::meter_t(a);
            Y = units::length::meter_t(b);
            deg = units::angle::degree_t(c);

            //Trajectory is between two points, so we need at least 2 points to work with...
            if (trajIdx>0) {
              //launchDeg is the starting angle of trajectory (not the robot facing angle).
              launchDeg = m_drive.GetAngleToXY(lastX, lastY, X, Y);
              
              fmt::print("from X: {} from Y: {} from path angle: {} to X: {} to Y: {} to angle: {} \n",lastX,lastY,launchDeg.Degrees().value(),X,Y,deg);
              trajectories[wayFileIndex][trajIdx-1] = frc::TrajectoryGenerator::GenerateTrajectory(
                    frc::Pose2d(lastX, lastY, launchDeg),
                    translations[wayFileIndex],//{frc::Translation2d(3.817_m,1.818_m)},
                    frc::Pose2d(X,Y, frc::Rotation2d(deg)), //frc::Pose2d(3.817_m,1.818_m, frc::Rotation2d(0_deg)),
                    *trajectoryConfigs[wayFileIndex][trajIdx-1]); 

              while(translations[wayFileIndex].size())
                translations[wayFileIndex].pop_back(); //clean up any Via's
              //Reset any Command settings that were for completed trajectory.
              maxVelocityFactor=1.0;
              maxAccelerationFactor=1.0;
              reversedFlag=false;
            } else {
              autoStartPose[wayFileIndex] = frc::Pose2d(X,Y, frc::Rotation2d(deg));
            }
            lastX=X;
            lastY=Y;
            lastDeg=deg;
            trajIdx++;
            break;
          default:
            fmt::print("*** WAYPOINT HAS UNEXPECTED FIELD: {}\n",n);
        } //switch (n) //process each comma seprated field
        if (comma_ptr == nullptr) { //this was the last field in the line
          n=0;
          break; //break from field reading while loop and read another line;
        } else {
          field_ptr = comma_ptr+1;
          n++;
        }   
      } //while there is another field in the line
    } //while there are more lines in the file
  }//f.is_open
  f.close();
  if (trajIdx>0)
    trajectoryCount[wayFileIndex]=trajIdx-1;
  fmt::print("TRAJECTORY COUNT {}\n",trajectoryCount[wayFileIndex]);  
}

/*
 * Sets the next Rotation2d (angle of the robot in relation to the x-axis, the long side of field), 
 * based on the position along the active trajectory targeted for this 
 * moment in time. If a SHOOT command applies to this trajectory, it will rotate to aim at the Hub, 
 * while still traversing the trajectory, for as long as possible, and then get back to the 
 * trajectory in time to land in the final position (ready to pick up another ball).
 */
//frc::Rotation2d RobotContainer::hubAimTrajectory(){return frc::Rotation2d(180_deg);}

//frc::Rotation2d RobotContainer::hubAimTrajectory(int trajectoryIndex, units::second_t curTime)
double RobotContainer::hubAimTrajectory(int wayfileIndex,int trajectoryIndex, double curTime)
{
  frc::Pose2d currentPose = m_drive.GetPose();
  int finalStateIndex = trajectories[wayfileIndex][trajectoryIndex].States().size()-1;
  frc::Pose2d finalPose = trajectories[wayfileIndex][trajectoryIndex].States().at(finalStateIndex).pose;
  frc::Pose2d commenceFinalPose = trajectories[wayfileIndex][trajectoryIndex].States().at(finalStateIndex).pose;
  double distanceToFinalPose=0, commenceFinalPoseIndex=finalStateIndex;

  /*
   * Get the index of the currentState of the trajectory
   * theoretically more accurate way to get currentStateIndex, but didn't work? 
  int currentStateIndex=finalStateIndex;
  frc::Trajectory::State currentState = trajectories[trajectoryIndex].Sample(units::second_t(curTime));  
  for (int x = 0; x<finalStateIndex; x++){
     if (trajectories[trajectoryIndex].States().at(x).operator==(currentState)) {
       currentStateIndex = x;
       fmt::print("*** CURRENT STATE FOUND curTime: {}, currentStateIndex: {}\n",curTime,currentStateIndex);
       break;
     }  
  }  
   */

  /*
   * Estimate the trajectory, currentStateIndex, that we are currently at (ie closest to)
   */
  double distanceToPose,lastDistance {9999.0};
  int currentStateIndex=finalStateIndex;
 
  for (int x=0; x<=finalStateIndex; x++)
  {
    distanceToPose = sqrt((pow(trajectories[wayfileIndex][trajectoryIndex].States().at(x).pose.X().value()-currentPose.X().value(),2.0)+
                           pow(trajectories[wayfileIndex][trajectoryIndex].States().at(x).pose.Y().value()-currentPose.Y().value(),2.0)));
    if (distanceToPose<lastDistance)
      currentStateIndex=x;
    lastDistance=distanceToPose;
  }
    
  /*
   * Check if there is a Command (HUB_AIM_SECS,HUB_OFFSET_AIM_SECS) for this trajectory.
   */
  bool hubAimFlag=false;
  char *ptr,*ptr2;
  char secs_ca[20];
  char offsetX_ca[20];
  char offsetY_ca[20];
  double secs_l,offsetX_l,offsetY_l;
  offsetX_l=0;
  offsetY_l=0;
  secs_l=0;
  for (int commandIdx=0;commandIdx<commandCount[wayfileIndex];commandIdx++){      
    if (commandBeforeTrajectoryIndex[wayfileIndex][commandIdx]==trajectoryIndex) {
      if (strncmp("HUB_AIM",commands[wayfileIndex][commandIdx],7)==0){
        if (hubAimState[wayfileIndex]!=kHubAimStateHUB_AIM) {
          fmt::print("HUB_AIM,,,,,,,,,,,\n");
          hubAimState[wayfileIndex]=kHubAimStateHUB_AIM;
        }
        hubAimFlag=true;
        break;  //only one directional command per trajectory, take first
      }
      //eg HUB_AIM_SECS;0.3,
      if (strncmp("HUB_AIM_SECS",commands[wayfileIndex][commandIdx],12)==0){
        ptr=strchr(commands[wayfileIndex][commandIdx],';');//ptr = colon before offsetX
        if (ptr) {
          strcpy(secs_ca,ptr+1);
          secs_l = std::stod(secs_ca);
          if (curTime>secs_l) {
            if (hubAimState[wayfileIndex]!=kHubAimStateHUB_AIM_SECS) {
              //XXXfmt::print("HUB_AIM_SECS,{:f10.5},,,,,,,,,,\n",secs_ca);
              hubAimState[wayfileIndex]=kHubAimStateHUB_AIM_SECS;
            }
            hubAimFlag=true;
          }//secs of this trajectory has passed
        }//if there was a comma after the HUB_AIM_SECS field
        break; //only one directional command per trajectory, take first
      }

      //eg HUB_OFFSET_AIM_SECS;1.1;0.4;2.3,
      if (strncmp("HUB_OFFSET_AIM_SECS",commands[wayfileIndex][commandIdx],12)==0){
        ptr=strchr(commands[wayfileIndex][commandIdx],';'); //ptr = colon before offsetX
        if (ptr) {
          ptr2=strchr(ptr+1,';'); //ptr2 = colon after offsetX
          if (ptr2) {
            strncpy(offsetX_ca,ptr+1,ptr2-ptr-1);
            offsetX_ca[ptr2-ptr-1]=0;
            offsetX_l = std::stod(offsetX_ca);
            //fmt::print("***OFFSET-X:{} OFFSET-X_CA:{}",offsetX_l,offsetX_ca);
            ptr=strchr(ptr2+1,';');//ptr = colon after offsetY
            if (ptr) {
              strncpy(offsetY_ca,ptr2+1,ptr-ptr2-1);
              offsetY_ca[ptr-ptr2-1]=0;
              offsetY_l = std::stod(offsetY_ca);
              //fmt::print("***OFFSET-Y:{} OFFSET-Y_CA:{}",offsetY_l,offsetY_ca);
              strcpy(secs_ca,ptr+1);
              secs_l = std::stod(secs_ca);
              //fmt::print("***SECS:{} SECS_CA:{}",secs_l,secs_ca);
              if (curTime>secs_l) {
                if (hubAimState[wayfileIndex]!=kHubAimStateHUB_OFFSET_AIM_SECS) {
                  //fmt::print("HUB_OFFSET_AIM_SECS,{:f10.4},{:f10.4},{:f10.4},,,,,,,,\n",offsetX_ca,offsetY_ca,secs_ca);
                  hubAimState[wayfileIndex]=kHubAimStateHUB_OFFSET_AIM_SECS;
                }
                hubAimFlag=true;
              }//secs of this trajectory has passed
            }//if there was a colon after offsetY
          }//if there was a colon after the offsetX field
        }//if there was a colon before the offsetX field
        break; //only one directional command per trajectory, take first
      }

    }//if the command applies to the current trajectory
  }//for each command (up to commandCount)

  /*
   * If we are not planning to Shoot the ball on this trajectory, then rotate the nose 
   * towards the trajectory  (auto trajecties are mostly heading towards balls)
   */
  if (!hubAimFlag) {
    //rotate towards path heading straight away
    //rotation2d = trajectories[trajectoryIndex].Sample(units::second_t(curTime)).pose.Rotation();
    rotation2d[wayfileIndex] = trajectories[wayfileIndex][trajectoryIndex].States().back().pose.Rotation();
    hubAimState[wayfileIndex]=kHubAimStateDEFAULT;
    lastHubAimState[wayfileIndex]=hubAimState[wayfileIndex];
    return rotation2d[wayfileIndex].Degrees().value();
  }
    
  //fmt::print("hubAimTrajectory - ");
  
  /*
   * Distance to Final Pose
   * Determine where in the trajectory to start going back to final pose.
   *
   * Calculate the best state index, finalStateIndex, to start returning from Aiming at hub to final pose
   * It is based on the actual distance travelled along the trajectory from end to kCommenceFinalPoseDistance
   * CHANGE: kCommenceFinalPoseDistance - change, if we are not getting to the final pose position
   */
  for (int x=finalStateIndex-1;x>0;x--) //starts at second last so that we can compare to last on first loop.
  {
    //d = √[(x2−x1)2 + (y2−y1)2]
    distanceToFinalPose += sqrt((pow(trajectories[wayfileIndex][trajectoryIndex].States().at(x).pose.X().value()-trajectories[wayfileIndex][trajectoryIndex].States().at(x+1).pose.X().value(),2.0)+
                           pow(trajectories[wayfileIndex][trajectoryIndex].States().at(x).pose.Y().value()-trajectories[wayfileIndex][trajectoryIndex].States().at(x+1).pose.Y().value(),2.0)));
    if (distanceToFinalPose > kCommenceFinalPoseDistance) {
      commenceFinalPoseIndex = x;
      commenceFinalPose = trajectories[wayfileIndex][trajectoryIndex].States().at(commenceFinalPoseIndex).pose;
      #ifdef DEBUG 
        fmt::print("*** distanceToFinalPose: {}, commenceFinalPoseIndex: {}\n",distanceToFinalPose,commenceFinalPoseIndex);
      #endif
      break;
    }
  }

  /*
   * calculate the distance from currentPose X,Y to commenceFinalPose X,Y.
   *
  double distanceToCommenceFinalPose = sqrt((pow(commenceFinalPose.X().value()-currentPose.X().value(),2.0)+
                                 pow(commenceFinalPose.Y().value()-currentPose.Y().value(),2.0)));
  int currentStateIndex=trajectory.States().size()-1;
  
   */
  

  
  if ((hubAimState[wayfileIndex]==kHubAimStateHUB_AIM) || (currentStateIndex < commenceFinalPoseIndex)) //aim at hub (or hub offset) for as long as possible
  {
    /*
     * Aim at Hub, 
     * from the start of the trajectory until we get to the State index commenceFinalPositionIndex
     * assume HUB Center is at X, Y: 8.05, 4.12 and 
     * calculate desired rotation2d to aim at Hub based on current X,Y compared to X,Y positon of Hub
     */
    double distanceToHUBOffset;
    rotation2d[wayfileIndex] = m_drive.GetAngleToHUBOffset(
                          trajectories[wayfileIndex][trajectoryIndex].States().at(currentStateIndex).velocity(),
                          trajectories[wayfileIndex][trajectoryIndex].States().at(currentStateIndex).pose.Rotation().Degrees().value(),
                          &distanceToHUBOffset);

    m_drive.SetDistanceToHUBOffset(distanceToHUBOffset); //parallel shoot commands read this

    //rotation2d = m_drive.GetAngleToHUB();
    /*m_drive.GetAngleToXY(currentPose.X(), 
                              currentPose.Y(), 
                              units::length::meter_t(kHubX+offsetX_l), 
                              units::length::meter_t(kHubY+offsetY_l));*/
    if (hubAimState!=lastHubAimState){
      fmt::print ("AIM AT HUB,{},{},{}of{},          ,          ,          ,          ,          ,{},{},      {}\n",
                  offsetX_l,
                  offsetY_l,
                  currentStateIndex,
                  commenceFinalPoseIndex,
                  rotation2d[wayfileIndex].Degrees().value(),
                  curTime,
                  trajectoryIndex
                  );
      fmt::print ("  seconds , current X, current Y,curr. rot.,next veloc,  next X  ,  next Y  , next rot.,final rot.,ovrde rot.,traj. time, traj. idx\n");

    }
    
  }
  else
  { 
      /*
       * Now it's time to go towards Final Pose (just follow the trajectory)
       */
      rotation2d[wayfileIndex] = trajectories[wayfileIndex][trajectoryIndex].States().at(currentStateIndex).pose.Rotation();      //or rotation of final state
      if (hubAimState[wayfileIndex]!=lastHubAimState[wayfileIndex]) {
        hubAimState[wayfileIndex] = kHubAimStateFINAL_POSE;     
        //XXXfmt::print ("FINAL POSE,          ,          ,{4d}of{4d},          ,          ,          ,          ,          ,{:f10.4},{:f10.4},      {4d}\n",
        fmt::print ("FINAL POSE,          ,          ,{}of{},          ,          ,          ,          ,          ,{},{},      {}\n",
        
                  currentStateIndex,
                  finalStateIndex,
                  rotation2d[wayfileIndex].Degrees().value(),
                  curTime,
                  trajectoryIndex
                  );
        fmt::print ("  seconds , current X, current Y,curr. rot.,next veloc,  next X  ,  next Y  , next rot.,final rot.,ovrde rot.,traj. time, traj. idx\n");
      }
  }
  lastHubAimState[wayfileIndex]=hubAimState[wayfileIndex];
  return rotation2d[wayfileIndex].Degrees().value();
    
}




frc2::Command *RobotContainer::GetAutonomousCommand()
{
  if (wayfileIndex == 6){
    m_drive.ForcedResetOdometry(frc::Pose2d(kStartingPosition6x, kStartingPosition6y, frc::Rotation2d(units::angle::degree_t(136.5))));
    return &m_intakeInCMD;
  }
  if (wayfileIndex == 7){
    m_drive.ForcedResetOdometry(frc::Pose2d(units::meter_t(5.89), units::meter_t(4.7), frc::Rotation2d(units::angle::degree_t(-180.0))));
    return &m_intakeInCMD;
  }
  if (wayfileIndex == 8){
    m_drive.ForcedResetOdometry(frc::Pose2d(units::meter_t(7.59), units::meter_t(1.77), frc::Rotation2d(units::angle::degree_t(-90.0))));
    return &m_intakeInCMD;
  }
  if (wayfileIndex == 9){
    m_drive.ForcedResetOdometry(frc::Pose2d(units::meter_t(6.67), units::meter_t(4.71), frc::Rotation2d(units::angle::degree_t(159.0))));
    return &m_setLowShooterSpeedsThenFeedWhenReadyCMDGRP;
  }
  if (wayfileIndex == 10){
    m_drive.ForcedResetOdometry(frc::Pose2d(units::meter_t(7.64), units::meter_t(2.59), frc::Rotation2d(units::angle::degree_t(-111.0))));
    return &m_setLowShooterSpeedsThenFeedWhenReadyCMDGRP;
  }
  if (wayfileIndex == 14){
    m_drive.ForcedResetOdometry(frc::Pose2d(units::meter_t(5.89), units::meter_t(4.7), frc::Rotation2d(units::angle::degree_t(-180.0))));
    return &m_setLowShooterSpeedsThenFeedWhenReadyCMDGRP;
  }
  if (wayfileIndex == 15){
    m_drive.ForcedResetOdometry(frc::Pose2d(units::meter_t(7.59), units::meter_t(1.77), frc::Rotation2d(units::angle::degree_t(-90.0))));
    return &m_setLowShooterSpeedsThenFeedWhenReadyCMDGRP;
  }
  

  auto scgControls = new frc2::SequentialCommandGroup();
  auto scgWaypoints = new frc2::SequentialCommandGroup();
  auto pcg = new frc2::ParallelCommandGroup();

  frc::ProfiledPIDController<units::radians> thetaController{
      AutoConstants::kPThetaController, 0, 0,
      AutoConstants::kThetaControllerConstraints};
  
  m_drive.ForcedResetOdometry(autoStartPose[wayfileIndex]);
  fmt::print("Resetting Odometry at first waypoint X:{} Y:{} Rot:{}\n",
             autoStartPose[wayfileIndex].X().value(),
             autoStartPose[wayfileIndex].Y().value(),
             autoStartPose[wayfileIndex].Rotation().Degrees().value());

  thetaController.Reset((units::radian_t(m_drive.GetHeading())));
  thetaController.EnableContinuousInput(units::radian_t(-wpi::numbers::pi),
                                      units::radian_t(wpi::numbers::pi));
  
   
  /*
   * We can schedule any Commands here, to be activated before the trajectory that they apply to.
   */
  fmt::print("*** WAYFILEINDEX: {} trajectoryCount: {} \n",wayfileIndex,trajectoryCount[wayfileIndex]);
  int x;
  for (x=0;x<=(trajectoryCount[wayfileIndex]+1);x++)//because we check for commands after the last trajectoryCount
  {
    /*
     * Schedule any Commands required before the next Trajectory
     */ 
    fmt::print("commandCount: {}\n",commandCount[wayfileIndex]);
    for (int commandIdx=0;commandIdx<commandCount[wayfileIndex];commandIdx++){ 
      if (commandBeforeTrajectoryIndex[wayfileIndex][commandIdx]==x) {
        fmt::print("Command Index: {} command: {}\n",commandIdx,commands[wayfileIndex][commandIdx]);
        /*
         * Parse any Commands and them to the scheduler
         */

        /*
         * Waypoint Commands
         */
        if (strncmp("STOP",commands[wayfileIndex][commandIdx],4)==0)
          //m_autoWaypoints.AddCommands(frc2::InstantCommand(frc2::InstantCommand(
           scgWaypoints->AddCommands(frc2::InstantCommand(frc2::InstantCommand(    
            [this]() {
              m_drive.Drive(units::meters_per_second_t(0),
                          units::meters_per_second_t(0),
                          units::radians_per_second_t(0), false);
            },{})));
        if ((strlen(commands[wayfileIndex][commandIdx]) >= 17) && (strncmp("W_AWAIT_COUNTDOWN",commands[wayfileIndex][commandIdx],17)==0)) {
          const auto wAwaitCountdown = units::time::second_t(std::stod(&commands[wayfileIndex][commandIdx][18]));
          scgWaypoints->AddCommands(WaitUntilGetMatchTime(wAwaitCountdown, &m_drive)); // Wait until we should have picked up two balls (after having shot first ball)
         fmt::print("Adding W_AWAIT_COUNTDOWN\n");
        }
        
        /*
         * Control Commands
         * eg AutoRHSOneThenTwoThenTwoLow
         * 
         * ^SHOOT_LOW
         * ^INTAKE_OUT            - IntakeIn/IntakeStop - three commands that all finish immediately
         * ^AWAIT_COUNTDOWN;10.40 - Wait until we should have picked up two balls (after having shot first ball)
         * ^INTAKE_IN
         * ^AWAIT_COUNTDOWN;9.82  - Wait until our shooter should be angled towards the target
         * ^SHOOT_LOW
         * ^AWAIT_COUNTDOWN;7.62  - Wait until we should be approaching next two balls
         * ^INTAKE_OUT           
         * ^AWAIT_COUNTDOWN;4.2   - Wait until we should have picked up last two balls (after having shot three balls)
         * ^INTAKE_IN
         * ^AWAIT_COUNTDOWN;1.62  - Wait until our shooter should be angled towards the target
         * ^SHOOT_LOW
         * 
         * 
   nits::time::second_t(10.4085); // intake When 10.0 seconds remains in Autonomous, we should have picked up two balls
    Until = units::time::second_t(9.821794); // shoot When 8.0 seconds remains in Autonomous, we should have moved and be ready to shoot
    c = units::time::second_t(7.621426); // intake down/spin When 3.0 seconds remains in Autonomous, we should be approaching the last two balls
    il = units::time::second_t(4.205843); // When 2.0 seconds remains in Autonomous, we should have picked up last two balls
     = units::time::second_t(1.62168); // When 0.5 seconds remains in Autonomous, we should have moveds and be ready to shoot

         * AddCommands equivalent:
         *  SetLowShooterSpeedsThenFeedWhenReady(m_shooterSUB, m_storageSUB), // , [this] {return kBottomFenderLowSpeed;}, [this] {return kTopFenderLowSpeed;}
         *  IntakeOutAndIntakeDriveInwards(m_intakeSUB, m_storageSUB, kLoaderMotorTestSpeed), // This command group has three commands that all finish immediately
         *  WaitUntilGetMatchTime(m_firstWaitUntil), // Wait until we should have picked up two balls (after having shot first ball)
         *  IntakeIn(m_intakeSUB),
         *  IntakeStop(m_intakeSUB),
         *  WaitUntilGetMatchTime(m_secondWaitUntil), // Wait until our shooter should be angled towards the target
         *  SetLowShooterSpeedsThenFeedWhenReady(m_shooterSUB, m_storageSUB), // , [this] {return kBottomFenderLowSpeed;}, [this] {return kTopFenderLowSpeed;}
         *  WaitUntilGetMatchTime(m_thirdWaitUntil), // Wait until we should be approaching next two balls
         *  IntakeOutAndIntakeDriveInwards(m_intakeSUB, m_storageSUB, kLoaderMotorTestSpeed), // This command group has three commands that all finish immediately
         *  WaitUntilGetMatchTime(m_fourthWaitUntil), // Wait until we should have picked up last two balls (after having shot three balls)
         *  IntakeIn(m_intakeSUB),
         *  IntakeStop(m_intakeSUB),
         *  WaitUntilGetMatchTime(m_fifthWaitUntil), // Wait until our shooter should be angled towards the target
         *  SetLowShooterSpeedsThenFeedWhenReady(m_shooterSUB, m_storageSUB) // , [this] {return kBottomFenderLowSpeed;}, [this] {return kTopFenderLowSpeed;}
         * 
         */

        if ((strlen(commands[wayfileIndex][commandIdx]) >= 10) && (strncmp("INTAKE_OUT",commands[wayfileIndex][commandIdx],10)==0)) {
          fmt::print("Adding INTAKE_OUT\n");
          scgControls->AddCommands(IntakeOutAndIntakeDriveInwards(&m_intakeSUB, &m_storageSUB, kLoaderMotorTestSpeed)); // This command group has three commands that all finish immediately
        }
        if ((strlen(commands[wayfileIndex][commandIdx]) >= 15) && (strncmp("AWAIT_COUNTDOWN",commands[wayfileIndex][commandIdx],15)==0)) {
          const auto awaitCountdown = units::time::second_t(std::stod(&commands[wayfileIndex][commandIdx][16]));
          scgControls->AddCommands(WaitUntilGetMatchTime(awaitCountdown, &m_drive)); // Wait until we should have picked up two balls (after having shot first ball)
          fmt::print("Adding AWAIT_COUNTDOWN;{}\n",awaitCountdown);
        }
        if ((strlen(commands[wayfileIndex][commandIdx]) >= 9) && (strncmp("INTAKE_IN",commands[wayfileIndex][commandIdx],9)==0)) {
          scgControls->AddCommands(IntakeIn(&m_intakeSUB));
          fmt::print("Adding INTAKE_IN\n");
          scgControls->AddCommands(IntakeStop(&m_intakeSUB));
          fmt::print("Adding INTAKE_STOP\n");
        }
        if ((strlen(commands[wayfileIndex][commandIdx]) >= 9) && (strncmp("SHOOT_LOW",commands[wayfileIndex][commandIdx],9)==0)) {
          scgControls->AddCommands(SetLowShooterSpeedsThenFeedWhenReady(&m_shooterSUB, &m_storageSUB)); // , [this] {return kBottomFenderLowSpeed;}, [this] {return kTopFenderLowSpeed;}
          fmt::print("Adding SHOOT_LOW\n");
        }
        if ((strlen(commands[wayfileIndex][commandIdx]) >= 10) && (strncmp("SHOOT_HIGH",commands[wayfileIndex][commandIdx],10)==0)) {
           scgControls->AddCommands(SetShooterSpeedsThenFeedWhenReady(&m_shooterSUB, &m_storageSUB, [this] {return kTopManualUpperSpeed;}, [this] {return kBottomManualUpperSpeed;}));
           fmt::print("Adding SHOOT_HIGH\n");
        }
        if ((strlen(commands[wayfileIndex][commandIdx]) >= 8) && (strncmp("SHOOT_H1",commands[wayfileIndex][commandIdx],8)==0)) {
           scgControls->AddCommands(SetShooterSpeedsThenFeedWhenReady(&m_shooterSUB, &m_storageSUB, [this] {return kTopManualUpperSpeedH1;}, [this] {return kBottomManualUpperSpeedH1;}));
           fmt::print("Adding SHOOT_H1\n");
        }
        if ((strlen(commands[wayfileIndex][commandIdx]) >= 8) && (strncmp("SHOOT_H2",commands[wayfileIndex][commandIdx],8)==0)) {
           scgControls->AddCommands(SetShooterSpeedsThenFeedWhenReady(&m_shooterSUB, &m_storageSUB, [this] {return kTopManualUpperSpeedH2;}, [this] {return kBottomManualUpperSpeedH2;}));
           fmt::print("Adding SHOOT_H2\n");
        }
         if ((strlen(commands[wayfileIndex][commandIdx]) >= 8) && (strncmp("SHOOT_H3",commands[wayfileIndex][commandIdx],8)==0)) {
           scgControls->AddCommands(SetShooterSpeedsThenFeedWhenReady(&m_shooterSUB, &m_storageSUB, [this] {return kTopManualUpperSpeedH3;}, [this] {return kBottomManualUpperSpeedH3;}));
           fmt::print("Adding SHOOT_H3\n");
        }
        if ((strlen(commands[wayfileIndex][commandIdx]) >= 10) && (strncmp("SHOOT_CALC",commands[wayfileIndex][commandIdx],10)==0)) {       
           scgControls->AddCommands(CalcShoot(&m_shooterSUB, &m_storageSUB, &m_drive));
           fmt::print("Adding SHOOT_CALC\n");
        }


         /*
          * Queue up any critical commands to be started, assuming that they will be
          * finalizaed with an action at the appropriate point in the appropriate trajectory.
          */   
  
      }
      if (commandBeforeTrajectoryIndex[wayfileIndex][commandIdx]>x) 
        break;
    }

    if (x < trajectoryCount[wayfileIndex]){
      fmt::print("Trajectory Index: {} x: {} y: {} launch angle {} to X:{} to Y: {} final angle: {}\n",
              x,
              trajectories[wayfileIndex][x].States().front().pose.X().value(),
              trajectories[wayfileIndex][x].States().front().pose.Y().value(),
              trajectories[wayfileIndex][x].States().front().pose.Rotation().Degrees().value(),
              trajectories[wayfileIndex][x].States().back().pose.X().value(),
              trajectories[wayfileIndex][x].States().back().pose.Y().value(),
              trajectories[wayfileIndex][x].States().back().pose.Rotation().Degrees().value());
      frc::SmartDashboard::PutNumber("Trajectory total time",trajectories[wayfileIndex][x].TotalTime().value());
      //MDE20220203  position monitoring
      // Send Field2d to SmartDashboard.
      #ifdef DEBUG_WITH_GLASS
      frc::SmartDashboard::PutData(&m_drive.m_field);
    
      // Reset the drivetrain's odometry to the starting pose of the trajectory.
      //m_drive.ResetOdometry(exampleTrajectory.InitialPose());
      // Send our generated trajectory to Field2d.
      
      m_drive.m_field.GetObject("traj")->SetTrajectory(trajectories[wayfileIndex][x]);
      #endif
  


    } //if x < trajectoryCount
  } //x loop that goes beyond trajectories by 2
  //m_autoWaypoints.AddCommands(std::move(*swerveControllerCommands[x]));

  /* ensure that we stop after final trajectory */
  //scg->AddCommands(m_autoRHSOneThenTwoThenTwoLowCMDGRP);
  //return &m_autoRHSOneThenTwoThenTwoLowLauncherCMDGRP;
  //scg->AddCommands(WaitForNumberOfSeconds(units::time::second_t(0.0)));
  //pcg->AddCommands(SetFeedMotorSpeed(&m_storageSUB,0.2));
  //pcg->AddCommands(EjectIntakeAndLoader(&m_intakeSUB,&m_storageSUB,units::time::second_t(3.3)));
  //pcg->AddCommands(SetCalculatedShooterSpeedsThenFeedWhenReady(&m_shooterSUB,&m_storageSUB,&m_drive));
  /*pcg->AddCommands(AutoRHSOneThenTwoThenTwoLow(&m_intakeSUB, &m_storageSUB, &m_shooterSUB, 
                  kAutoRHSOneThenTwoThenTwoLowFirstWaitUntil, 
                  kAutoRHSOneThenTwoThenTwoLowSecondWaitUntil,
                  kAutoRHSOneThenTwoThenTwoLowThirdWaitUntil,
                  kAutoRHSOneThenTwoThenTwoLowFourthWaitUntil,
                  kAutoRHSOneThenTwoThenTwoLowFifthWaitUntil,
                  &m_limelightSUB));*/
  

  /*
  * Finally we will add all of the trajectories in one command, for smooth traversal with
  * minimal loss of odometry updates.  [nb: it subsequently turns out that only
  * via's generate a super smooth transition between trajectories].  But we have 
  * a SwerveControllerCommand that can handle an array of trajectories now.
  * 
  * hubAimTrajectory will get regular progress reports
  * of the current trajectory and the trajectory timer value at that point.  It will be able to
  * look at commands to know if a hubAim is required.  It will also be able to set any flags to finalize
  * any other timely commands that may have been initiated, but are waiting on a point in the trajectory 
  * to be reached and the important action to be commenced.
  */
  swerveControllerCommands[wayfileIndex][0] = new frc2::SwerveControllerCommand<4>(
      &trajectories[wayfileIndex][0], /* nb: all the trajectories, in one command */
      trajectoryCount[wayfileIndex],
      wayfileIndex,
      [this]() { return m_drive.GetPose(); }, //where we think we are x,y/angle facing
      m_drive.kDriveKinematics,
      frc2::PIDController(AutoConstants::kPXController, 0, 0),
      frc2::PIDController(AutoConstants::kPYController, 0, 0), 
      thetaController,
      //[this]() { return frc::Rotation2d(180_deg);},//function provide desired angle at important locations
      //[this,hubAimFlag](int trajectoryIndex, units::second_t curTime) { return (hubAimFlag?hubAimTrajectory(trajectoryIndex,curTime):trajectories[trajectoryIndex].Sample(curTime).pose.Rotation());},
  //[this](int trajectoryIndex, units::second_t curTime) -> frc::Rotation2d { return hubAimTrajectory(trajectoryIndex,curTime); } ,
  [this]( auto wayfileIndex, auto trajectoryIndex, auto curTime) ->double { return hubAimTrajectory(wayfileIndex,trajectoryIndex,curTime); } ,
//[this]() { return hubAimTrajectory(); } ,
  //[this](auto test) { /*fmt::print("{}",test);*/},
      [this](auto moduleStates) { m_drive.SetModuleStates(moduleStates); },
      {&m_drive}
      );

  //m_drive.ResetOdometry(frc::Pose2d(kStartX, kStartY, frc::Rotation2d(kStartRotation)));
  /*JUST MOVED 202203121308 up before resetTheta
  m_drive.ForcedResetOdometry(autoStartPose[wayfileIndex]);
  fmt::print("Resetting Odometry at first waypoint X:{} Y:{} Rot:{}\n",
             autoStartPose[wayfileIndex].X().value(),
             autoStartPose[wayfileIndex].Y().value(),
             autoStartPose[wayfileIndex].Rotation().Degrees().value());
  */
  /*
  * Schedule the array of Trajectories
  * nb: - Two Way points in the text file make one trajectory.  
  *     - The next  trajectry starts with the last Waypoint.
  *     - Via's must be between two Waypoints and go into making one trajectory.
  */
  scgWaypoints->AddCommands(std::move(*swerveControllerCommands[wayfileIndex][0]));
  scgWaypoints->AddCommands(frc2::InstantCommand(
          [this]() {
            m_drive.Drive(units::meters_per_second_t(0),
                          units::meters_per_second_t(0),
                          units::radians_per_second_t(0), false);
          },{})); 
  
  pcg->AddCommands(std::move(*scgControls));
  pcg->AddCommands(std::move(*scgWaypoints)); 
  return pcg;
  //return &m_autoLauncher;
  // return &m_autoRHSOneThenTwoCMDGRP;
  // return &m_autoRHSTwoThenOneCMDGRP;
  // return &m_autoRHSOneThenTwoLauncherCMDGRP;
  // return &m_autoRHSTwoThenOneLauncherCMDGRP;
  
}
