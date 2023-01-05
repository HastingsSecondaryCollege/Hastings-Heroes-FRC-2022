// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <memory>
#include <utility>

#include "SwerveControllerCommand6508.h"
#include "Constants.h"

namespace frc2
{
  template <size_t NumModules>
  SwerveControllerCommand<NumModules>::SwerveControllerCommand(
      frc::Trajectory trajectory, std::function<frc::Pose2d()> pose,
      frc::SwerveDriveKinematics<NumModules> kinematics,
      frc2::PIDController xController, frc2::PIDController yController,
      frc::ProfiledPIDController<units::radians> thetaController,
      std::function<frc::Rotation2d()> desiredRotation,
      std::function<void(std::array<frc::SwerveModuleState, NumModules>)> output,
      std::initializer_list<Subsystem *> requirements)
      : m_trajectory(std::move(trajectory)),
        m_trajectories(nullptr),
        m_pose(std::move(pose)),
        m_kinematics(kinematics),
        m_controller(xController, yController, thetaController),
        m_desiredRotation(std::move(desiredRotation)),
        m_outputStates(output)
  {
    this->AddRequirements(requirements);
  }

  template <size_t NumModules>
  SwerveControllerCommand<NumModules>::SwerveControllerCommand(
      frc::Trajectory trajectory, std::function<frc::Pose2d()> pose,
      frc::SwerveDriveKinematics<NumModules> kinematics,
      frc2::PIDController xController, frc2::PIDController yController,
      frc::ProfiledPIDController<units::radians> thetaController,
      std::function<void(std::array<frc::SwerveModuleState, NumModules>)> output,
      std::initializer_list<Subsystem *> requirements)
      : m_trajectory(std::move(trajectory)),
        m_trajectories(nullptr),
        m_pose(std::move(pose)),
        m_kinematics(kinematics),
        m_controller(xController, yController, thetaController),
        m_outputStates(output)
  {
    this->AddRequirements(requirements);
    //m_desiredRotation = [&] {
    //  return m_trajectory.States().back().pose.Rotation();
    //};
  }

  template <size_t NumModules>
  SwerveControllerCommand<NumModules>::SwerveControllerCommand(
      frc::Trajectory trajectory, std::function<frc::Pose2d()> pose,
      frc::SwerveDriveKinematics<NumModules> kinematics,
      frc2::PIDController xController, frc2::PIDController yController,
      frc::ProfiledPIDController<units::radians> thetaController,
      std::function<frc::Rotation2d()> desiredRotation,
      std::function<void(std::array<frc::SwerveModuleState, NumModules>)> output,
      //MDE20220117 migrated 2022
      //wpi::ArrayRef<Subsystem *> requirements
      wpi::span<Subsystem *> requirements)
      : m_trajectory(std::move(trajectory)),
        m_trajectoryIndex(0),
        m_trajectories(nullptr),
        m_pose(std::move(pose)),
        m_kinematics(kinematics),
        m_controller(xController, yController, thetaController),
        m_desiredRotation(std::move(desiredRotation)), //added desiredRotation
        m_outputStates(output)
  {
    this->AddRequirements(requirements);
  }

//MATT - support trajectories array
  template <size_t NumModules>
  SwerveControllerCommand<NumModules>::SwerveControllerCommand(
      frc::Trajectory trajectories[20],
      int trajectoryCount,
      int wayFileIndex,
      std::function<frc::Pose2d()> pose,
      frc::SwerveDriveKinematics<NumModules> kinematics,
      frc2::PIDController xController, frc2::PIDController yController,
      frc::ProfiledPIDController<units::radians> thetaController,
      //std::function<frc::Rotation2d(int, units::second_t)> desiredRotation,
      std::function<double(int, int, double)> desiredRotation,
      //std::function<frc::Rotation2d()> desiredRotation,
      std::function<void(std::array<frc::SwerveModuleState, NumModules>)> output,
      //MDE20220117 migrated 2022
      //wpi::ArrayRef<Subsystem *> requirements
      //wpi::span<Subsystem *> requirements
      Subsystem * requirements
      )
      : m_trajectories(trajectories),
        m_trajectoryIndex(0),
        m_wayFileIndex(wayFileIndex),
        m_trajectoryCount(trajectoryCount),
        m_pose(std::move(pose)),
        m_kinematics(kinematics),
        m_controller(xController, yController, thetaController),
        m_desiredRotation(desiredRotation), //added desiredRotation
        m_outputStates(output)
  {
    this->AddRequirements(requirements);
  }

  template <size_t NumModules>
  SwerveControllerCommand<NumModules>::SwerveControllerCommand(
      frc::Trajectory trajectory, std::function<frc::Pose2d()> pose,
      frc::SwerveDriveKinematics<NumModules> kinematics,
      frc2::PIDController xController, frc2::PIDController yController,
      frc::ProfiledPIDController<units::radians> thetaController,
      std::function<void(std::array<frc::SwerveModuleState, NumModules>)> output,
      //MDE20220117 migrated 2022
      //wpi::ArrayRef<Subsystem *> requirements
      Subsystem * requirements
      )
      : m_trajectory(std::move(trajectory)),
        m_pose(std::move(pose)),
        m_kinematics(kinematics),
        m_controller(xController, yController, thetaController),
        m_outputStates(output)
  {
    this->AddRequirements(requirements);
    //m_desiredRotation = [&] {
    //  return m_trajectory.States().back().pose.Rotation();
    //};
  }

  template <size_t NumModules>
  void SwerveControllerCommand<NumModules>::Initialize()
  {
    m_trajectoryIndex=0;
    m_iterations=0;
    if (m_desiredRotation == nullptr)
    {
      m_desiredRotation = [this](int a,int b,double c) ->double {return m_trajectory.States().back().pose.Rotation().Degrees().value();};
      
/*
      m_desiredRotation = [&] {
        return m_trajectory.States().back().pose.Rotation().Degrees().value();
        //return frc::Rotation2d(-180_deg);
      }*/
    }
    else
    {
      //MDE20220107 migrate 2022
      //std::cout << "--Using custom End Heading\n---";
      fmt::print("--Using custom End Heading\n---");
    }
    
    fmt::print ("  seconds , current X, current Y,curr. rot.,next veloc,  next X  ,  next Y  , next rot.,final rot.,ovrde rot.,traj. time, traj. idx\n");
    m_timer.Reset();
    m_timer.Start();
  }

  template <size_t NumModules>
  void SwerveControllerCommand<NumModules>::Execute()
  {
    if ((m_trajectories!=nullptr)  && (m_trajectoryIndex>=m_trajectoryCount)  )
      return;
    auto curTime = units::second_t(m_timer.Get());
    double currentTime=curTime.value();
    int trajIndex = m_trajectoryIndex; 
    double desiredRotationDegrees = m_desiredRotation(m_wayFileIndex,trajIndex,currentTime);
    frc::Trajectory::State m_desiredState = (m_trajectories==nullptr)?m_trajectory.Sample(curTime):
                                                 m_trajectories[m_trajectoryIndex].Sample(curTime);
    frc::Trajectory::State finalState = (m_trajectories==nullptr)?m_trajectory.States().back():
                                                 m_trajectories[m_trajectoryIndex].States().back();  
    frc::Pose2d currentPose = m_pose();
    double totalTime;
    
    m_iterations++;
    if (m_iterations%10 == 0) { //just display every half second
      if (m_trajectories==nullptr)
        totalTime=currentTime;
      else {
        totalTime=0.0;
        for (int x=0;x<m_trajectoryIndex;x++) {
          totalTime+=m_trajectories[x].TotalTime().value();
        }  
        totalTime+=currentTime;  
      }
    //fmt::print ("  seconds , current X, current Y,curr. rot.,next veloc,  next X  ,  next Y  , next rot.,final rot.,ovrde rot.,traj. time, traj. idx\n");
    //fmt::print("{:f10.4},{:f10.4},{:f10.4},{:f10.4},{:f10.4},{:f10.4},{:f10.4},{:f10.4},{:f10.4},{:f10.4},{:f10.4},{10d}\n",
    //fmt::print("{},{},{},{},{},{},{},{},{},{},{},{}\n",
    
        totalTime,
        currentPose.X().value(),
        currentPose.Y().value(),
        currentPose.Rotation().Degrees().value(),
        m_desiredState.velocity.value(),
        m_desiredState.pose.X().value(),
        m_desiredState.pose.Y().value(),
        m_desiredState.pose.Rotation().Degrees().value(),
        finalState.pose.Rotation().Degrees().value(),
        desiredRotationDegrees,
        currentTime,
        m_trajectoryIndex
        );
    }
      /*fmt::print("current pose X: {} Y: {} rotation: {} -- next traj. state X: {}  Y: {} rotation: {} -- desired END rotation: {} -- currentTime: {}\n",
        currentPose.X().value(),
        currentPose.Y().value(),
        currentPose.Rotation().Degrees().value(),
        m_desiredState.pose.X().value(),
        m_desiredState.pose.Y().value(),
        m_desiredState.pose.Rotation().Degrees().value(),
        desiredRotationDegrees,
        currentTime
        );*/    
    //std::cout << m_desiredState.pose.X() << "," << m_desiredState.pose.Y() << "," << m_desiredState.pose.Rotation().Degrees().value() << "," << m_pose().Rotation().Degrees().value() << ",";
   // auto targetChassisSpeeds =
     //   m_controller.Calculate(m_pose(), m_desiredState, m_desiredRotation(m_trajectoryIndex,currentTime));//think Calculate(m_drive.GetPose(),m_desired_state, hubAimTrajectory(m_trajectoryIndex)
  

    auto targetChassisSpeeds =
        m_controller.Calculate(currentPose, m_desiredState,frc::Rotation2d(units::degree_t(desiredRotationDegrees)));//think Calculate(m_drive.GetPose(),m_desired_state, hubAimTrajectory(m_trajectoryIndex)
    
    auto targetModuleStates =
        m_kinematics.ToSwerveModuleStates(targetChassisSpeeds);

    m_outputStates(targetModuleStates); //think m_drive.SetModuleStates(targetModuleStates)
  }

  template <size_t NumModules>
  void SwerveControllerCommand<NumModules>::End(bool interrupted)
  {
    m_timer.Stop();
    
  }

  template <size_t NumModules>
  bool SwerveControllerCommand<NumModules>::IsFinished()
  {
    /*
     * For a trajectory array, reset the timer between each trajectory,
     * otherwise it's just the normal single trajectory timer.
     */
    if ((m_trajectories!=nullptr)  &&
        m_trajectoryIndex < m_trajectoryCount-1 &&
        (m_timer.HasElapsed(m_trajectories[m_trajectoryIndex].TotalTime()))) {
      //fmt::print("**** COMPLETED TRAJECTORY: trajectoryIndex: {}, that's {} of {}\n",m_trajectoryIndex,m_trajectoryIndex+1,m_trajectoryCount);
      m_trajectoryIndex++;
      m_timer.Reset();
      m_timer.Start();
    }
    
    /*
     * For a trajectory array, check that we are upto the final trajectory, and that it's total time has elapsed,
     * otherwise just check that the single trajectory total time  has elapsed.
     */
    return (m_trajectories==nullptr)?m_timer.HasElapsed(m_trajectory.TotalTime()):
                                  m_timer.HasElapsed(m_trajectories[m_trajectoryIndex].TotalTime()) &&
                                  (m_trajectoryIndex==m_trajectoryCount-1);
  }

} // namespace frc2
