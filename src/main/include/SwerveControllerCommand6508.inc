// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <memory>
#include <utility>

#include "SwerveControllerCommand6508.h"

namespace frc2
{

    template <size_t NumModules>
    SwerveControllerCommand<NumModules>::SwerveControllerCommand(
        pathplanner::PathPlannerTrajectory trajectory,
        std::function<frc::Pose2d()> pose,
        frc::SwerveDriveKinematics<NumModules> kinematics,
        frc2::PIDController xController, frc2::PIDController yController,
        frc::ProfiledPIDController<units::radians> thetaController,
        std::function<frc::Rotation2d()> desiredRotation,
        std::function<void(std::array<frc::SwerveModuleState, NumModules>)> output,
        std::initializer_list<Subsystem *> requirements)
        : m_trajectory(std::move(trajectory)),
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
        pathplanner::PathPlannerTrajectory trajectory,
        std::function<frc::Pose2d()> pose,
        frc::SwerveDriveKinematics<NumModules> kinematics,
        frc2::PIDController xController,
        frc2::PIDController yController,
        frc::ProfiledPIDController<units::radians> thetaController,
        std::function<void(std::array<frc::SwerveModuleState, NumModules>)> output,
        std::initializer_list<Subsystem *> requirements)
        : m_trajectory(std::move(trajectory)),
          m_pose(std::move(pose)),
          m_kinematics(kinematics),
          m_controller(xController, yController, thetaController),
          m_outputStates(output)
    {
        this->AddRequirements(requirements);
        // m_desiredRotation = [&] {
        //   return m_trajectory.States().back().pose.Rotation();
        // };
    }

    //---------------------WDR Version with drivesubsystem--------------------------
    template <size_t NumModules>
    SwerveControllerCommand<NumModules>::SwerveControllerCommand(
        pathplanner::PathPlannerTrajectory trajectory,
        std::function<frc::Pose2d()> pose,
        frc::SwerveDriveKinematics<NumModules> kinematics,
        frc2::PIDController xController,
        frc2::PIDController yController,
        frc::ProfiledPIDController<units::radians> thetaController,
        std::function<void(std::array<frc::SwerveModuleState, NumModules>)> output,
        DriveSubsystem *subsystem)
        : m_trajectory(std::move(trajectory)),
          m_pose(std::move(pose)),
          m_kinematics(kinematics),
          m_controller(xController, yController, thetaController),
          m_outputStates(output),
          m_drive(subsystem)
    {
        this->AddRequirements(subsystem);
    }

    template <size_t NumModules>
    SwerveControllerCommand<NumModules>::SwerveControllerCommand(
        pathplanner::PathPlannerTrajectory trajectory,
        std::function<frc::Pose2d()> pose,
        frc::SwerveDriveKinematics<NumModules> kinematics,
        frc2::PIDController xController, frc2::PIDController yController,
        frc::ProfiledPIDController<units::radians> thetaController,
        std::function<frc::Rotation2d()> desiredRotation,
        std::function<void(std::array<frc::SwerveModuleState, NumModules>)> output,
        // MDE20220117 migrated 2022
        // wpi::ArrayRef<Subsystem *> requirements
        wpi::span<Subsystem *> requirements)
        : m_trajectory(std::move(trajectory)),
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
        pathplanner::PathPlannerTrajectory trajectory,
        std::function<frc::Pose2d()> pose,
        frc::SwerveDriveKinematics<NumModules> kinematics,
        frc2::PIDController xController, frc2::PIDController yController,
        frc::ProfiledPIDController<units::radians> thetaController,
        std::function<void(std::array<frc::SwerveModuleState, NumModules>)> output,
        // MDE20220117 migrated 2022
        // wpi::ArrayRef<Subsystem *> requirements
        wpi::span<Subsystem *> requirements)
        : m_trajectory(std::move(trajectory)),
          m_pose(std::move(pose)),
          m_kinematics(kinematics),
          m_controller(xController, yController, thetaController),
          m_outputStates(output)
    {
        this->AddRequirements(requirements);
        // m_desiredRotation = [&] {
        //   return m_trajectory.States().back().pose.Rotation();
        // };
    }

    template <size_t NumModules>
    void SwerveControllerCommand<NumModules>::Initialize()
    {           //Desired Rotation is no longer being used, because PP trajectory have holonomic rotations in them
        if (m_desiredRotation == nullptr)
        {
            m_desiredRotation = [&]
            {
                // return m_trajectory.States().back().pose.Rotation();    //Doesn't work as syntax is different between Path Planner and WPILib
                return frc::Rotation2d(0_deg);
            };
        }
        else
        {
        }
        m_timer.Reset();
        m_timer.Start();
    }

    template <size_t NumModules>
    void SwerveControllerCommand<NumModules>::Execute()
    {
        auto curTime = units::second_t(m_timer.Get());
        // WDR 25-3-22 had to change .Sample t0 .sample
        auto m_desiredState = m_trajectory.sample(curTime);

        // std::cout << m_desiredState.pose.X() << "," << m_desiredState.pose.Y() << "," << m_desiredState.pose.Rotation().Degrees().value() << "," << m_pose().Rotation().Degrees().value() << ",";
        auto targetChassisSpeeds =
            // WDR 25-3-22 Trying to implement PathPlaner Trajectories
            // m_controller.Calculate(m_pose(), m_desiredState, m_desiredRotation());
            // Need to change parameters to .Calculate method of holomnic Controller
            m_controller.Calculate(m_pose(), m_desiredState.pose, m_desiredState.velocity, m_desiredState.holonomicRotation);
        auto targetModuleStates =
            m_kinematics.ToSwerveModuleStates(targetChassisSpeeds);

        m_outputStates(targetModuleStates);
    }

    template <size_t NumModules>
    void SwerveControllerCommand<NumModules>::End(bool interrupted)
    {
        m_timer.Stop();
        m_controller.m_firstRun=true;
        fmt::print("Just finish Traj. Interrupted={}\n",interrupted);
    }

    template <size_t NumModules>
    bool SwerveControllerCommand<NumModules>::IsFinished()
    {
        //fmt::print("{} of {}\n",m_timer.Get(),m_trajectory.getTotalTime());
        return m_timer.HasElapsed(m_trajectory.getTotalTime());
    }

} // namespace frc2
