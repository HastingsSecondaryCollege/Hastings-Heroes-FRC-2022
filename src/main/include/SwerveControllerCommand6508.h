// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <cmath>
#include <functional>
#include <initializer_list>
#include <memory>

//#include <frc/controller/HolonomicDriveController.h>  //WDR Tryin to use own version of controller to debug
#include "HolonomicDriveController6508.h"

#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveModuleState.h>
   
    /* WDR 25-3-22 Having a go at implementing Path Planner Trajectory that includes Holonomic Headings
#include <frc/trajectory/Trajectory.h>
    */
    #include <pathplanner/lib/PathPlanner.h>
    #include <pathplanner/lib/PathPlannerTrajectory.h>

#include "subsystems/DriveSubsystem.h"

#include <units/length.h>
#include <units/time.h>
#include <units/voltage.h>


#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "frc/Timer.h"

#pragma once

namespace frc2 {

/**
 * A command that uses two PID controllers ({@link PIDController}) and a
 * ProfiledPIDController ({@link ProfiledPIDController}) to follow a trajectory
 * {@link Trajectory} with a swerve drive.
 *
 * <p>The command handles trajectory-following, Velocity PID calculations, and
 * feedforwards internally. This is intended to be a more-or-less "complete
 * solution" that can be used by teams without a great deal of controls
 * expertise.
 *
 * <p>Advanced teams seeking more flexibility (for example, those who wish to
 * use the onboard PID functionality of a "smart" motor controller) may use the
 * secondary constructor that omits the PID and feedforward functionality,
 * returning only the raw module states from the position PID controllers.
 *
 * <p>The robot angle controller does not follow the angle given by
 * the trajectory but rather goes to the angle given in the final state of the
 * trajectory.
 */
template <size_t NumModules>
class SwerveControllerCommand
    : public CommandHelper<CommandBase, SwerveControllerCommand<NumModules>> {
 /* -WDR: volts second stuff not used in our implementation
  using voltsecondspermeter =
      units::compound_unit<units::voltage::volt, units::second,
                           units::inverse<units::meter>>;
  using voltsecondssquaredpermeter =
      units::compound_unit<units::voltage::volt, units::squared<units::second>,
                           units::inverse<units::meter>>;
*/

 public:
  /**
   * Constructs a new SwerveControllerCommand that when executed will follow the
   * provided trajectory. This command will not return output voltages but
   * rather raw module states from the position controllers which need to be put
   * into a velocity PID.
   *
   * <p>Note: The controllers will *not* set the outputVolts to zero upon
   * completion of the path- this is left to the user, since it is not
   * appropriate for paths with nonstationary endstates.
   *
   * @param trajectory      The trajectory to follow.
   * @param pose            A function that supplies the robot pose,
   *                        provided by the odometry class.
   * @param kinematics      The kinematics for the robot drivetrain.
   * @param xController     The Trajectory Tracker PID controller
   *                        for the robot's x position.
   * @param yController     The Trajectory Tracker PID controller
   *                        for the robot's y position.
   * @param thetaController The Trajectory Tracker PID controller
   *                        for angle for the robot.
   * @param desiredRotation The angle that the drivetrain should be
   *                        facing. This is sampled at each time step.
   * @param output          The raw output module states from the
   *                        position controllers.
   * @param requirements    The subsystems to require.
   */
  SwerveControllerCommand(
      pathplanner::PathPlannerTrajectory trajectory, std::function<frc::Pose2d()> pose,
      frc::SwerveDriveKinematics<NumModules> kinematics,
      frc2::PIDController xController, frc2::PIDController yController,
      frc::ProfiledPIDController<units::radians> thetaController,
      std::function<frc::Rotation2d()> desiredRotation,
      std::function<void(std::array<frc::SwerveModuleState, NumModules>)>
          output,
      std::initializer_list<Subsystem*> requirements);

  /**
   * Constructs a new SwerveControllerCommand that when executed will follow the
   * provided trajectory. This command will not return output voltages but
   * rather raw module states from the position controllers which need to be put
   * into a velocity PID.
   *
   * <p>Note: The controllers will *not* set the outputVolts to zero upon
   * completion of the path- this is left to the user, since it is not
   * appropriate for paths with nonstationary endstates.
   *
   * <p>Note 2: The final rotation of the robot will be set to the rotation of
   * the final pose in the trajectory. The robot will not follow the rotations
   * from the poses at each timestep. If alternate rotation behavior is desired,
   * the other constructor with a supplier for rotation should be used.
   *
   * @param trajectory      The trajectory to follow.
   * @param pose            A function that supplies the robot pose,
   *                        provided by the odometry class.
   * @param kinematics      The kinematics for the robot drivetrain.
   * @param xController     The Trajectory Tracker PID controller
   *                        for the robot's x position.
   * @param yController     The Trajectory Tracker PID controller
   *                        for the robot's y position.
   * @param thetaController The Trajectory Tracker PID controller
   *                        for angle for the robot.
   * @param output          The raw output module states from the
   *                        position controllers.
   * @param requirements    The subsystems to require.
   */
  SwerveControllerCommand(
      pathplanner::PathPlannerTrajectory trajectory, 
      std::function<frc::Pose2d()> pose,
      frc::SwerveDriveKinematics<NumModules> kinematics,
      frc2::PIDController xController, 
      frc2::PIDController yController,
      frc::ProfiledPIDController<units::radians> thetaController,
      std::function<void(std::array<frc::SwerveModuleState, NumModules>)>
          output,
      std::initializer_list<Subsystem*> requirements);

//------------------WDR Version----------------------
  /**
   * Constructs a new SwerveControllerCommand that when executed will follow the
   * provided trajectory. This command will not return output voltages but
   * rather raw module states from the position controllers which need to be put
   * into a velocity PID.
   *
   * <p>Note: The controllers will *not* set the outputVolts to zero upon
   * completion of the path- this is left to the user, since it is not
   * appropriate for paths with nonstationary endstates.
   *
   * <p>Note 2: The final rotation of the robot will be set to the rotation of
   * the final pose in the trajectory. The robot will not follow the rotations
   * from the poses at each timestep. If alternate rotation behavior is desired,
   * the other constructor with a supplier for rotation should be used.
   *
   * @param trajectory      The trajectory to follow.
   * @param pose            A function that supplies the robot pose,
   *                        provided by the odometry class.
   * @param kinematics      The kinematics for the robot drivetrain.
   * @param xController     The Trajectory Tracker PID controller
   *                        for the robot's x position.
   * @param yController     The Trajectory Tracker PID controller
   *                        for the robot's y position.
   * @param thetaController The Trajectory Tracker PID controller
   *                        for angle for the robot.
   * @param output          The raw output module states from the
   *                        position controllers.
   * @param DriveSubsystem  The subsystems to require.
   */
  SwerveControllerCommand(
      pathplanner::PathPlannerTrajectory trajectory, 
      std::function<frc::Pose2d()> pose,
      frc::SwerveDriveKinematics<NumModules> kinematics,
      frc2::PIDController xController, 
      frc2::PIDController yController,
      frc::ProfiledPIDController<units::radians> thetaController,
      std::function<void(std::array<frc::SwerveModuleState, NumModules>)> output,
      DriveSubsystem* subsystem);



  /**
   * Constructs a new SwerveControllerCommand that when executed will follow the
   * provided trajectory. This command will not return output voltages but
   * rather raw module states from the position controllers which need to be put
   * into a velocity PID.
   *
   * <p>Note: The controllers will *not* set the outputVolts to zero upon
   * completion of the path- this is left to the user, since it is not
   * appropriate for paths with nonstationary endstates.
   *
   *
   * @param trajectory      The trajectory to follow.
   * @param pose            A function that supplies the robot pose,
   *                        provided by the odometry class.
   * @param kinematics      The kinematics for the robot drivetrain.
   * @param xController     The Trajectory Tracker PID controller
   *                        for the robot's x position.
   * @param yController     The Trajectory Tracker PID controller
   *                        for the robot's y position.
   * @param thetaController The Trajectory Tracker PID controller
   *                        for angle for the robot.
   * @param desiredRotation The angle that the drivetrain should be
   *                        facing. This is sampled at each time step.
   * @param output          The raw output module states from the
   *                        position controllers.
   * @param requirements    The subsystems to require.
   */
  SwerveControllerCommand(
      pathplanner::PathPlannerTrajectory trajectory, std::function<frc::Pose2d()> pose,
      frc::SwerveDriveKinematics<NumModules> kinematics,
      frc2::PIDController xController, frc2::PIDController yController,
      frc::ProfiledPIDController<units::radians> thetaController,
      std::function<frc::Rotation2d()> desiredRotation,
      std::function<void(std::array<frc::SwerveModuleState, NumModules>)>
          output,
      //MDE20220117 migrated 2022
      //wpi::ArrayRef<Subsystem*> requirements = {}
      wpi::span<Subsystem*> requirements = {});

  /**
   * Constructs a new SwerveControllerCommand that when executed will follow the
   * provided trajectory. This command will not return output voltages but
   * rather raw module states from the position controllers which need to be put
   * into a velocity PID.
   *
   * <p>Note: The controllers will *not* set the outputVolts to zero upon
   * completion of the path- this is left to the user, since it is not
   * appropriate for paths with nonstationary endstates.
   *
   * <p>Note 2: The final rotation of the robot will be set to the rotation of
   * the final pose in the trajectory. The robot will not follow the rotations
   * from the poses at each timestep. If alternate rotation behavior is desired,
   * the other constructor with a supplier for rotation should be used.
   *
   * @param trajectory      The trajectory to follow.
   * @param pose            A function that supplies the robot pose,
   *                        provided by the odometry class.
   * @param kinematics      The kinematics for the robot drivetrain.
   * @param xController     The Trajectory Tracker PID controller
   *                        for the robot's x position.
   * @param yController     The Trajectory Tracker PID controller
   *                        for the robot's y position.
   * @param thetaController The Trajectory Tracker PID controller
   *                        for angle for the robot.
   * @param output          The raw output module states from the
   *                        position controllers.
   * @param requirements    The subsystems to require.
   */
  SwerveControllerCommand(
      pathplanner::PathPlannerTrajectory trajectory, std::function<frc::Pose2d()> pose,
      frc::SwerveDriveKinematics<NumModules> kinematics,
      frc2::PIDController xController, frc2::PIDController yController,
      frc::ProfiledPIDController<units::radians> thetaController,
      std::function<void(std::array<frc::SwerveModuleState, NumModules>)>
          output,
    //MDE20220117 migrated 2022
    //wpi::ArrayRef<Subsystem*> requirements = {}
      wpi::span<Subsystem*> requirements = {});

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  pathplanner::PathPlannerTrajectory m_trajectory;
  std::function<frc::Pose2d()> m_pose;
  frc::SwerveDriveKinematics<NumModules> m_kinematics;
  frc::HolonomicDriveController m_controller;
  std::function<frc::Rotation2d()> m_desiredRotation;
  std::function<void(std::array<frc::SwerveModuleState, NumModules>)> m_outputStates;
  DriveSubsystem* m_drive;
  frc::Timer m_timer;
  units::second_t m_prevTime;
  frc::Rotation2d m_finalRotation;
};
}  // namespace frc2

#include "SwerveControllerCommand6508.inc"