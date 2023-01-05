// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "ctre/Phoenix.h" //Needed for Falcons and Pidgeons

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/StateSpaceUtil.h>
#include <frc2/command/SubsystemBase.h>
#include <string.h>
//#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/controller/ProfiledPIDController.h>
//MDE20220203 position monitoring
//#include <frc/smartdashboard/Field2d.h>

#include "Constants.h"
#include "SwerveModule.h"

using namespace DriveConstants;

class DriveSubsystem : public frc2::SubsystemBase
{
public:
    DriveSubsystem();

    /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
    void Periodic() override;

    // Subsystem methods go here.

    /**
   * Drives the robot at given x, y and theta speeds. Speeds range from [-1, 1]
   * and the linear speeds have no effect on the angular speed.
   *
   * @param xSpeed        Speed of the robot in the x direction
   *                      (forward/backwards).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to
   *                      the field.
   */
    void Drive(units::meters_per_second_t xSpeed,
               units::meters_per_second_t ySpeed, units::radians_per_second_t rot,
               bool fieldRelative);
               
    /**
    * Stop the robot
    * Used at the end of trajectories to make sure the robot is told to completely stop
    */
        void DriveStop();

   /**
   * Turning PID Off/On.
   */
    void TurningPid(bool setFlag);
 
   /**
   * Drive PID Off/On.
   */
    void DrivePid(bool setFlag);
  
   /**
   * Resets the drive encoders to currently read a position of 0.
   */
    void ResetEncoders();

    /**
   * Sets the drive SpeedControllers to a power from -1 to 1.
   */
    void SetModuleStates(wpi::array<frc::SwerveModuleState, 4> desiredStates);
    
    units::velocity::meters_per_second_t GetVelocity();

    /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
    units::degree_t GetHeading() const;

    /**
   * Returns the Rotation2D of the robot.
   *
   * @return the robot's heading as a Rotation2D
   */
    frc::Rotation2d GetPigeonRotation2D();

    /**
   * Zeroes the heading of the robot.
   */
    void ZeroHeading();

    /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
    double GetTurnRate();

    /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
    frc::Pose2d GetPose();

    /**
     * Sets the heading of the pigeon 
     * @param AngleDeg  Angle in Degrees as a double
     */ 
    void SetHeading(double AngleDeg);

    /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
    void ResetOdometry(frc::Pose2d pose);
    void ForcedResetOdometry(frc::Pose2d pose);
    void UpdateOdometryVision(frc::Pose2d pose,int trust);
    int * GetOdometryUpdateCounters();

   /**
   * Sets active track heading mode.
   *
   * @param activeTrackState Active Track Mode bool.
   */
   void SetActiveTrackMode(bool activeTrackState);

    /**
   * Sets locked heading mode.
   *
   * @param headingState Heading locked bool.
   */
    void SetLockedHeadingMode(bool headingState);
    /**
 
   * Gets locked heading mode.
   *
   */
    bool GetLockedHeadingMode();

    /**
   * Sets locked heading angle.
   *
   * @param lockedHeadingAngle The angle to make the robot face.
   */
    void SetLockedHeadingAngle(units::radian_t lockedHeadingAngle);

    /**
   * Resets thetaPIDController to current heading in radians
   *
   */
    void ResetThetaController(units::radian_t currentHeading);


   void SetLimelightAngleToHub(double angle);
   double GetSetLimelightAngleToHub();

   void SetLimelightTrackMode(bool trackFlag);
   bool GetLimelightTrackMode();

   frc::Rotation2d GetAngleToXY(units::length::meter_t fromX, units::length::meter_t fromY, 
                                   units::length::meter_t toX, units::length::meter_t toY);
  
   double GetDistanceToXY(units::length::meter_t fromX, units::length::meter_t fromY, 
                                   units::length::meter_t toX, units::length::meter_t toY);

   /*
    * Gets the angle to an appropriate Offset to the HUB if the robot is traveling at a 
    * velocity and heading, such that the ball will land in the HUB. 
    */
   frc::Rotation2d GetAngleToHUBOffset(double velocity, double angle, double *distanceToHUBOffset);
   double GetDistanceToHUBOffset();
   void SetDistanceToHUBOffset(double distance);

   frc::Rotation2d GetAngleToHUB();
   double GetDistanceToHUB();
   
   bool IsHubTurnComplete();

   void SetDriveSpeedMultiplier(double driveSpeedMultiplier); // If this is set to 0.25, this will change the drive speed to quarter speed for the x, y and z axes.

/*
   void ResetAutoTimer();

   void StartAutoTimer();

   void StopAutoTimer();
*/
    //WDR:Trying out custom sendable interfaces to help write better code in the future
   void InitSendable(wpi::SendableBuilder& builder);

  // units::time::second_t GetAutoTimerCountdown();

#ifdef SWERVY
    units::meter_t kTrackWidth =
        .502_m; // Distance between centers of right and left wheels on robot
    units::meter_t kWheelBase =
        .502_m; // Distance between centers of front and back wheels on robot
#else
    units::meter_t kTrackWidth =
        .444_m; // Distance between centers of right and left wheels on robot
    units::meter_t kWheelBase =
        .444_m; // Distance between centers of front and back wheels on robot
#endif

    frc::SwerveDriveKinematics<4> kDriveKinematics{
        frc::Translation2d(kWheelBase / 2, kTrackWidth / 2),    //Front Left
        frc::Translation2d(kWheelBase / 2, -kTrackWidth / 2),   //Front Right
        frc::Translation2d(-kWheelBase / 2, kTrackWidth / 2),   //Back Left
        frc::Translation2d(-kWheelBase / 2, -kTrackWidth / 2)}; //Back Right
    
    // MDE20220203 position monitoring
    //frc::Field2d m_field;
    bool SwervePidState;

private:
    // Components (e.g. motor controllers and sensors) should generally be
    // declared private and exposed only through public methods.

    SwerveModule m_frontLeft;
    SwerveModule m_rearLeft;
    SwerveModule m_frontRight;
    SwerveModule m_rearRight;

    // The gyro sensor
    //PigeonIMU *m_pigeon = new PigeonIMU{DriveConstants::kPigeonID};
    //Pigeon2 m_pigeon{kPigeonID, std::string{kRoboRIOCANBusName}};

    
    Pigeon2 *m_pigeon = new Pigeon2{kPigeonID};
    
    /*
     * MDE20220312 pigeonOffset needs to be initialized (here, set to 0) 
     * before DriveSubsystem m_odemetry.Update() uses GetPigeonRotation2D() in it's Periodic()
     * We subsequently set the pigeonOffset to a meaningful value, once we ge to the point 
     * of doing a ResetOdometry().
     * 0) RobotContainer is instantiated within Robot
     * 1) DriveSubsystem m_drive is instantiated within RobotContainer
     * 2) at that time PoseEstimator m_odometry is constructed with gyro=0, pose: x=kX,kY rot=0
     * 3) m_drive Periodic() fires up immediately doing m_odometry.Update(GetPigeonRotation2D(),<swervestates...>)
     *     -- if pigeonOffset is wrong, ie uninitialized, could this be a problem?
     *     -- could a Periodic() {m_odometry.Update(GetPigeonRotation2D,...)} occur with unset pigeonOffset a
     *        fraction after or during step 4) below?
     *     -- Does a m_odometry.Update(GetPigeonRotation2D,...) correct the m_odometry internally maintained gyro 
     *        offset in a way that a subsequent m_odometry.ResetPosition() doesn't ovverride?
     *     -- Is there a time, before GetAutonomousCommand()'s call to ForceResetodometry(), where
     *          Drive() is called?  
     *          * yes, but it's desgned to not drive unless positive joystick values or SetLockedHeadingMode(true) called; 
     *          * default command DefaultDrive is the DriveSubsystems default command.
     *          * DefaultDrive Execute() repeatedly calls m_drive.Drive() (maybe with joystick disabled in auto?)
     *     -- The very first SwerveControllerCommand provided by GetAutonomousCommand() uses getPose() to know
     *        where it is throughout the trajectory that it is running through.  
     *          * Could there be a remaining DriveSubsystem Periodic() queued up before this ServeControllerCommand
     *            that is still setting m_odometry.Update(getPigeonRotation2D...) using the uninitialized pigeonOffset? 
     *   
     * 4) GetAutonomousCommand() calls ForceResetodometry(), which zeros the Pigeon (IMU) and sets the 
     *    pigeonOffset to actual pose.rotation of actual known start position (pose) of the bot based on 
     *    first Waypoint on selected autonomous-n.txt, and m_odometry.ResetPosition(pose, GetPigeonRotation2D());
     * 
     */

    // WDR:Not Used anymore.    double pigeonOffset=0;  //MDE202203122229 this was not initialized until now (theories above) 
    // frc::ADXRS450_Gyro m_gyro; //Can be Deleted

    // Odometry class for tracking robot pose
    // 4 defines the number of modules
    //frc::SwerveDriveOdometry<4> m_odometry;
    frc::SwerveDrivePoseEstimator<4> m_odometry;

    bool m_headingLock = false;      //true=lock heading, false=joystick controlled
    bool m_activeTrack = false;
    bool m_limelightTrack = false;
    frc::Rotation2d m_limelightAngleToHub;  
    bool m_hasOdometryBeenReset = false;
    units::radian_t m_headingToTrack; //Angle to keep the robot facing when locked in radians
    double distanceToHubOffset;
    int odometryUpdateCounter[11];
    frc::ProfiledPIDController<units::radians> m_thetaController{
        AutoConstants::kPThetaController, 0, 0,
        AutoConstants::kThetaControllerConstraints};

    units::time::second_t m_maxRunTime = units::time::second_t(0);
    long iterationCount;
    double m_driveSpeedMultiplier = 1.0;
    //frc::Timer m_autoTimer;
    //frc::XboxController* m_xboxController;
};

