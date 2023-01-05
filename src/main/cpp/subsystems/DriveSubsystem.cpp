// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#include "subsystems/DriveSubsystem.h"
#include "RobotContainer.h"
#include <frc/geometry/Rotation2d.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>
#include <cmath>
#include "Constants.h"

using namespace DriveConstants;

DriveSubsystem::DriveSubsystem() // WR: Drivesystem Constructor:Inline Initialise
    : m_frontLeft{kFrontLeftDriveMotorID,
                  kFrontLeftTurningMotorID,
                  kFrontLeftAbsoluteTurningEncoderID,
                  kFrontleftAngleOffset,
                  "Front Left "},

      m_rearLeft{
          kRearLeftDriveMotorID,
          kRearLeftTurningMotorID,
          kRearLeftAbsoluteTurningEncoderID,
          kRearleftAngleOffset,
          "Rear Left "},

      m_frontRight{
          kFrontRightDriveMotorID,
          kFrontRightTurningMotorID,
          kFrontRightAbsoluteTurningEncoderID,
          kFrontRightAngleOffset,
          "Front Right "},

      m_rearRight{
          kRearRightDriveMotorID,
          kRearRightTurningMotorID,
          kRearRightAbsoluteTurningEncoderID,
          kRearRightAngleOffset,
          "Rear Right "},
      // m_odometry{kDriveKinematics, frc::Rotation2d(), frc::Pose2d(kStartX,kStartY,frc::Rotation2d(/*kStartRotation*/))}
      m_odometry{
          frc::Rotation2d(),
          frc::Pose2d(kStartX, kStartY, frc::Rotation2d()),
          kDriveKinematics,
          {0.01, 0.01, 0.01},
          {0.1},
          {0.1, 0.1, 0.1},
          0.02_s}
/*SwerveDrivePoseEstimator(
const Rotation2d& gyroAngle, const Pose2d& initialPose,
SwerveDriveKinematics<NumModules>& kinematics,
const wpi::array<double, 3>& stateStdDevs,
const wpi::array<double, 1>& localMeasurementStdDevs,
const wpi::array<double, 3>& visionMeasurementStdDevs,
units::second_t nominalDt = 0.02_s)
*/
{
  // frc::ShuffleboardTab& DefaultTab = frc::Shuffleboard::GetTab("SmartDashboard");

  // setup the PID controller used by locked heading.
  m_thetaController.EnableContinuousInput(units::radian_t(-wpi::numbers::pi),
                                          units::radian_t(wpi::numbers::pi));
// MDE20220203 position monitoring
#ifdef DEBUG_WITH_GLASS
  frc::SmartDashboard::PutData("Field", &m_field);
#endif
  SwervePidState = true;

  // m_pigeon->SetFusedHeading(kStartRotation.value());
  // pigeonOffset=kStartRotation.value();  //odometry is updated with pigeon + pigeonOffset (GetPigeonRotation2D --> calls --> GetHeading [reads pigeon and adds pigeonOffset])
  // m_driveSubsystem->ResetOdometry(frc::Pose2d(kStartX, kStartY, frc::Rotation2d(kStartRotation)));

} // drive subsystem constructor

void DriveSubsystem::Periodic()
{
  m_odometry.Update(GetPigeonRotation2D(), m_frontLeft.GetState(), // swapped front right and rear left to match order of kinematics
                    m_frontRight.GetState(), m_rearLeft.GetState(),
                    m_rearRight.GetState());

  // Put odometry information onto the dashboard
  //frc::SmartDashboard::PutNumber("RobotPoseOdometry/X: ", m_odometry.GetEstimatedPosition().X().value());
  //frc::SmartDashboard::PutNumber("RobotPoseOdometry/Y: ", m_odometry.GetEstimatedPosition().Y().value());
  //frc::SmartDashboard::PutNumber("RobotPoseOdometry/Rotation: ", m_odometry.GetEstimatedPosition().Rotation().Degrees().value());
 #ifdef DEBUG_WITH_GLASS
  m_field.SetRobotPose(m_odometry.GetEstimatedPosition());
#endif
} // Periodic

void DriveSubsystem::Drive(units::meters_per_second_t xSpeed,
                           units::meters_per_second_t ySpeed,
                           units::radians_per_second_t rot,
                           bool fieldRelative)
{
#ifdef DO_SMART_DASHBOARD
  frc::SmartDashboard::PutNumber("Joystick Fwd/Rev in m/s", xSpeed.value());
  frc::SmartDashboard::PutNumber("Joystick Left/right in m/s", ySpeed.value());
  frc::SmartDashboard::PutNumber("Rotation in radians/s", rot.value());
#endif

  /***
   * Turn off Drive and Turning PID's when the joystick is at rest
   */
  /* if ((units::meters_per_second_t((sqrt(pow(xSpeed.value(),2)+pow(ySpeed.value(),2)))) > 0.05*kMaxTeleOpSpeed) ||
       (rot.value()>0?rot.value():-rot.value() > 0.05*kMaxDrivingRotation.value())) {
    */
  /*
if (xSpeed.value()==0 && ySpeed.value()==0 && rot.value()==0)
{
 #ifdef DEBUG
 fmt::print("****TURNING PID OFF rot.value(): {}\n",rot.value());
 #endif
 if (SwervePidState)
 {
   TurningPid(false);
   DrivePid(false);
   SwervePidState = false;
 }
} else {
 #ifdef DEBUG
 fmt::print("****TURNING PID ON rot.value(): {}\n",rot.value());
 #endif
 if (!SwervePidState)
 {
   TurningPid(true);
   DrivePid(true);
   SwervePidState = true;
 }
}*/
  // if (m_activeTrack) m_headingToTrack = units::radian_t(GetAngleToHUB().Degrees());
  double velocity = sqrt(pow(xSpeed.value(), 2) + pow(ySpeed.value(), 2));
  double angle = atan2(ySpeed.value(), xSpeed.value()) * 180 / M_PI;

  if (m_activeTrack)
  {
    m_headingToTrack = (m_limelightTrack ? units::radian_t(m_limelightAngleToHub.Degrees()) : 
                                            units::radian_t(GetAngleToHUBOffset(velocity, angle, &distanceToHubOffset).Degrees()));
#ifdef DO_SMART_DASHBOARD
    frc::SmartDashboard::PutNumber("ROBOT/angle", angle);
    frc::SmartDashboard::PutNumber("ROBOT/velocity", velocity);
    auto forwardVelocity = GetVelocity(); // get actual velocity
    frc::SmartDashboard::PutNumber("ROBOT/huboffset angle", frc::Rotation2d(m_headingToTrack).Degrees().value());
    frc::SmartDashboard::PutNumber("ROBOT/distanceToHubOffset", distanceToHubOffset);
#endif
  }

  auto thetaFF = units::radians_per_second_t(m_thetaController.Calculate(
      units::radian_t(GetHeading()), m_headingToTrack));

  // Overwrite rot if heading lock is on, effectively disconnecting the joystick twist axis
  if (m_headingLock){
    rot = thetaFF;
    //fmt::print("HeadingLock On, Rot value is {}\n",rot.value());
  }
#ifdef DO_SMART_DASHBOARD
  frc::SmartDashboard::PutNumber("Locked Heading (Deg)", m_headingToTrack.value() / wpi::numbers::pi * 180.0);
  frc::SmartDashboard::PutBoolean("Heading Lock", m_headingLock);
  frc::SmartDashboard::PutNumber("Turn Rate", thetaFF.value());
#endif

  //frc::SmartDashboard::PutBoolean("Heading Lock", m_headingLock);
  auto states = kDriveKinematics.ToSwerveModuleStates(
      fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                          (xSpeed * m_driveSpeedMultiplier),
                          (ySpeed * m_driveSpeedMultiplier),
                          (rot * m_driveSpeedMultiplier),
                          GetPigeonRotation2D())                // Add multiplier to these 3 parameters. Default to 1.0, Will change to 0.25 when climbing
                    : frc::ChassisSpeeds{xSpeed, ySpeed, rot}); // Don't GetPigeonRotation2D if you want to not use field relative
  kDriveKinematics.DesaturateWheelSpeeds(&states, DriveConstants::kMaxTeleOpSpeed);

  auto [fl, fr, bl, br] = states;
  m_frontLeft.SetDesiredState(fl);
  m_frontRight.SetDesiredState(fr);
  m_rearLeft.SetDesiredState(bl);
  m_rearRight.SetDesiredState(br);
} // DriveSubsystem::Drive

void DriveSubsystem::DriveStop(){
  Drive(units::meters_per_second_t(0),units::meters_per_second_t(0),units::radians_per_second_t(0),false);
}

void DriveSubsystem::SetModuleStates(wpi::array<frc::SwerveModuleState, 4> desiredStates)
{
  kDriveKinematics.DesaturateWheelSpeeds(&desiredStates,AutoConstants::kMaxSpeed);
  m_frontLeft.SetDesiredState(desiredStates[0]);
  m_frontRight.SetDesiredState(desiredStates[1]);
  m_rearLeft.SetDesiredState(desiredStates[2]);
  m_rearRight.SetDesiredState(desiredStates[3]);
}

void DriveSubsystem::TurningPid(bool setFlag)
{
  if (setFlag)
  {
    m_frontLeft.TurningPidON();
    m_rearLeft.TurningPidON();
    m_frontRight.TurningPidON();
    m_rearRight.TurningPidON();
  }
  else
  {
    m_frontLeft.TurningPidOFF();
    m_rearLeft.TurningPidOFF();
    m_frontRight.TurningPidOFF();
    m_rearRight.TurningPidOFF();
  }
}
void DriveSubsystem::DrivePid(bool setFlag)
{
  if (setFlag)
  {
    m_frontLeft.DrivePidON();
    m_rearLeft.DrivePidON();
    m_frontRight.DrivePidON();
    m_rearRight.DrivePidON();
  }
  else
  {
    m_frontLeft.DrivePidOFF();
    m_rearLeft.DrivePidOFF();
    m_frontRight.DrivePidOFF();
    m_rearRight.DrivePidOFF();
  }
}

void DriveSubsystem::ResetEncoders()
{
  m_frontLeft.ResetEncoders();
  m_rearLeft.ResetEncoders();
  m_frontRight.ResetEncoders();
  m_rearRight.ResetEncoders();
}

void DriveSubsystem::SetActiveTrackMode(bool activeTrackState)
{
  m_activeTrack = activeTrackState;
  //fmt::print("ActiveTrackMode just set to:{}\n",activeTrackState);
}

void DriveSubsystem::SetLockedHeadingMode(bool headingState)
{
  m_headingLock = headingState;
  //fmt::print("LockHeadingMode just set to:{}\n",headingState);
}

bool DriveSubsystem::GetLockedHeadingMode()
{
  //fmt::print("LockHeadingMode just read at:{}\n",m_headingLock);
  return m_headingLock;
}

void DriveSubsystem::SetLockedHeadingAngle(units::radian_t lockedHeadingAngle)
{
  m_headingToTrack = lockedHeadingAngle; // Heading in radians to keep the robot facing while in locked heading mode
  //fmt::print("Locked Heading Angle set to {}\n",lockedHeadingAngle);
}

//WDR:25-4-2022 Don't think this is used, unless shooting on the run
// Maybe can be deleted to help build times
// It returns incorrect information as it should have been the Hypot of forward and sideways
units::velocity::meters_per_second_t DriveSubsystem::GetVelocity()
{
  auto [forward, sideways, angular] = kDriveKinematics.ToChassisSpeeds(
      m_frontLeft.GetState(),
      m_frontRight.GetState(),
      m_rearLeft.GetState(),
      m_rearLeft.GetState());
  // frc::SmartDashboard::PutNumber("BOT SPEED/forward", forward.value());
  // frc::SmartDashboard::PutNumber("BOT SPEED/sideways", sideways.value());
  // frc::SmartDashboard::PutNumber("BOT SPEED/angular", angular.value());
  return forward;
}


units::degree_t DriveSubsystem::GetHeading() const
{
  // return m_gyro.GetRotation2d().Degrees();  //when a difference gyro was used
  // Returns the Heading in degrees. By Going through Rotation2D(  ).Degrees() it will return -180 to 180
  // fmt::print("m_pigeon->GetFusedHeading():{}",m_pigeon->GetFusedHeading());
  //  return frc::Rotation2d(units::degree_t(m_pigeon->GetFusedHeading()+pigeonOffset)).Degrees(); // This is for Pigeon 1
  return frc::Rotation2d(units::degree_t(m_pigeon->GetYaw())).Degrees(); // This is for Pigeon2
}

frc::Rotation2d DriveSubsystem::GetPigeonRotation2D()
{
  // This one was built by WDR to have something that returned the heading as a Rotation2d
  // The constructor for a Rotation2d expects something in units::degrees or units::radian
  // so have to take call GetHeading() so that the heading is in degrees
  // return a Rotation2d object by using the degrees constructor for a rotation2d
  return frc::Rotation2d(GetHeading());
}

void DriveSubsystem::ZeroHeading()
{
  // std::cout << "Just zeroed Heading";
  // m_pigeon->SetFusedHeading(0.0); // This is for Pigeon 1
  int error = m_pigeon->SetYaw(0.0); // This is for Pigeon 2
  if (error != 0)
    fmt::print("*** PIGEON RESET FAILED {} ***\n", m_pigeon->GetYaw());
  else
    fmt::print("*** PIGEON RESET TO {} degrees.\n", 0.0);
  // fmt::print("*** PIGEON RESET TO {} degrees.\n",kStartRotation.value());
  // m_pigeon->SetFusedHeading(kStartRotation.value());
  //  m_gyro.Reset();
}

// Aded by WDR when PathPlannerTrajectory Stuff was added
// Needed a simple method
void DriveSubsystem::SetHeading(double AngleDeg)
{
  m_pigeon->SetYaw(AngleDeg);
}

double DriveSubsystem::GetTurnRate()
{
  // return -m_gyro.GetRate();  //Need to work out how to find the turn rate for a pigeon
  double xyz_dps[3];
  m_pigeon->GetRawGyro(xyz_dps); // returns degrees per second apparently
  return xyz_dps[2];
}

frc::Pose2d DriveSubsystem::GetPose()
{
  return m_odometry.GetEstimatedPosition();
}

void DriveSubsystem::ForcedResetOdometry(frc::Pose2d pose)
{
  m_hasOdometryBeenReset = false;
  ResetOdometry(pose);
}

void DriveSubsystem::ResetOdometry(frc::Pose2d pose)
{
  m_odometry.ResetPosition(pose, GetPigeonRotation2D());
  // if (!m_hasOdometryBeenReset)
  // {
  //   ZeroHeading();
  //   pigeonOffset = pose.Rotation().Degrees().value();
  //   ResetEncoders();
  //   m_odometry.ResetPosition(pose, GetPigeonRotation2D());
  //   m_hasOdometryBeenReset = true;
  //   fmt::print("Odometry Reset Occured: pigeonOffset:{} X:{} Y:{} Rotation:{} GetPigeonRotation2D:{}\n",pigeonOffset,pose.X().value(),pose.Y().value(),pose.Rotation().Degrees().value(),GetPigeonRotation2D().Degrees().value());
  //   frc::Pose2d confirmPose = GetPose();
  //   fmt::print("CONFIRM CURRENT POSE : pigeonOffset:{} X:{} Y:{} Rotation:{}\n",pigeonOffset,confirmPose.X().value(),confirmPose.Y().value(),confirmPose.Rotation().Degrees().value());
  // }
}

void DriveSubsystem::UpdateOdometryVision(frc::Pose2d pose, int trustFactor)
{
  switch (trustFactor)
  {
  case kTrustHIGH: // Track To vision target and stop
    m_odometry.SetVisionMeasurementStdDevs(kTrustHIGHStdDevs);
    odometryUpdateCounter[kTrustHIGH]++;
    break;
  case kTrustMEDIUM: // Lock on target while facing, minimal movement
    m_odometry.SetVisionMeasurementStdDevs(kTrustMEDIUMStdDevs);
    odometryUpdateCounter[kTrustMEDIUM]++;
    break;
  case kTrustLOW: // Got a brief lock on target while driving around
    m_odometry.SetVisionMeasurementStdDevs(kTrustLOWStdDevs);
    odometryUpdateCounter[kTrustLOW]++;
    break;
  }
  m_odometry.AddVisionMeasurement(pose, frc::Timer::GetFPGATimestamp());
}

int *DriveSubsystem::GetOdometryUpdateCounters()
{
  return &odometryUpdateCounter[0];
}

void DriveSubsystem::ResetThetaController(units::radian_t currentHeading)
{
  m_thetaController.Reset(currentHeading);
}

void DriveSubsystem::SetLimelightAngleToHub(double angle)
{
  m_limelightAngleToHub = frc::Rotation2d(units::angle::degree_t(angle));
}

double DriveSubsystem::GetSetLimelightAngleToHub()
{
  return m_limelightAngleToHub.Degrees().value(); 
}

void DriveSubsystem::SetLimelightTrackMode(bool trackFlag)
{
  //fmt::print("LimeLightTrackMode just set to:{}\n",trackFlag);
  m_limelightTrack = trackFlag;

}
bool DriveSubsystem::GetLimelightTrackMode()
{
  //fmt::print("LimeLightTrackMode read at:{}\n",m_limelightTrack);
  return m_limelightTrack;

}

frc::Rotation2d DriveSubsystem::GetAngleToXY(units::length::meter_t fromX, units::length::meter_t fromY,
                                             units::length::meter_t toX, units::length::meter_t toY)
{
  frc::Rotation2d angleToXY;
  if (fromX == toX) // if X is unchanged, then it straight up (90) or straight down (-90).
  {
    if (toY < fromY)
      angleToXY = frc::Rotation2d(units::radian_t(-M_PI / 2.0));
    else
      angleToXY = frc::Rotation2d(units::radian_t(M_PI / 2.0));
  }
  else
  {
    angleToXY = frc::Rotation2d(units::math::atan2((toY - fromY), (toX - fromX)));
    /*if (toX<fromX) //if we are moving left on x-axis, invert launch angle, staying between -180 to 180
      angleToXY = (angleToXY.Degrees().value()<0.0)?
                   angleToXY+frc::Rotation2d(units::degree_t(180.0)):   //add 180 degrees
                   angleToXY-frc::Rotation2d(units::degree_t(180.0));*/
  }

  return angleToXY;
}

double DriveSubsystem::GetDistanceToXY(units::length::meter_t fromX, units::length::meter_t fromY,
                                       units::length::meter_t toX, units::length::meter_t toY)
{
  // d = √[(x2−x1)2 + (y2−y1)2]
  double distanceToXY = sqrt(pow(toX.value() - fromX.value(), 2.0) + pow(toY.value() - fromY.value(), 2.0));
  return distanceToXY;
}

double AirTimeDistance(double distance)
{
  // array time[distanceMM] based on measured values?
  double factor;
  double time = 1.3;
  double deltaTime;
  for (int x = 0; x < 7; x++)
  {
    if (distance > AirTime[x][0] && distance < AirTime[x + 1][0])
    {                                                                            // eg distance 3.7 is between 3.5 and 4.0
      factor = (distance - AirTime[x][0]) / (AirTime[x + 1][0] - AirTime[x][0]); // eg (3.7-3.5)/4.0-3.5=0.2/0.5=0.4
      deltaTime = AirTime[x + 1][1] - AirTime[x][1];                             // eg deltaTime = 1.8-1.2 = 0.6
      time = AirTime[x][1] + factor * deltaTime;                                 // eg time = 1.2+(0.4*0.6) = 1.2 + 0.24 = 1.44
#ifdef DEBUG
      fmt::print("distance: {} air time: {} \n", distance, time);
#endif
      break;
    }
  }
  return time;
}

double DriveSubsystem::GetDistanceToHUBOffset()
{
  return distanceToHubOffset;
}

void DriveSubsystem::SetDistanceToHUBOffset(double distance)
{
  distanceToHubOffset = distance;
}

frc::Rotation2d DriveSubsystem::GetAngleToHUBOffset(double velocity, double angle, double *distanceToHUBOffset)
{
  /*
   * If moving, then Offset X,Y based on current velocity
   */
  constexpr double kAirTimeDistanceFactor = 1.0;
  frc::Pose2d myPose = GetPose();
  double distance = GetDistanceToHUB();
  double airTimeDrivingDistance = kAirTimeDistanceFactor * velocity * AirTimeDistance(distance);
  double xOffset = airTimeDrivingDistance * cos(angle * M_PI / 180);
  double yOffset = airTimeDrivingDistance * sin(angle * M_PI / 180);

  frc::Rotation2d angleToHubOffset = GetAngleToXY(
      myPose.X(), myPose.Y(),
      units::length::meter_t(kHubX - xOffset),
      units::length::meter_t(kHubY - yOffset));

  *distanceToHUBOffset = GetDistanceToXY(units::length::meter_t(kHubX - xOffset),
                                         units::length::meter_t(kHubY - yOffset),
                                         myPose.X(), myPose.Y());
#ifdef DO_SMART_DASHBOARD
  frc::SmartDashboard::PutNumber("ROBOT/distance to hub", distance);
  frc::SmartDashboard::PutNumber("ROBOT/airTimeDrivingDistance", airTimeDrivingDistance);
  frc::SmartDashboard::PutNumber("ROBOT/xOffset", xOffset);
  frc::SmartDashboard::PutNumber("ROBOT/yOffset", yOffset);
  frc::SmartDashboard::PutNumber("ROBOT/angleToHubOffset (before invert)", angleToHubOffset.Degrees().value());
#endif

  return angleToHubOffset.RotateBy(frc::Rotation2d(units::degree_t(180)));
}

frc::Rotation2d DriveSubsystem::GetAngleToHUB()
{
  frc::Pose2d myPose = GetPose();

  frc::Rotation2d angleToHub = GetAngleToXY(
      myPose.X(), myPose.Y(),
      units::length::meter_t(kHubX), units::length::meter_t(kHubY));

  /*angleToHub = (angleToHub.Degrees().value()<0.0)?
                   angleToHub+frc::Rotation2d(units::degree_t(180.0)):   //add 180 degrees
                   angleToHub-frc::Rotation2d(units::degree_t(180.0));*/
  // getting angle from hub to robot, as we want back of robot (shooter) angle to hub.
  return angleToHub.RotateBy(frc::Rotation2d(units::degree_t(180)));
}

double DriveSubsystem::GetDistanceToHUB()
{
  frc::Pose2d myPose = GetPose();

  double distanceToHub = GetDistanceToXY(units::length::meter_t(kHubX), units::length::meter_t(kHubY),
                                         myPose.X(), myPose.Y());
  return distanceToHub;
}

bool DriveSubsystem::IsHubTurnComplete()
{
  frc::Rotation2d myBotAngle = GetPigeonRotation2D();

  if ((myBotAngle.Degrees().value() > frc::Rotation2d(m_headingToTrack).Degrees().value() - kShootAngleTolerance) &&
      (myBotAngle.Degrees().value() < frc::Rotation2d(m_headingToTrack).Degrees().value() + kShootAngleTolerance))
    return true;
  else
    return false;
}

void DriveSubsystem::SetDriveSpeedMultiplier(double driveSpeedMultiplier)
{
  m_driveSpeedMultiplier = driveSpeedMultiplier;
}

/*
void DriveSubsystem::ResetAutoTimer()
{
  m_autoTimer.Reset();
}

void DriveSubsystem::StartAutoTimer()
{
  m_autoTimer.Start();
}

void DriveSubsystem::StopAutoTimer()
{
  m_autoTimer.Start();
}
units::time::second_t DriveSubsystem::GetAutoTimerCountdown()
{
  return (kAutoCountdown - m_autoTimer.Get());
}
*/
//  WDR: This is an attempt at creating our own Sendable implementation that displays
//  the robot heading on the gyro widget.
void DriveSubsystem::InitSendable(wpi::SendableBuilder &builder)
{
  builder.SetSmartDashboardType("Gyro");
  builder.AddDoubleProperty("Value", [this]
                            { return -GetHeading().value(); },
                            {});
}

