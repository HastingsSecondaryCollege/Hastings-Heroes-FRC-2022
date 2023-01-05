// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "subsystems/IntakeSubsystem.h"

#include "Constants.h"
#include <frc/DigitalInput.h>
#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix.h>

//#include <frc/smartdashboard/smartdashboard.h>
#include <frc/util/color.h>

#include "rev/ColorSensorV3.h"
#include "rev/ColorMatch.h"

#include <commands/Intake/IntakeEjectAndClose.h>

using namespace StorageConstants;
using namespace OIConstants;
class StorageSubsystem : public frc2::SubsystemBase
{
public:
  StorageSubsystem(IntakeSubsystem* intakeSUB);

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  bool GetFirstBallStorage();
  bool GetSecondBallStorage(); // This may be replaced by GetColorSensorBallClose()
  bool GetFirstBallPastFeeder();
  bool GetTest();
  bool GetColorSensorBallClose();
  std::string GetColorOfBall(); // Use ColorMatch to decide whether ball is blue or red
  bool AreBallColorsDifferent();
  int GetBallCount();
  int GetTargetBallCount();
  bool IsFirstBallOurAllianceColor(); // Returns true if first ball is the same color as our Alliance
  bool GetIgnoreBallColorStatus(); 
  

  void SetFeedMotorPower(double FeedSpeed);

  void SetLoaderMotorPower(double LoaderSpeed);

  void SetLoaderMotorTargetSpeed(double LoaderTargetSpeed);

  void SetLoaderMotorTargetPosition();

  void SetReadyToShoot(bool ReadyToShoot);
  
  void SetIgnoreBallColorStatus(bool ignoreBallColor);

  void ZeroBallCount();
  
  void SetTargetBallCount(int targetBallCount);

  void CheckBallCount();

  void InitialisePreloadedBallColour();

private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  frc::DigitalInput FirstBallStorage{kDIOPortFirstBallStorage};
  frc::DigitalInput SecondBallStorage{kDIOPortSecondBallStorage}; // This may be replaced by colour sensor proximity check
  frc::DigitalInput FirstBallPastFeeder{kDIOPortFirstBallPastFeeder};
  frc::DigitalInput Test{kDIOPortTest}; // This is only for initial testing during build

  IntakeSubsystem* m_intakeSUB;

  IntakeEjectAndClose m_intakeEjectAndCloseCMDGRP{m_intakeSUB, kEjectTimeWhenAutoClosing};

  TalonFX m_feedMotor{kCANIDFeedMotor, std::string{kCANivoreCANBusName}};
  TalonFX m_loaderMotor{kCANIDLoaderMotor, std::string{kCANivoreCANBusName}};
  double m_feedPercentagePower = 0.0;
  double m_loaderPercentagePower = 0.0;
  double m_lastLoaderIncrementPosition = 0.0;
  double m_lastLoaderDecrementPosition = 0.0;
  double m_loaderTargetSpeedUnitsPer100ms = 0.0;
  double m_loaderTargetSpeedRPM = 0.0;
  double m_loaderCurrentSpeedRPM = 0.0;
  double m_loaderTargetPosition = 0.0;
  double m_lastFeederDecrementPosition = 0.0;
  double m_loaderCurrentPosition = 0.0;
  double m_feederCurrentPosition = 0.0;

  bool m_isLoaderInReverse = false; 
  bool m_isFeederInReverse = false;
  bool m_readyToShoot = false;
  bool m_recentlyIncrementedBallCount = false;
  bool m_recentlyDecrementedBallCountAtLoader = false;
  bool m_recentlyDecrementedBallCountAtFeeder = false;
  bool m_needToCheckForSecondBallClose = false;
  bool m_ignoreBallColorSensor = true;

  int m_ballCount = 1; // Default to 1 in comp as we will typically preload one ball. Defaulting to 0 for testing
  //May need radio button in Driver Station to specify if no balls are loaded

  int m_targetBallCount = 0; //The number of balls we should have after shooting
  
  bool m_firstBallStorage = false; // true if sensor can see a ball.
  //The firstBallStorage sensor is at the point where we will store the first ball prior to shooting
  bool m_previousFirstBallStorage = false; // The value of m_firstBallStorage last iteration. Used to detect a change

  bool m_secondBallStorage = false; // true if sensor can see a ball
  // THe secondBallStorage sensor is at the point where we will store the second ball prior to shooting, near the color sensor.
  bool m_previousSecondBallStorage = false; // The value of m_secondBallStorage last iteratino. Used to detect a change.
 
  bool m_firstBallPastFeeder = false; // true if sensor can see a ball.
  // When the firstBallPastFeeder sensor stops seeing a ball, that ball should no longer be in contact with the feeder roller
  bool m_previousFirstBallPastFeeder = false; // The value of m_firstBallPastFeeder last iteration. Used to detect a change
 
  bool m_colorSensorBallClose = false; // true if m_colorSensor.GetProximity() >= kMinProximity.
  // The color sensor will wait until m_colorSensor.GetProximity() >= kMinProximity before checking the colour of a ball
  bool m_previousColorSensorBallClose = false; // The value of m_colorSensorBallClose last iteration. Used to detect a change

  std::string m_ballColorArray[4] = { "Zero", "No ball", "No ball", "No ball"};
  // Array of strings to store color of balls in robot. Skipping first value in array so that ball is array value 1, not 0. 

  /**
   * A Rev Color Sensor V3 object is constructed with an I2C port as a 
   * parameter. The device will be automatically initialized with default 
   * parameters.
   */
  rev::ColorSensorV3 m_colorSensor{kI2CPort};

  /**
     * In addition to RGB IR values, the color sensor can also return an 
     * infrared proximity value. The chip contains an IR led which will emit
     * IR pulses and measure the intensity of the return. When an object is 
     * close the value of the proximity will be large (max 2047 with default
     * settings) and will approach zero when the object is far away.
     * 
     * Proximity can be used to roughly approximate the distance of an object
     * or provide a threshold for when an object is close enough to provide
     * accurate color values.
     */

  uint32_t m_proximity = 0;

    /**
   * A Rev Color Match object is used to register and detect known colors. This can 
   * be calibrated ahead of time or during operation.
   * 
   * This object uses a simple euclidian distance to estimate the closest match
   * with given confidence range.
   */


  rev::ColorMatch m_colorMatcher; 
  frc::Color m_detectedColor;
  double m_redRGB = 0.0; // Red RGB value last detected by colour sensor
  double m_greenRGB = 0.0; // Green RGB value last detected by colour sensor
  double m_blueRGB = 0.0; // Blue RGB value last detected by colour sensor
  double m_minRGB = 0.0; // Minimum RGB value last detected (will match m_redRGB, m_greenRGB or m_blueRGB)
  double m_maxRGB = 0.0; // Maximum RGB value last detected (will match m_redRGB, m_greenRGB or m_blueRGB)
  double m_deltaRGB = 0.0; // Delta RGB, calculated by subtracting m_minRGB from m_maxRGB
  double m_hue = 0.0; // Hue to be calculated from RGB values

    std::string m_colorString;
    double m_colorConfidence = 0.0;
    frc::Color m_matchedColor;

  //You thought it would be a Roborio outlet, but it was me, DIO!

  units::time::second_t m_maxRunTime = units::time::second_t(0);
  long iterationCount;
};
