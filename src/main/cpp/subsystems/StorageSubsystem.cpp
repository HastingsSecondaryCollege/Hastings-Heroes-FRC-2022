// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/StorageSubsystem.h"
#include "subsystems/ShooterSubsystem.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DriverStation.h>
#include <frc/Timer.h>
StorageSubsystem::StorageSubsystem(IntakeSubsystem *intakeSUB) : m_intakeSUB(intakeSUB)
{
  m_feedMotor.ConfigFactoryDefault();
  m_feedMotor.ConfigNeutralDeadband(kNeutralDeadband);
  m_feedMotor.ConfigVoltageCompSaturation(12.0);
  m_feedMotor.SetNeutralMode(Brake);
  m_feedMotor.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrame::Status_1_General_, 255, 0);
  m_feedMotor.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrame::Status_2_Feedback0_, 255, 0);
  m_feedMotor.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrame::Status_4_AinTempVbat_, 255, 0);
  m_feedMotor.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrame::Status_6_Misc_, 255, 0);
  m_feedMotor.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrame::Status_7_CommStatus_, 255, 0);
  m_feedMotor.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrame::Status_9_MotProfBuffer_, 255, 0);
  m_feedMotor.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrame::Status_10_MotionMagic_, 255, 0);
  m_feedMotor.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrame::Status_10_Targets_, 255, 0);
  m_feedMotor.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrame::Status_12_Feedback1_, 255, 0);
  m_feedMotor.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrame::Status_13_Base_PIDF0_, 255, 0);
  m_feedMotor.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrame::Status_14_Turn_PIDF1_, 255, 0);
  m_feedMotor.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrame::Status_15_FirmareApiStatus_, 255, 0);
  m_feedMotor.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrame::Status_17_Targets1_, 255, 0);
  m_loaderMotor.ConfigFactoryDefault();
  m_loaderMotor.ConfigNeutralDeadband(kNeutralDeadband);
  m_loaderMotor.ConfigVoltageCompSaturation(12.0);
  m_loaderMotor.SetNeutralMode(Brake);
  m_loaderMotor.ConfigSelectedFeedbackSensor(TalonFXFeedbackDevice::IntegratedSensor, kPIDLoopIdx, kTimeoutMs);
  m_loaderMotor.SetSelectedSensorPosition(0, kPIDLoopIdx, kTimeoutMs); // Reset Loader encoder position to 0 at robot boot
  m_loaderMotor.Config_kF(kPIDLoopIdx, kFLoaderMotor, kTimeoutMs);
  m_loaderMotor.Config_kP(kPIDLoopIdx, kPLoaderMotor, kTimeoutMs);
  m_loaderMotor.Config_kI(kPIDLoopIdx, kILoaderMotor, kTimeoutMs);
  m_loaderMotor.Config_kD(kPIDLoopIdx, kDLoaderMotor, kTimeoutMs);
  m_loaderMotor.Config_kF(kSecondaryPIDLoopIdx, kFLoaderMotorPosition, kTimeoutMs);
  m_loaderMotor.Config_kP(kSecondaryPIDLoopIdx, kPLoaderMotorPosition, kTimeoutMs);
  m_loaderMotor.Config_kI(kSecondaryPIDLoopIdx, kILoaderMotorPosition, kTimeoutMs);
  m_loaderMotor.Config_kD(kSecondaryPIDLoopIdx, kDLoaderMotorPosition, kTimeoutMs);
  m_colorMatcher.AddColorMatch(kBlueTarget);
  m_colorMatcher.AddColorMatch(kRedTarget);
}

// This method will be called once per scheduler run
void StorageSubsystem::Periodic()
{
  /*if (!((iterationCount++%5)==0)) //overruns: only check sensors every 0.1/seconds
    return;*/

#ifdef DO_PROCESSING_TIMERS
  units::time::second_t startTime = frc::Timer::GetFPGATimestamp();
#endif
  m_previousColorSensorBallClose = m_colorSensorBallClose; // Store the value of m_colorSensorBallClose from the last iteration. Used to detect a change
  m_colorSensorBallClose = GetColorSensorBallClose();      // Check whether there is currently a ball near the color sensor
  m_previousFirstBallStorage = m_firstBallStorage;         // Store the value of m_firstBallStorage from the last iteration. Used to detect a change
  m_firstBallStorage = GetFirstBallStorage();              // Check whether the m_firstBallStorage sensor can currently see a ball
  m_previousFirstBallPastFeeder = m_firstBallPastFeeder;   // Store the value of m_firstBallPastFeeder from the last iteration. Used to detect a change
  m_firstBallPastFeeder = GetFirstBallPastFeeder();        // Check whether the m_firstBallPastFeeder sensor can currently see a ball
  m_previousSecondBallStorage = m_secondBallStorage;
  m_secondBallStorage = GetSecondBallStorage();
  m_loaderCurrentPosition = m_loaderMotor.GetSelectedSensorPosition();
  m_feederCurrentPosition = m_feedMotor.GetSelectedSensorPosition();

  if ((m_loaderCurrentPosition > (m_lastLoaderIncrementPosition + kLoaderMotorBallDistance)) || (m_loaderCurrentPosition < (m_lastLoaderIncrementPosition - kLoaderMotorBallDistance)))
  {
    m_recentlyIncrementedBallCount = false;
  }

  if ((m_loaderCurrentPosition > (m_lastLoaderDecrementPosition + kLoaderMotorBallDistance)) || (m_loaderCurrentPosition < (m_lastLoaderDecrementPosition - kLoaderMotorBallDistance)))
  {
    m_recentlyDecrementedBallCountAtLoader = false;
  }

  if ((m_feederCurrentPosition > (m_lastFeederDecrementPosition + kFeederMotorBallDistance)) || (m_feederCurrentPosition < (m_lastFeederDecrementPosition + kFeederMotorBallDistance)))
  {
    m_recentlyDecrementedBallCountAtFeeder = false;
  }

  if (m_firstBallPastFeeder != m_previousFirstBallPastFeeder)
  { // Has there been a change in the last sensor detecting a ball?
    if (!m_firstBallPastFeeder)
    { // If the last sensor has changed to no longer detect a ball, then we need to deindex the ball
      if (!m_isFeederInReverse)
      { // ensure feeder is not in reverse
        if (!m_recentlyDecrementedBallCountAtFeeder)
        {                                                            // ensure that we haven't recently decremented ball count at feeder
          m_ballCount -= 1;                                          // Subtract 1 from current ball count
          frc::SmartDashboard::PutNumber("Ball Count", m_ballCount); // How many balls are currently in the robot
          m_lastFeederDecrementPosition = m_feederCurrentPosition;
          m_recentlyDecrementedBallCountAtLoader = true;
          if (m_ballCount <= 0)
          {                  // Check if m_ballCount goes lower than 0
            m_ballCount = 0; // If it does, reset m_ballCount to 0
          }
          m_ballColorArray[1] = m_ballColorArray[2]; // Move the recorded ball colours from position 2 to 1, then 3 to 2
          m_ballColorArray[2] = m_ballColorArray[3]; //
          m_ballColorArray[3] = "No ball";           // Clear the colour of ball 3 as we no longer have three balls
          frc::SmartDashboard::PutString("First Ball Color", m_ballColorArray[1]);
          frc::SmartDashboard::PutString("Second Ball Color", m_ballColorArray[2]);
          // frc::SmartDashboard::PutString("Third Ball Color", m_ballColorArray[3]);
        }
      }
    }
  }

  if (m_secondBallStorage != m_previousSecondBallStorage)
  { // Has there been a change in the color sensor detecting a ball?
    if (m_secondBallStorage)
    { // Is there currently a ball close to the color sensor?
      // if (m_loaderPercentagePower == kLoaderMotorReversePower) // Moving away from percentage power to PID control
      if (!m_isLoaderInReverse)
      {                                      // Check that loader is not in reverse. This would indicate that it is ejecting a ball out the intake
        if (!m_recentlyIncrementedBallCount) //
        {                                    // Avoid double counting ball by checking that LoaderMotor has moved a sufficient distance since the last time we counted a ball
          if (m_ballCount < 1)
          {
            SetFeedMotorPower(kFeederMotorPower);
            m_needToCheckForSecondBallClose = true;
          }
          m_ballCount += 1;                                          // Add 1 to current ball count if loader is not in reverse, and has moved far enough since the last ball was counted
          frc::SmartDashboard::PutNumber("Ball Count", m_ballCount); // How many balls are currently in the robot
          m_lastLoaderIncrementPosition = m_loaderCurrentPosition;   // Store encoder position of Loader Motor to avoid double counting this ball
          // frc::SmartDashboard::PutNumber("Last Loader Increment Position", m_lastLoaderIncrementPosition);
          m_recentlyIncrementedBallCount = true; // Set this bool so that we don't count the same ball twice
          if (m_ballCount >= kMaxBallCount)
          {                                            // Restrict m_ballCount to the max balls that will fit between our first and last sensor, probably 3
            m_ballCount = kMaxBallCount;               // Set m_ballCount to kMaxBallCount
            m_ballColorArray[1] = m_ballColorArray[2]; // It appears that we failed to notice a ball leaving the shooter
            m_ballColorArray[2] = m_ballColorArray[3]; // Move the recorded ball colours from position 2 to 1, then 3 to 2
          }
          m_ballColorArray[m_ballCount] = GetColorOfBall(); // Get the colour of the ball near the colour sensor and store it in an array
          frc::SmartDashboard::PutString("First Ball Color", m_ballColorArray[1]);
          frc::SmartDashboard::PutString("Second Ball Color", m_ballColorArray[2]);
          // frc::SmartDashboard::PutString("Third Ball Color", m_ballColorArray[3]);
          if (m_ballCount > 1)
          { // Check whether we have more than one ball?

            // m_intakeSUB -> CloseIntake();
            // m_intakeSUB -> IntakeRollersOut();
            m_intakeEjectAndCloseCMDGRP.Schedule(); // Pulling intake and reversing intake motors after a specified amount of time intake motors stop

            // frc::SmartDashboard::PutBoolean("Mismatched ball colours", AreBallColorsDifferent());

            if (!m_readyToShoot || AreBallColorsDifferent())
            { // Check whether  m_readyToShoot is false or whether the ball at the color sensor is a different color to the ball further ahead in the shooter
              SetLoaderMotorTargetPosition();
              m_ballColorArray[m_ballCount] = GetColorOfBall(); // Get the colour of the ball near the colour sensor and store it in an array. This is the second get of the same colour to increase accuracy
              frc::SmartDashboard::PutString("First Ball Color", m_ballColorArray[1]);
              frc::SmartDashboard::PutString("Second Ball Color", m_ballColorArray[2]);
              // frc::SmartDashboard::PutString("Third Ball Color", m_ballColorArray[3]);
              //  SetLoaderMotorTargetSpeed(0); // If either of the above things are true, stop the loader motor
              //  m_loaderPercentagePower = 0; // Old method was to set percentage power to 0. We now plan to use PID control to keep LoaderMotor stopped
            }
          }
        }
      }
    }
    else if (m_isLoaderInReverse) // Check if loader is in reverse. This would indicate that is is ejecting a ball out the intake
    // else if (m_loaderPercentagePower == kLoaderMotorReversePower) // Moving away from percentage power to PID control
    {
      if (!m_recentlyDecrementedBallCountAtLoader)
      {
        m_lastLoaderDecrementPosition = m_loaderCurrentPosition;
        m_ballColorArray[m_ballCount] = "No ball"; // Clear colour of ball as it is being ejected out the intake
        if (m_ballCount > 0)
        {
          m_ballColorArray[m_ballCount] = "No ball";                 // Clear color of ball being ejected
          m_ballCount -= 1;                                          // Reduce ball count by 1 as ball is being ejected out intake
          frc::SmartDashboard::PutNumber("Ball Count", m_ballCount); // How many balls are currently in the robot
          frc::SmartDashboard::PutString("First Ball Color", m_ballColorArray[1]);
          frc::SmartDashboard::PutString("Second Ball Color", m_ballColorArray[2]);
          // frc::SmartDashboard::PutString("Third Ball Color", m_ballColorArray[3]);
        }
        m_recentlyDecrementedBallCountAtLoader = true;
      }
    }
  }
  /* else
  {
    if (m_secondBallStorage)
    {
      if (!m_isLoaderInReverse)
      {
        if (m_needToCheckForSecondBallClose)
        {
          if (!m_recentlyIncrementedBallCount)
          {
            m_ballCount += 1;                                          // Add 1 to current ball count if loader is not in reverse, and has moved far enough since the last ball was counted
            frc::SmartDashboard::PutNumber("Ball Count", m_ballCount); // How many balls are currently in the robot
            m_lastLoaderIncrementPosition = m_loaderCurrentPosition;   // Store encoder position of Loader Motor to avoid double counting this ball
            frc::SmartDashboard::PutNumber("Last Loader Increment Position", m_lastLoaderIncrementPosition);
            m_recentlyIncrementedBallCount = true; // Set this bool so that we don't count the same ball twice
            if (m_ballCount >= kMaxBallCount)
            {                                            // Restrict m_ballCount to the max balls that will fit between our first and last sensor, probably 3
              m_ballCount = kMaxBallCount;               // Set m_ballCount to kMaxBallCount
              m_ballColorArray[1] = m_ballColorArray[2]; // It appears that we failed to notice a ball leaving the shooter
              m_ballColorArray[2] = m_ballColorArray[3]; // Move the recorded ball colours from position 2 to 1, then 3 to 2
            }
            m_ballColorArray[m_ballCount] = GetColorOfBall(); // Get the colour of the ball near the colour sensor and store it in an array
            frc::SmartDashboard::PutString("First Ball Color", m_ballColorArray[1]);
            frc::SmartDashboard::PutString("Second Ball Color", m_ballColorArray[2]);
            frc::SmartDashboard::PutString("Third Ball Color", m_ballColorArray[3]);
            if (m_ballCount > 1)
            { // Check whether we have more than one ball?

              // m_intakeSUB -> CloseIntake();
              // m_intakeSUB -> IntakeRollersOut();
              m_intakeEjectAndCloseCMDGRP.Schedule(); // Pulling intake and reversing intake motors after a specified amount of time intake motors stop

              frc::SmartDashboard::PutBoolean("Mismatched ball colours", AreBallColorsDifferent());

              if (!m_readyToShoot || AreBallColorsDifferent())
              { // Check whether  m_readyToShoot is false or whether the ball at the color sensor is a different color to the ball further ahead in the shooter
                SetLoaderMotorTargetPosition();
                m_ballColorArray[m_ballCount] = GetColorOfBall(); // Get the colour of the ball near the colour sensor and store it in an array. This is the second get of the same colour to increase accuracy
                frc::SmartDashboard::PutString("First Ball Color", m_ballColorArray[1]);
                frc::SmartDashboard::PutString("Second Ball Color", m_ballColorArray[2]);
                frc::SmartDashboard::PutString("Third Ball Color", m_ballColorArray[3]);
                // SetLoaderMotorTargetSpeed(0); // If either of the above things are true, stop the loader motor
                // m_loaderPercentagePower = 0; // Old method was to set percentage power to 0. We now plan to use PID control to keep LoaderMotor stopped
              }
            }
          }
        }
      }
    }
  } */
  // Commenting out while testing new proximity sensor. The above code is to count two balls close together

  if (m_firstBallStorage != m_previousFirstBallStorage)
  { // Has there been a change in the second sensor detecting a ball?
    if (m_firstBallStorage)
    { // Does the second sensor currently detect a ball.
      if (!m_readyToShoot)
      {
        SetFeedMotorPower(0);
      }
      if (m_secondBallStorage)
      { // Can the colour sensor see a ball?
        if (m_ballCount == 1)
        { // Is our ball count 1?
          m_ballCount = 2;
          frc::SmartDashboard::PutNumber("Ball Count", m_ballCount); // How many balls are currently in the robot
          m_ballColorArray[m_ballCount] = GetColorOfBall();          // Get the colour of the ball near the colour sensor and store it in an array
          frc::SmartDashboard::PutString("First Ball Color", m_ballColorArray[1]);
          frc::SmartDashboard::PutString("Second Ball Color", m_ballColorArray[2]);
          // frc::SmartDashboard::PutString("Third Ball Color", m_ballColorArray[3]);
          if (!m_readyToShoot || AreBallColorsDifferent())
          {                               // Check whether  m_readyToShoot is false or whether the ball at the color sensor is a different color to the ball furt
            SetLoaderMotorTargetSpeed(0); // If either of the above things are true, stop the loader motor
          }
        }
      }

      if (!m_readyToShoot)
      {
        SetFeedMotorPower(0); // Set Feeder power to zero if we are not ready to shoot
      }
    }
  }

  /* if (!m_readyToShoot && GetFirstBallStorage())
  {
    m_feedPercentagePower = 0;
  } */
  // Let balls flow throw constantly for testing on 9/2/22

  m_feedMotor.Set(ControlMode::PercentOutput, m_feedPercentagePower);
  // m_loaderMotor.Set(ControlMode::PercentOutput, m_loaderPercentagePower); // Old method was to set percentage power to 0. We now plan to use PID control to keep LoaderMotor stopped

  m_proximity = m_colorSensor.GetProximity();
  // frc::SmartDashboard::PutNumber("Proximity", m_proximity);
  frc::SmartDashboard::PutNumber("Ball Count", m_ballCount); // How many balls are currently in the robot
/* // Block commenting out SmartDashboard lines

frc::SmartDashboard::PutString("Second Ball Color", m_ballColorArray[2]);
frc::SmartDashboard::PutString("Third Ball Color", m_ballColorArray[3]);
frc::SmartDashboard::PutBoolean("Color Sensor Ball Close", GetColorSensorBallClose());
frc::SmartDashboard::PutBoolean("First Ball Storage", GetFirstBallStorage());
// frc::SmartDashboard::PutBoolean("Second Ball Storage", GetSecondBallStorage()); // Replaced by color sensor
frc::SmartDashboard::PutBoolean("First Ball Past Feeder", GetFirstBallPastFeeder());
// frc::SmartDashboard::PutBoolean("SensorTest", GetTest()); // True means sensor can see a ball
frc::SmartDashboard::PutNumber("Loader Current Position", m_loaderCurrentPosition);
//  frc::SmartDashboard::PutData("Loader Current Control Mode", m_loaderMotor.GetControlMode());
frc::SmartDashboard::PutNumber("Loader Target Position", m_loaderTargetPosition);

*/
// Block commenting out SmartDashboard lines

/*if (GetColorSensorBallClose()) {
     m_ballColorArray[1] = GetColorOfBall(); //temporary code to test colour sensor
  } */
#ifdef DO_PROCESSING_TIMERS
  units::time::second_t periodicRunTime = (frc::Timer::GetFPGATimestamp() - startTime);
  if (periodicRunTime > kRunningTooLong)
  {
    fmt::print("\n\nStorage run time: {}\n\n\n", periodicRunTime);
  }

  if (periodicRunTime > m_maxRunTime)
  {
    m_maxRunTime = periodicRunTime;
    // frc::SmartDashboard::PutNumber("Storage Periodic Max run time", m_maxRunTime.value());
  }
#endif
}

bool StorageSubsystem::GetFirstBallStorage() // True means sensor can see a ball
{
  return !FirstBallStorage.Get();
}

bool StorageSubsystem::GetSecondBallStorage() // True means sensor can see a ball
{
  return !SecondBallStorage.Get();
}

bool StorageSubsystem::GetFirstBallPastFeeder() // True means sensor can see a ball
{
  return !FirstBallPastFeeder.Get();
}

bool StorageSubsystem::GetTest() // True means sensor can see a ball
{
  return !Test.Get();
}

bool StorageSubsystem::GetColorSensorBallClose() // True means ball is close to color sensor
{
  return (m_colorSensor.GetProximity() >= kMinProximity);
}

bool StorageSubsystem::AreBallColorsDifferent() // True if the ball at the colour sensor is a different colour to the ball further in the shooter
{
  if (m_ignoreBallColorSensor)
  {
    return false;
  }
  else
  {
    return (((m_ballColorArray[m_ballCount] == "Red") && (m_ballColorArray[m_ballCount - 1] == "Blue")) || ((m_ballColorArray[m_ballCount] == "Blue") && (m_ballColorArray[m_ballCount - 1] == "Red")));
  }
}

bool StorageSubsystem::IsFirstBallOurAllianceColor()
{
  if (m_ignoreBallColorSensor)
  {
    return true;
  }
  else
  {
    return ((m_ballColorArray[1] == "Blue" && frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue) ||
            (m_ballColorArray[1] == "Red" && frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed));
  }
}

int StorageSubsystem::GetBallCount() // number of balls currently in robot
{
  return m_ballCount;
}

int StorageSubsystem::GetTargetBallCount() // The number of balls we should have after shooting
{
  return m_targetBallCount;
}

std::string StorageSubsystem::GetColorOfBall() // "red" or "blue"
{
  /**
   * The method GetColor() returns a normalized color value from the sensor and can be
   * useful if outputting the color to an RGB LED or similar. To
   * read the raw color, use GetRawColor().
   *
   * The color sensor works best when within a few inches from an object in
   * well lit conditions (the built in LED is a big help here!). The farther
   * an object is the more light from the surroundings will bleed into the
   * measurements and make it difficult to accurately determine its color.
   */
  m_detectedColor = m_colorSensor.GetColor(); // Store RGB values of detected color in m_detectedColor
  /**
   * Run the color match algorithm on our detected color
   */
  m_redRGB = m_detectedColor.red;                         // Store the detected RGB Red value in a variable
  m_greenRGB = m_detectedColor.green;                     // Store the detected RGB Red value in a variable
  m_blueRGB = m_detectedColor.blue;                       // Store the detected RGB Red value in a variable
  m_minRGB = std::min({m_redRGB, m_greenRGB, m_blueRGB}); // Set m_minRGB to be the lowest value out of m_redRGB, m_greenRGB and m_blueRGB
  m_maxRGB = std::max({m_redRGB, m_greenRGB, m_blueRGB}); // Set m_maxRGB to be the highest value out of m_redRGB, m_greenRGB and m_blueRGB
  m_deltaRGB = m_maxRGB - m_minRGB;                       // m_deltaRGB is the difference between m_maxRGB nad m_minRGB
  if (m_maxRGB == m_redRGB)
  { // if the highest RGB value is red, use the following formula for hue
    m_hue = (fmod(((m_greenRGB - m_blueRGB) / m_deltaRGB), 6.0) * 60);
  }
  else if (m_maxRGB == m_greenRGB)
  { // if the highest RGB value is green, use the following formula for hue
    m_hue = ((((m_blueRGB - m_redRGB) / m_deltaRGB) + 2.0) * 60);
  }
  else if (m_maxRGB == m_blueRGB)
  { // if the highest RGB value is bluw, use the following formula for hue
    m_hue = ((((m_redRGB - m_greenRGB) / m_deltaRGB) + 4.0) * 60);
  }

  if (m_hue < kHueThreshold)
  {
    m_colorString = "Red";
  }
  else
  {
    m_colorString = "Blue";
  }

  /* // Replaced the color match with a hue check 6/4/22
  m_matchedColor = m_colorMatcher.MatchClosestColor(m_detectedColor, m_colorConfidence);

  if (m_matchedColor == kBlueTarget)
  {
    m_colorString = "Blue";
  }
  else if (m_matchedColor == kRedTarget)
  {
    m_colorString = "Red";
  }
  else
  {
    m_colorString = "Unknown";

  }
*/
  frc::SmartDashboard::PutNumber("Last Detected RGB Red", m_detectedColor.red);
  frc::SmartDashboard::PutNumber("Last Detected RGB Green", m_detectedColor.green);
  frc::SmartDashboard::PutNumber("Last Detected RGB Blue", m_detectedColor.blue);
  frc::SmartDashboard::PutNumber("Last Detected RGB Min", m_minRGB);
  frc::SmartDashboard::PutNumber("Last Detected RGB Max", m_maxRGB);
  frc::SmartDashboard::PutNumber("Last Detected RGB Delta", m_deltaRGB);
  frc::SmartDashboard::PutNumber("Last Detected Hue", m_hue);
  // frc::SmartDashboard::PutNumber("Last Detected Color Confidence", m_colorConfidence);
  // frc::SmartDashboard::PutString("Last Detected Color", m_colorString);

  return m_colorString;
}

void StorageSubsystem::SetFeedMotorPower(double FeedSpeed)
{
  m_feedPercentagePower = FeedSpeed;
  // frc::SmartDashboard::PutNumber("Feed Motor Speed in Subsystem", m_feedPercentagePower);
  if (m_feedPercentagePower < 0)
  {
    m_isFeederInReverse = true;
  }
  else
  {
    m_isFeederInReverse = false;
  }
}

void StorageSubsystem::SetLoaderMotorPower(double LoaderSpeed) // Moving away from this to SetLoaderMotorTargetSpeed
{
  m_loaderPercentagePower = LoaderSpeed;
  if (m_loaderPercentagePower < 0)
  {
    m_isLoaderInReverse = true;
  }
  else
  {
    m_isLoaderInReverse = false;
  }
  m_loaderMotor.Set(ControlMode::PercentOutput, m_loaderPercentagePower);
  // frc::SmartDashboard::PutNumber("Loader Motor Speed in Subsystem", m_loaderPercentagePower);
  // frc::SmartDashboard::PutString("Loader Control Mode", "Percent Output");
}

void StorageSubsystem::SetLoaderMotorTargetSpeed(double LoaderTargetSpeed) // Moving to this rather than SetLoaderMotorPower
{
  m_loaderTargetSpeedRPM = LoaderTargetSpeed;
  m_loaderTargetSpeedUnitsPer100ms = (m_loaderTargetSpeedRPM * kFalconTicksPerRevolution) / k100MillisecondsPerMinute;
  m_loaderMotor.Config_kF(kPIDLoopIdx, kFLoaderMotor, kTimeoutMs);
  m_loaderMotor.Config_kP(kPIDLoopIdx, kPLoaderMotor, kTimeoutMs);
  m_loaderMotor.Config_kI(kPIDLoopIdx, kILoaderMotor, kTimeoutMs);
  m_loaderMotor.Config_kD(kPIDLoopIdx, kDLoaderMotor, kTimeoutMs);
  m_loaderMotor.ConfigSelectedFeedbackSensor(TalonFXFeedbackDevice::IntegratedSensor, kPIDLoopIdx, kTimeoutMs);
  m_loaderMotor.Set(ControlMode::Velocity, m_loaderTargetSpeedUnitsPer100ms);
  if (m_loaderTargetSpeedRPM < 0)
  {
    m_isLoaderInReverse = true;
  }
  else
  {
    m_isLoaderInReverse = false;
  }
  // frc::SmartDashboard::PutString("Loader Control Mode", "Velocity Control");
}

void StorageSubsystem::SetLoaderMotorTargetPosition()
{
  m_loaderTargetPosition = m_lastLoaderIncrementPosition + kLoaderMotorSecondBallPositionModifier; // Store second ball a bit past point where color sensor detects ball
  m_loaderMotor.Config_kF(kPIDLoopIdx, kFLoaderMotorPosition, kTimeoutMs);
  m_loaderMotor.Config_kP(kPIDLoopIdx, kPLoaderMotorPosition, kTimeoutMs);
  m_loaderMotor.Config_kI(kPIDLoopIdx, kILoaderMotorPosition, kTimeoutMs);
  m_loaderMotor.Config_kD(kPIDLoopIdx, kDLoaderMotorPosition, kTimeoutMs);
  m_loaderMotor.ConfigSelectedFeedbackSensor(TalonFXFeedbackDevice::IntegratedSensor, kPIDLoopIdx, kTimeoutMs);
  m_loaderMotor.Set(ControlMode::Position, m_loaderTargetPosition);
  // frc::SmartDashboard::PutString("Loader Control Mode", "Position Control");
  // frc::SmartDashboard::PutNumber("Loader Target Position", m_loaderTargetPosition);
}

void StorageSubsystem::SetReadyToShoot(bool ReadyToShoot)
{
  m_readyToShoot = ReadyToShoot;
}

void StorageSubsystem::ZeroBallCount()
{
  m_ballCount = 0;
  m_ballColorArray[0] = "Zero";
  m_ballColorArray[1] = "No ball";
  m_ballColorArray[2] = "No ball";
  m_ballColorArray[3] = "No ball";
}

void StorageSubsystem::SetTargetBallCount(int targetBallCount)
{
  m_targetBallCount = targetBallCount;
}

void StorageSubsystem::CheckBallCount()
{
  // m_colorSensorBallClose = GetColorSensorBallClose(); // Replaced with proximity sensor
  m_secondBallStorage = GetSecondBallStorage();
  if (GetFirstBallStorage())
  {
    if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue)
    {
      m_ballColorArray[1] = "Blue";
    }
    else
    {
      m_ballColorArray[1] = "Red";
    }
    frc::SmartDashboard::PutString("First Ball Color", m_ballColorArray[1]);
    if (m_secondBallStorage)
    {
      m_ballCount = 2;
      m_ballColorArray[m_ballCount] = GetColorOfBall(); // Get the colour of the ball near the colour sensor and store it in an array
      frc::SmartDashboard::PutString("Second Ball Color", m_ballColorArray[2]);
    }
    else
    {
      m_ballCount = 1;
    }
  }
  else
  {
    if (m_secondBallStorage)
    {
      m_ballCount = 1;
      m_ballColorArray[m_ballCount] = GetColorOfBall(); // Get the colour of the ball near the colour sensor and store it in an array
      frc::SmartDashboard::PutString("First Ball Color", m_ballColorArray[1]);
    }
    else
    {
      m_ballCount = 0;
    }
  }
  frc::SmartDashboard::PutNumber("Ball Count", m_ballCount); // How many balls are currently in the robot
}

void StorageSubsystem::InitialisePreloadedBallColour()
{
  m_ballCount=1;
  if (m_ballCount == 1)
  {
    if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue)
    {
      m_ballColorArray[m_ballCount] = "Blue";
    }
    else
    {
      m_ballColorArray[m_ballCount] = "Red";
    }
  }
}

void StorageSubsystem::SetIgnoreBallColorStatus(bool ignoreBallColor)
{
  m_ignoreBallColorSensor = ignoreBallColor;
}

bool StorageSubsystem::GetIgnoreBallColorStatus()
{
  return m_ignoreBallColorSensor;
}