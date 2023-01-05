// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Intake/EjectIntakeAndLoaderAndFeeder.h"
#include "commands/Intake/IntakeDriveOutwards.h"
#include "commands/SetLoaderMotorSpeed.h"

#include <commands/SetFeedMotorSpeed.h>
#include <commands/WaitForNumberOfSeconds.h>
#include <commands/Intake/IntakeStop.h>


using namespace StorageConstants;


// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
EjectIntakeAndLoaderAndFeeder::EjectIntakeAndLoaderAndFeeder(IntakeSubsystem* IntakeSUB, StorageSubsystem* StorageSUB, units::time::second_t WaitTime) 


: m_intakeSUB{IntakeSUB},
  m_storageSUB{StorageSUB},
  m_waitTime{WaitTime}
  {
    AddCommands(
      IntakeDriveOutwards(m_intakeSUB),
      SetLoaderMotorSpeed(m_storageSUB, kLoaderMotorReversePower),
      SetFeedMotorSpeed(m_storageSUB, kFeederMotorReversePower),
      WaitForNumberOfSeconds(m_waitTime),
      SetFeedMotorSpeed(m_storageSUB, 0),
      SetLoaderMotorSpeed(m_storageSUB, 0),
      IntakeStop(m_intakeSUB)
    );
  // Add your commands here, e.g.
  // AddCommands(FooCommand(), BarCommand());

}
