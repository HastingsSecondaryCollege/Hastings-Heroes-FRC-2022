// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Intake/IntakeEjectAndClose.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
IntakeEjectAndClose::IntakeEjectAndClose(IntakeSubsystem* IntakeSUB, units::time::second_t WaitSeconds)
 : m_intakeSUB{IntakeSUB}, 
 m_waitSeconds{WaitSeconds}
 {
  // Add your commands here, e.g.
  // AddCommands(FooCommand(), BarCommand());
   AddCommands(
   IntakeDriveOutwards(m_intakeSUB),
   IntakeIn(m_intakeSUB),
   WaitForNumberOfSeconds(m_waitSeconds),
   IntakeStop(m_intakeSUB)
   );
}
