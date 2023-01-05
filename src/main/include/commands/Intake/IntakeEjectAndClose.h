// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <commands/Intake/IntakeDriveOutwards.h>
#include <commands/Intake/IntakeIn.h>
#include <commands/Intake/IntakeStop.h>
#include "subsystems/IntakeSubsystem.h"
#include <commands/WaitForNumberOfSeconds.h>

class IntakeEjectAndClose
    : public frc2::CommandHelper<frc2::SequentialCommandGroup,
                                 IntakeEjectAndClose> {
 public:
  IntakeEjectAndClose(IntakeSubsystem* IntakeSUB,  units::time::second_t WaitSeconds);

  private:
  IntakeSubsystem* m_intakeSUB;
   units::time::second_t m_waitSeconds;

};
