// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Climber/StopClimberSequence.h"

StopClimberSequence::StopClimberSequence(frc::Joystick * buttonBox, ClimberSubsystem * ClimberSUB)
  :m_ClimbButtonBox{buttonBox},
  m_ClimberSUB{ClimberSUB}{
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void StopClimberSequence::Initialize() {
}

// Called repeatedly when this Command is scheduled to run
void StopClimberSequence::Execute() {}

// Called once the command ends or is interrupted.
void StopClimberSequence::End(bool interrupted) {
      m_ClimberSUB->KillClimbMotor();
}

// Returns true when the command should end.
bool StopClimberSequence::IsFinished() {
  return m_ClimbButtonBox->GetRawButton(5);
}
