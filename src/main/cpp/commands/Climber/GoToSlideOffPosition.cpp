// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Climber/GoToSlideOffPosition.h"

GoToSlideOffPosition::GoToSlideOffPosition(ClimberSubsystem * ClimberSUB) 
:m_climberSUB{ClimberSUB} {
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void GoToSlideOffPosition::Initialize() {
  m_climberSUB->SetClimbMotorToCoastMode();   // Set to coast mode incase match timer expires during extension this step.
                                              // This will let the robot fall enough to come off the bar to score the next highest rung. 
  m_climberSUB->MotionMagicToPosition(kDisconnectFromBarWithPoweredHookPosition);
}

// Called repeatedly when this Command is scheduled to run
void GoToSlideOffPosition::Execute() {}

// Called once the command ends or is interrupted.
void GoToSlideOffPosition::End(bool interrupted) {
  m_climberSUB->SetClimbMotorToBrakeMode();   // Turn brake mode back on as we only want coast for this command only.
}

// Returns true when the command should end.
bool GoToSlideOffPosition::IsFinished() {
  return m_climberSUB->HasMotionMagicFinished();
}
