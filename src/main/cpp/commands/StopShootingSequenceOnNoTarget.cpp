// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/StopShootingSequenceOnNoTarget.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
StopShootingSequenceOnNoTarget::StopShootingSequenceOnNoTarget(LimelightSubsystem* subsystem)
:m_LimelightSubsystem{subsystem} 
{
  //  This command is a race command that runs in parallel with the shooting command so that 
  //  shooting command group can be cancelled

  //AddRequirements();
}

// Called when the command is initially scheduled.
void StopShootingSequenceOnNoTarget::Initialize() {
  fmt::print("Just started StopShootingSequenceOnNoTarget\n");
}

// Called repeatedly when this Command is scheduled to run
void StopShootingSequenceOnNoTarget::Execute() {}

// Called once the command ends or is interrupted.
void StopShootingSequenceOnNoTarget::End(bool interrupted) {
  if (interrupted){
    fmt::print("StopShootingOnNoTarget Command ended beacuse it was Interrupted\n");
  }
  else {    
    fmt::print("StopShootingOnNoTarget Command ended beacuse there was no target\n");
  }
}

// Returns true when the command should end.
bool StopShootingSequenceOnNoTarget::IsFinished()
{
  // This command is used in a parallel race group with shooting logic 
  // It ends if the target isn't visible, which forces the race group to end
  // So as low as the target is valid this command will continue to run
  return (!m_LimelightSubsystem->CheckTarget()&& !m_LimelightSubsystem->m_isTargetCheckFinished);
}
