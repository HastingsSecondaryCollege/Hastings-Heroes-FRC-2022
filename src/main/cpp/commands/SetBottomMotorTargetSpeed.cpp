// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/SetBottomMotorTargetSpeed.h"
//#include <frc/smartdashboard/SmartDashboard.h>

SetBottomMotorTargetSpeed::SetBottomMotorTargetSpeed(ShooterSubsystem *subsystem, std::function<double()> MotorSpeed)
    : m_subsystem{subsystem},
      m_motorTargetSpeedFunction{std::move(MotorSpeed)}
{
  // Use addRequirements() here to declare subsystem dependencies.
  // AddRequirements({subsystem}); // We will only use AddRequirements statements if required
}

// Called when the command is initially scheduled.
void SetBottomMotorTargetSpeed::Initialize()
{
  //fmt::print("FPGA Time: {}\n",frc::Timer::GetFPGATimestamp());
  //fmt::print("SHOOT_LOW\n");
  
  m_subsystem->SetBottomMotorState(true);
}

// Called repeatedly when this Command is scheduled to run
void SetBottomMotorTargetSpeed::Execute()
{
  //fmt::print("Calling m_subsytem->SetBottomMotorTargetSpeed() with this value{} ",m_motorTargetSpeedFunction());
  m_subsystem->SetBottomMotorTargetSpeed(m_motorTargetSpeedFunction());
}

// Called once the command ends or is interrupted.
void SetBottomMotorTargetSpeed::End(bool interrupted) {}

// Returns true when the command should end.
bool SetBottomMotorTargetSpeed::IsFinished()
{
  return true;
}
