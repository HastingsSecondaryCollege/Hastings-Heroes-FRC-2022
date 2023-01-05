// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/button/Trigger.h>
#include "subsystems/StorageSubsystem.h"

class BallCountEqualsTwo : public frc2::Trigger{
 public:
  BallCountEqualsTwo(StorageSubsystem *storageSUB);
  std::function<bool()> m_isActive; // This gives an error when I add override. Both with capital and lower case g.
  
  private:
    StorageSubsystem *m_storageSUB;
};
