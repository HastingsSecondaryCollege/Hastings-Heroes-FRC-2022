#pragma once

// This file was created to create a simple to follow autochooser that the students could understand
// It is basically a continuation of the RobotContainer.cpp file.

#include "RobotContainer.h"
#include "commands/InitializeOdometry.h"
#include <frc2/command/WaitCommand.h>
#include <frc2/command/ScheduleCommand.h>
//#include <frc/smartdashboard/SmartDashboard.h>

//  Every chance that the implementation of the auto chooser will change
frc2::Command *RobotContainer::GetAutonomousCommand()
{
  //fmt::print("Auto Option {} Choosen\n",GetAutoChooser());
  return m_autos[GetAutoChooser()];
  //return m_autos[0];
}

/**
 * Reads the trajectories JSON files created by the PathPlaner app that are stored in the \deploy\pathplanner directory on the Rio
 * Generates the trajectories and save them into variables to be used later when SwerveControllerCommand object are created
 *
 * Naming Convention: Starts with 'Path', Then the auto number, then optional A,B,C,etc for autos with Multiple segments.
 * Then an '_' and then the .path filename from the deply/pathplanner folder with all the '-' removed.
 */

void RobotContainer::LoadAndBuildAutoTrajectories()
{

  // Read the JSON File and populate the trajectory objects
  fmt::print("Building Trajector followers\n");

  // Build the path following commands for each auto choice

  // Auto Path 1
  frc::ProfiledPIDController<units::radians> thetaController1{
      AutoConstants::kPThetaController, 0, 0,
      AutoConstants::kThetaControllerConstraints};
  thetaController1.EnableContinuousInput(units::radian_t(-wpi::numbers::pi),
                                         units::radian_t(wpi::numbers::pi));

  frc::ProfiledPIDController<units::radians> thetaController2{
      AutoConstants::kPThetaController, 0, 0,
      AutoConstants::kThetaControllerConstraints};
  thetaController2.EnableContinuousInput(units::radian_t(-wpi::numbers::pi),
                                         units::radian_t(wpi::numbers::pi));

  fmt::print("just created TestPathCommand, but it is going out of scope");

// clang-format off
    //H2_H1_H2 5 Ball Auto (All high)
    frc2::SwerveControllerCommand<4> H2_H1_H2_Part1_MoveCMD(
        Path_H2_H1_H2_Part1,[this](){ return m_drive.GetPose(); },m_drive.kDriveKinematics,
        frc2::PIDController(AutoConstants::kPXController, 0, 0), frc2::PIDController(AutoConstants::kPYController, 0, 0), thetaController1,
        [this](auto moduleStates){ m_drive.SetModuleStates(moduleStates); },{&m_drive}); // end of AutoPath1Command

    frc2::SwerveControllerCommand<4> H2_H1_H2_Part2_MoveCMD(
        Path_H2_H1_H2_Part2,[this](){ return m_drive.GetPose(); },m_drive.kDriveKinematics,
        frc2::PIDController(AutoConstants::kPXController, 0, 0), frc2::PIDController(AutoConstants::kPYController, 0, 0), thetaController1,
        [this](auto moduleStates){ m_drive.SetModuleStates(moduleStates); },{&m_drive}); // end of AutoPath1BCommand

    frc2::SwerveControllerCommand<4> H2_H1_H2_Part3_MoveCMD(
        Path_H2_H1_H2_Part3,[this](){ return m_drive.GetPose(); },m_drive.kDriveKinematics,
        frc2::PIDController(AutoConstants::kPXController, 0, 0), frc2::PIDController(AutoConstants::kPYController, 0, 0), thetaController1,
        [this](auto moduleStates){ m_drive.SetModuleStates(moduleStates); },{&m_drive}); // end of AutoPathCCommand

    frc2::SwerveControllerCommand<4> H2_H1_H2_Part4_MoveCMD(
        Path_H2_H1_H2_Part4,[this](){ return m_drive.GetPose(); },m_drive.kDriveKinematics,
        frc2::PIDController(AutoConstants::kPXController, 0, 0), frc2::PIDController(AutoConstants::kPYController, 0, 0), thetaController1,
        [this](auto moduleStates){ m_drive.SetModuleStates(moduleStates); },{&m_drive}); // end of AutoPathCCommand
   
        //L1_H2 Paths 3 Ball Auto (1 Low, 2 High)
    frc2::SwerveControllerCommand<4> L1_H2_Part1_MoveCMD(
        Path_L1_H2_H2_Part1,[this](){ return m_drive.GetPose(); },m_drive.kDriveKinematics,
        frc2::PIDController(AutoConstants::kPXController, 0, 0), frc2::PIDController(AutoConstants::kPYController, 0, 0), thetaController1,
        [this](auto moduleStates){ m_drive.SetModuleStates(moduleStates); },{&m_drive}); // end of AutoPathCCommand


        //L1_H2_H2 Paths 5 Ball Auto (1 Low, 4 High)
    frc2::SwerveControllerCommand<4> L1_H2_H2_Part1_MoveCMD(
        Path_L1_H2_H2_Part1,[this](){ return m_drive.GetPose(); },m_drive.kDriveKinematics,
        frc2::PIDController(AutoConstants::kPXController, 0, 0), frc2::PIDController(AutoConstants::kPYController, 0, 0), thetaController1,
        [this](auto moduleStates){ m_drive.SetModuleStates(moduleStates); },{&m_drive}); // end of AutoPathCCommand

    frc2::SwerveControllerCommand<4> L1_H2_H2_Part2_MoveCMD(
        Path_L1_H2_H2_Part2,[this](){ return m_drive.GetPose(); },m_drive.kDriveKinematics,
        frc2::PIDController(AutoConstants::kPXController, 0, 0), frc2::PIDController(AutoConstants::kPYController, 0, 0), thetaController1,
        [this](auto moduleStates){ m_drive.SetModuleStates(moduleStates); },{&m_drive}); // end of AutoPathCCommand
    
    frc2::SwerveControllerCommand<4> L1_H2_H2_Part2B_MoveCMD(
        Path_L1_H2_H2_Part2B,[this](){ return m_drive.GetPose(); },m_drive.kDriveKinematics,
        frc2::PIDController(AutoConstants::kPXController, 0, 0), frc2::PIDController(AutoConstants::kPYController, 0, 0), thetaController1,
        [this](auto moduleStates){ m_drive.SetModuleStates(moduleStates); },{&m_drive}); // end of AutoPathCCommand
    
    frc2::SwerveControllerCommand<4> L1_H2_H2_Part3_MoveCMD(
        Path_L1_H2_H2_Part3,[this](){ return m_drive.GetPose(); },m_drive.kDriveKinematics,
        frc2::PIDController(AutoConstants::kPXController, 0, 0), frc2::PIDController(AutoConstants::kPYController, 0, 0), thetaController1,
        [this](auto moduleStates){ m_drive.SetModuleStates(moduleStates); },{&m_drive}); // end of AutoPathCCommand

            //H2_H2 Paths 4 Ball Auto (all High)
    frc2::SwerveControllerCommand<4> H2_H2_Part1_MoveCMD(
        Path_H2_H2_Part1,[this](){ return m_drive.GetPose(); },m_drive.kDriveKinematics,
        frc2::PIDController(AutoConstants::kPXController, 0, 0), frc2::PIDController(AutoConstants::kPYController, 0, 0), thetaController1,
        [this](auto moduleStates){ m_drive.SetModuleStates(moduleStates); },{&m_drive}); // end of AutoPathCCommand
    
     frc2::SwerveControllerCommand<4> H2_H2_Part2_MoveCMD(
        Path_H2_H2_Part2,[this](){ return m_drive.GetPose(); },m_drive.kDriveKinematics,
        frc2::PIDController(AutoConstants::kPXController, 0, 0), frc2::PIDController(AutoConstants::kPYController, 0, 0), thetaController1,
        [this](auto moduleStates){ m_drive.SetModuleStates(moduleStates); },{&m_drive}); // end of AutoPathCCommand


    frc2::SwerveControllerCommand<4> H2_H2_Return_MoveCMD(
        Path_4BallReset,[this](){ return m_drive.GetPose(); },m_drive.kDriveKinematics,
        frc2::PIDController(AutoConstants::kPXController, 0, 0), frc2::PIDController(AutoConstants::kPYController, 0, 0), thetaController1,
        [this](auto moduleStates){ m_drive.SetModuleStates(moduleStates); },{&m_drive}); // end of AutoPathCCommand
    
    
            //H2 From LEL Paths 2 Ball Auto (all High)
    frc2::SwerveControllerCommand<4> H2_Part1_MoveCMD(
        Path_H2_Part1,[this](){ return m_drive.GetPose(); },m_drive.kDriveKinematics,
        frc2::PIDController(AutoConstants::kPXController, 0, 0), frc2::PIDController(AutoConstants::kPYController, 0, 0), thetaController1,
        [this](auto moduleStates){ m_drive.SetModuleStates(moduleStates); },{&m_drive}); // end of AutoPathCCommand
 
    frc2::SwerveControllerCommand<4> H2_Part2_MoveCMD(
        Path_H2_Part2,[this](){ return m_drive.GetPose(); },m_drive.kDriveKinematics,
        frc2::PIDController(AutoConstants::kPXController, 0, 0), frc2::PIDController(AutoConstants::kPYController, 0, 0), thetaController1,
        [this](auto moduleStates){ m_drive.SetModuleStates(moduleStates); },{&m_drive}); // end of AutoPathCCommand
 
             //H2 From REL Paths 2 Ball Auto (all High)
    frc2::SwerveControllerCommand<4> H2_Part1_REL_MoveCMD(
        Path_H2_H2_Part1,[this](){ return m_drive.GetPose(); },m_drive.kDriveKinematics,
        frc2::PIDController(AutoConstants::kPXController, 0, 0), frc2::PIDController(AutoConstants::kPYController, 0, 0), thetaController1,
        [this](auto moduleStates){ m_drive.SetModuleStates(moduleStates); },{&m_drive}); // end of AutoPathCCommand

            //Test Paths
    frc2::SwerveControllerCommand<4> ResetFrom5BallFinish(  
        Path_5BallReset,[this](){ return m_drive.GetPose(); },m_drive.kDriveKinematics,
        frc2::PIDController(AutoConstants::kPXController, 0, 0), frc2::PIDController(AutoConstants::kPYController, 0, 0), thetaController1,
        [this](auto moduleStates){ m_drive.SetModuleStates(moduleStates); },{&m_drive}); // end of AutoPathCCommand

    frc2::SwerveControllerCommand<4> TestPathCMD(  
        Path_1MTestPath,[this](){ return m_drive.GetPose(); },m_drive.kDriveKinematics,
        frc2::PIDController(AutoConstants::kPXController, 0, 0), frc2::PIDController(AutoConstants::kPYController, 0, 0), thetaController1,
        [this](auto moduleStates){ m_drive.SetModuleStates(moduleStates); },{&m_drive}); // end of AutoPathCCommand


#define __AUTO_SHOOT SetCalculatedShooterSpeedsThenFeedWhenReady{&m_shooterSUB,&m_storageSUB,&m_drive,&m_limelightSUB}

#define __AUTO_INTAKE frc2::ScheduleCommand(new IntakeOutAndIntakeDriveInwards{&m_intakeSUB,&m_storageSUB,kLoaderMotorTestSpeed})

#define __AUTO_STOP frc2::InstantCommand{[this] {m_drive.DriveStop();}}

    //4Ball:H2_H2 Setup REL, pickup 2 ball, Auto Shoot high 1&2, pickup 3&4 ball, Autoshoot High.
m_autos[0] = new frc2::SequentialCommandGroup{
            frc2::InstantCommand([]{ fmt::print("Started H2_H2 Auto at {}\n",frc::Timer::GetFPGATimestamp());}),
            InitializeOdometry(&m_drive, Path_H2_H2_Part1.getInitialState()->pose.Translation(), Path_H2_H2_Part1.getInitialState()->holonomicRotation), 
            __AUTO_INTAKE,
            std::move(H2_H2_Part1_MoveCMD),
            __AUTO_STOP,
            frc2::ScheduleCommand(new frc2::SequentialCommandGroup{
                frc2::InstantCommand([this]{m_shooterSUB.SetDistanceToTarget(3750.0);}), //Manually set distance incase limelight is offline/ not recognising the target
                __AUTO_SHOOT,           //Shoot Ball 1 & 2
                __AUTO_INTAKE,
                frc2::ScheduleCommand(new frc2::SequentialCommandGroup{
                    std::move(H2_H2_Part2_MoveCMD),
                    __AUTO_STOP,
                    frc2::ScheduleCommand(new frc2::SequentialCommandGroup{
                        frc2::InstantCommand([this]{m_shooterSUB.SetDistanceToTarget(4800.0);}),  //Manually set distance incase limelight is offline/ not recognising the target
                        __AUTO_SHOOT       //Shoot Balls 3&4
                        //frc2::WaitCommand(2_s),
                        //frc2::ScheduleCommand(new frc2::SequentialCommandGroup{
                        //    std::move(H2_H2_Return_MoveCMD),
                        //    __AUTO_STOP
                        //})
                    })
                })
            })
        };              

    //2Ball:H2 Setup LEL, pickup 2nd ball, Auto Shoot high 1&2, pickup opponent ball eject under climb bars.
m_autos[1] = new frc2::SequentialCommandGroup{
            frc2::InstantCommand([]{ fmt::print("Started H2 Auto at {}\n",frc::Timer::GetFPGATimestamp());}),
            InitializeOdometry(&m_drive, Path_H2_Part1.getInitialState()->pose.Translation(), Path_H2_Part1.getInitialState()->holonomicRotation), 
            __AUTO_INTAKE,
            std::move(H2_Part1_MoveCMD),
            __AUTO_STOP,
            frc2::ScheduleCommand(new frc2::SequentialCommandGroup{
                frc2::InstantCommand([this]{m_shooterSUB.SetDistanceToTarget(3650.0);}),  //Manually set distance incase limelight is offline/ not recognising the target
                __AUTO_SHOOT,           //Shoot Ball 1 & 2
                __AUTO_INTAKE,
                frc2::ScheduleCommand(new frc2::SequentialCommandGroup{
                    std::move(H2_Part2_MoveCMD), //Pickup opponent Ball
                    __AUTO_STOP,
                    frc2::ScheduleCommand(new frc2::SequentialCommandGroup{
                        frc2::InstantCommand([this]{m_shooterSUB.SetDistanceToTarget(0.0);}),
                        __AUTO_SHOOT,       //Shoot Opponent Ball into Corner
                        frc2::InstantCommand([]{ fmt::print("Finished H2 Auto at {}\n",frc::Timer::GetFPGATimestamp());})
                    })
                })
            })
        };              

    //5Ball:H2_H1_H2 Setup RER, pickup 2 ball, Auto Shoot high 1&2, pickup 3 ball, Autoshoot High, pickup 4 & Human Ball, Autoshoot High 
m_autos[2] = new frc2::SequentialCommandGroup{
            frc2::InstantCommand([]{ fmt::print("Started Auto at {}\n",frc::Timer::GetFPGATimestamp());}),
            InitializeOdometry(&m_drive, Path_H2_H1_H2_Part1.getInitialState()->pose.Translation(), Path_H2_H1_H2_Part1.getInitialState()->holonomicRotation), 
            __AUTO_INTAKE,
            std::move(H2_H1_H2_Part1_MoveCMD),
            __AUTO_STOP,
            frc2::ScheduleCommand(new frc2::SequentialCommandGroup{
                frc2::InstantCommand([this]{m_shooterSUB.SetDistanceToTarget(3201.0);}),  //Manually set distance incase limelight is offline/ not recognising the target
                __AUTO_SHOOT,           //Shoot Ball 1 & 2
                __AUTO_INTAKE,
                frc2::ScheduleCommand(new frc2::SequentialCommandGroup{
                    std::move(H2_H1_H2_Part2_MoveCMD),
                    __AUTO_STOP,
                    frc2::ScheduleCommand(new frc2::SequentialCommandGroup{
                        frc2::InstantCommand([this]{m_shooterSUB.SetDistanceToTarget(3750.0);}),  //Manually set distance incase limelight is offline/ not recognising the target
                        __AUTO_SHOOT,       //Shoot Ball 3
                        __AUTO_INTAKE,
                        frc2::ScheduleCommand(new frc2::SequentialCommandGroup{
                            std::move(H2_H1_H2_Part3_MoveCMD),
                            __AUTO_STOP,
                            frc2::WaitCommand(0.20_s), //wait for 0.2 sec to get next ball
                            std::move(H2_H1_H2_Part4_MoveCMD),
                            __AUTO_STOP,
                            frc2::ScheduleCommand(new frc2::SequentialCommandGroup{
                                frc2::InstantCommand([this]{m_shooterSUB.SetDistanceToTarget(4800.0);}),//Manually set distance incase limelight is offline/ not recognising the target
                                __AUTO_SHOOT,     //Shoot Ball 4 & 5
                                frc2::InstantCommand([]{ fmt::print("Finished Auto at {}\n",frc::Timer::GetFPGATimestamp());}),
                            })
                        })
                    })
                })
            })
        };              

    //5Ball:L1_H2_H2 Setup RER, Auto shoot low, pickup 2 & 3 ball, Autoshoot High, pickup 4 & Human Ball, Autoshoot High 
m_autos[3] = new frc2::SequentialCommandGroup{
            frc2::InstantCommand([]{ fmt::print("Started L1-H2-H2 Auto at {}\n",frc::Timer::GetFPGATimestamp());}),
            InitializeOdometry(&m_drive, Path_L1_H2_H2_Part1.getInitialState()->pose.Translation(), Path_L1_H2_H2_Part1.getInitialState()->holonomicRotation), 
            frc2::InstantCommand([this]{m_shooterSUB.SetDistanceToTarget(2400.0);}),  //Manually set distance incase limelight is offline/ not recognising the target
             __AUTO_SHOOT,           //Shoot preloaded into Low Goal 
            __AUTO_INTAKE,
            frc2::ScheduleCommand(new frc2::SequentialCommandGroup{
                std::move(L1_H2_H2_Part1_MoveCMD),      //Pick 2nd & 3rd Ball
                __AUTO_STOP,
                frc2::ScheduleCommand(new frc2::SequentialCommandGroup{
                    frc2::InstantCommand([this]{m_shooterSUB.SetDistanceToTarget(3750.0);}),  //Manually set distance incase limelight is offline/ not recognising the target
                    __AUTO_SHOOT,           //Shoot 2nd & 3rd High
                    __AUTO_INTAKE,
                    frc2::ScheduleCommand(new frc2::SequentialCommandGroup{
                        std::move(L1_H2_H2_Part2_MoveCMD),  //Pickup 4th & 5th
                        __AUTO_STOP,
                        frc2::WaitCommand(1_s),     //Wait 1 Second to give time to load 5th ball
                        std::move(L1_H2_H2_Part2B_MoveCMD), //Move to Shooting Position for Ball 4&5
                        __AUTO_STOP,    
                        frc2::ScheduleCommand(new frc2::SequentialCommandGroup{
                            frc2::InstantCommand([this]{m_shooterSUB.SetDistanceToTarget(4800.0);}), //Manually set distance incase limelight is offline/ not recognising the target                         
                            __AUTO_SHOOT,       //Shoot 4th & 5th
                            frc2::ScheduleCommand(new frc2::SequentialCommandGroup{
                                std::move(L1_H2_H2_Part3_MoveCMD),  //Spin ready to intake
                                __AUTO_STOP,
                                frc2::InstantCommand([]{ fmt::print("Finished Auto at {}\n",frc::Timer::GetFPGATimestamp());}),
                            })
                        })
                    })
                })
            })
        };            

// 5 Ball Auto Before Modification
// m_autos[3] = new frc2::SequentialCommandGroup{
//             frc2::InstantCommand([]{ fmt::print("Started L1-H2-H2 Auto at {}\n",frc::Timer::GetFPGATimestamp());}),
//             InitializeOdometry(&m_drive, Path_L1_H2_H2_Part1.getInitialState()->pose.Translation(), Path_L1_H2_H2_Part1.getInitialState()->holonomicRotation), 
//             frc2::InstantCommand([this]{m_shooterSUB.SetDistanceToTarget(2400.0);}),  //Manually set distance incase limelight is offline/ not recognising the target
//              __AUTO_SHOOT,           //Shoot preloaded into Low Goal 
//             __AUTO_INTAKE,
//             frc2::ScheduleCommand(new frc2::SequentialCommandGroup{
//                 std::move(L1_H2_H2_Part1_MoveCMD),      //Pick 2nd & 3rd Ball
//                 __AUTO_STOP,
//                 frc2::ScheduleCommand(new frc2::SequentialCommandGroup{
//                     frc2::InstantCommand([this]{m_shooterSUB.SetDistanceToTarget(3750.0);}),  //Manually set distance incase limelight is offline/ not recognising the target
//                     __AUTO_SHOOT,           //Shoot 2nd & 3rd High
//                     __AUTO_INTAKE,
//                     frc2::ScheduleCommand(new frc2::SequentialCommandGroup{
//                         std::move(L1_H2_H2_Part2_MoveCMD),  //Pickup 4th & 5th
//                         __AUTO_STOP,
//                         frc2::ScheduleCommand(new frc2::SequentialCommandGroup{
//                             frc2::InstantCommand([this]{m_shooterSUB.SetDistanceToTarget(4800.0);}), //Manually set distance incase limelight is offline/ not recognising the target                         
//                             __AUTO_SHOOT,       //Shoot 4th & 5th
//                             frc2::ScheduleCommand(new frc2::SequentialCommandGroup{
//                                 std::move(L1_H2_H2_Part3_MoveCMD),  //Spin ready to intake
//                                 __AUTO_STOP,
//                                 frc2::InstantCommand([]{ fmt::print("Finished Auto at {}\n",frc::Timer::GetFPGATimestamp());}),
//                             })
//                         })
//                     })
//                 })
//             })
//         };            

    //3Ball:L1_H2 Setup RER, Auto shoot low, pickup 2 & 3 ball, Autoshoot High. 
m_autos[4] = new frc2::SequentialCommandGroup{
            frc2::InstantCommand([]{ fmt::print("Started Auto at {}\n",frc::Timer::GetFPGATimestamp());}),
            InitializeOdometry(&m_drive, Path_L1_H2_H2_Part1.getInitialState()->pose.Translation(), Path_L1_H2_H2_Part1.getInitialState()->holonomicRotation),
            frc2::InstantCommand([this]{m_shooterSUB.SetDistanceToTarget(2400.0);}), 
             __AUTO_SHOOT,           //Shoot preloaded into Low Goal 
            __AUTO_INTAKE,
            frc2::ScheduleCommand(new frc2::SequentialCommandGroup{
                std::move(L1_H2_Part1_MoveCMD),      //Pick 2nd & 3rd Ball
                __AUTO_STOP,
                frc2::ScheduleCommand(new frc2::SequentialCommandGroup{
                    frc2::InstantCommand([this]{m_shooterSUB.SetDistanceToTarget(3750.0);}),
                    __AUTO_SHOOT
                })
            })
        };            

    //2Ball:H2 Setup REL, pickup 2 ball, Auto Shoot high 1&2.
m_autos[5] = new frc2::SequentialCommandGroup{
            frc2::InstantCommand([]{ fmt::print("Started H2 REL Auto at {}\n",frc::Timer::GetFPGATimestamp());}),
            InitializeOdometry(&m_drive, Path_H2_H2_Part1.getInitialState()->pose.Translation(), Path_H2_H2_Part1.getInitialState()->holonomicRotation), 
            __AUTO_INTAKE,
            std::move(H2_Part1_REL_MoveCMD),
            __AUTO_STOP,
            frc2::ScheduleCommand(new frc2::SequentialCommandGroup{
                frc2::InstantCommand([this]{m_shooterSUB.SetDistanceToTarget(3750.0);}),
                __AUTO_SHOOT           //Shoot Ball 1 & 2    
            })
        }; 

  // clang-format on

} // End of LoadAndBuildAutoTrajectories()
