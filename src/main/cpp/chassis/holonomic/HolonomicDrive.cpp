//====================================================================================================================================================
// Copyright 2022 Lake Orion Robotics FIRST Team 302
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE
// OR OTHER DEALINGS IN THE SOFTWARE.
//====================================================================================================================================================

// C++ Includes
#include <algorithm>
#include <memory>
#include <string>

// FRC includes
#include <units/velocity.h>
#include <units/angular_velocity.h>
#include <frc/kinematics/ChassisSpeeds.h>

// Team 302 Includes
#include <chassis/ChassisMovement.h>
#include <chassis/ChassisOptionEnums.h>
#include <chassis/holonomic/HolonomicDrive.h>
#include <chassis/IChassis.h>
#include <hw/DragonPigeon.h>
#include <gamepad/IDragonGamePad.h>
#include <TeleopControl.h>
#include <TeleopControlFunctions.h>
#include <State.h>
#include <chassis/ChassisFactory.h>
#include <hw/factories/PigeonFactory.h>
#include <utils/Logger.h>
#include <chassis/swerve/driveStates/DragonTrajectoryGenerator.h>

using namespace std;
using namespace frc;

/// @brief initialize the object and validate the necessary items are not nullptrs
HolonomicDrive::HolonomicDrive() : State(string("HolonomicDrive"), -1),
                                   m_chassis(ChassisFactory::GetChassisFactory()->GetIChassis()),
                                   m_controller(TeleopControl::GetInstance()),
                                   m_swerve(ChassisFactory::GetChassisFactory()->GetSwerveChassis()),
                                   m_mecanum(ChassisFactory::GetChassisFactory()->GetMecanumChassis()),
                                   m_trajectoryGenerator(new DragonTrajectoryGenerator(ChassisFactory::GetChassisFactory()->GetSwerveChassis()->GetMaxSpeed(),
                                                                                ChassisFactory::GetChassisFactory()->GetSwerveChassis()->GetMaxAcceleration()))
{
    if (m_controller == nullptr)
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, string("HolonomicDrive"), string("Constructor"), string("TeleopControl is nullptr"));
    }
}

/// @brief initialize the profiles for the various gamepad inputs
/// @return void
void HolonomicDrive::Init()
{
    auto controller = GetController();
    if (controller != nullptr)
    {
        controller->SetAxisProfile(TeleopControl::FUNCTION_IDENTIFIER::HOLONOMIC_DRIVE_FORWARD, IDragonGamePad::AXIS_PROFILE::CUBED);
        controller->SetDeadBand(TeleopControl::FUNCTION_IDENTIFIER::HOLONOMIC_DRIVE_FORWARD, IDragonGamePad::AXIS_DEADBAND::APPLY_STANDARD_DEADBAND);
        controller->SetAxisScaleFactor(TeleopControl::FUNCTION_IDENTIFIER::HOLONOMIC_DRIVE_FORWARD, 0.6);

        controller->SetAxisProfile(TeleopControl::FUNCTION_IDENTIFIER::HOLONOMIC_DRIVE_STRAFE, IDragonGamePad::AXIS_PROFILE::CUBED);
        controller->SetDeadBand(TeleopControl::FUNCTION_IDENTIFIER::HOLONOMIC_DRIVE_STRAFE, IDragonGamePad::AXIS_DEADBAND::APPLY_STANDARD_DEADBAND);
        controller->SetAxisScaleFactor(TeleopControl::FUNCTION_IDENTIFIER::HOLONOMIC_DRIVE_STRAFE, -0.6);

        controller->SetAxisProfile(TeleopControl::FUNCTION_IDENTIFIER::HOLONOMIC_DRIVE_ROTATE, IDragonGamePad::AXIS_PROFILE::CUBED);
        controller->SetDeadBand(TeleopControl::FUNCTION_IDENTIFIER::HOLONOMIC_DRIVE_ROTATE, IDragonGamePad::AXIS_DEADBAND::APPLY_STANDARD_DEADBAND);
        controller->SetAxisScaleFactor(TeleopControl::FUNCTION_IDENTIFIER::HOLONOMIC_DRIVE_ROTATE, 0.5);
    }
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("HolonomicDrive::Init"), string("end"));   
}

/// @brief calculate the output for the wheels on the chassis from the throttle and steer components
/// @return void
void HolonomicDrive::Run()
{
    ChassisMovement moveInfo;
    moveInfo.driveOption = ChassisOptionEnums::DriveStateType::FIELD_DRIVE;
    moveInfo.controllerType = ChassisOptionEnums::AutonControllerType::HOLONOMIC;

    auto controller = GetController();
    if (controller != nullptr && m_chassis != nullptr)
    {
        moveInfo.headingOption = ChassisOptionEnums::HeadingOption::MAINTAIN;
        if (controller->IsButtonPressed(TeleopControlFunctions::FINDTARGET))
        {
            moveInfo.headingOption = ChassisOptionEnums::HeadingOption::TOWARD_GOAL;
        }                                       
        else if (controller->IsButtonPressed(TeleopControlFunctions::FUNCTION::DRIVE_TO_SHOOTING_SPOT))
        {
            moveInfo.headingOption = ChassisOptionEnums::HeadingOption::TOWARD_GOAL;
        }
        
        if (controller->IsButtonPressed(TeleopControlFunctions::FUNCTION::REZERO_PIGEON))
        {
            auto factory = PigeonFactory::GetFactory();
            auto m_pigeon = factory->GetPigeon(DragonPigeon::PIGEON_USAGE::CENTER_OF_ROBOT);
            m_pigeon->ReZeroPigeon(0.0, 0.0);
            if (m_swerve != nullptr)
            {
                m_swerve->ZeroAlignSwerveModules();
                m_swerve->ReZero();
            }
        }

        if (m_swerve != nullptr)
        {
            auto wheelbase = m_swerve->GetWheelBase();
            auto track = m_swerve->GetTrack();

            if (controller->IsButtonPressed(TeleopControlFunctions::FUNCTION::HOLONOMIC_ROTATE_FRONT))
            {
                moveInfo.centerOfRotationOffset.X = wheelbase/2.0;
                moveInfo.centerOfRotationOffset.Y = units::length::inch_t(0.0);
            }
            else if (controller->IsButtonPressed(TeleopControlFunctions::FUNCTION::HOLONOMIC_ROTATE_RIGHT))
            {
                moveInfo.centerOfRotationOffset.X = units::length::inch_t(0.0);
                moveInfo.centerOfRotationOffset.Y = track/2.0;
            }
            else if (controller->IsButtonPressed(TeleopControlFunctions::FUNCTION::HOLONOMIC_ROTATE_LEFT))
            {
                moveInfo.centerOfRotationOffset.X = units::length::inch_t(0.0);
                moveInfo.centerOfRotationOffset.Y = -1.0*track/2.0;
        }
            else if (controller->IsButtonPressed(TeleopControlFunctions::FUNCTION::HOLONOMIC_ROTATE_BACK))
            {
                moveInfo.centerOfRotationOffset.X = -1.0*wheelbase/2.0;
                moveInfo.centerOfRotationOffset.Y = units::length::inch_t(0.0);
            }
            else 
            {
                moveInfo.centerOfRotationOffset.X = units::length::inch_t(0.0);
                moveInfo.centerOfRotationOffset.Y = units::length::inch_t(0.0);
            }
        }

        //Automated driving
        if (controller->IsButtonPressed(TeleopControl::FUNCTION_IDENTIFIER::DRIVE_TO_COL_ONE))
        {
            moveInfo.driveOption = ChassisOptionEnums::DriveStateType::TRAJECTORY_DRIVE;
            
            moveInfo.trajectory = m_trajectoryGenerator->GenerateTrajectory(ChassisFactory::GetChassisFactory()->GetSwerveChassis()->GetPose(), DragonTrajectoryGenerator::TARGET_POSITION::COLUMN_ONE);
        }

        else if (controller->IsButtonPressed(TeleopControl::FUNCTION_IDENTIFIER::DRIVE_TO_COL_TWO))
        {
            moveInfo.driveOption = ChassisOptionEnums::DriveStateType::TRAJECTORY_DRIVE;
            moveInfo.trajectory = m_trajectoryGenerator->GenerateTrajectory(ChassisFactory::GetChassisFactory()->GetSwerveChassis()->GetPose(), DragonTrajectoryGenerator::TARGET_POSITION::COLUMN_TWO);
        }
        else if (controller->IsButtonPressed(TeleopControl::FUNCTION_IDENTIFIER::DRIVE_TO_COL_THREE))
        {
            moveInfo.driveOption = ChassisOptionEnums::DriveStateType::TRAJECTORY_DRIVE;
            moveInfo.trajectory = m_trajectoryGenerator->GenerateTrajectory(ChassisFactory::GetChassisFactory()->GetSwerveChassis()->GetPose(), DragonTrajectoryGenerator::TARGET_POSITION::COLUMN_THREE);
        }
        //add button to drive to loading zone
        else //if we aren't trying to automatically drive anywhere, take us out of trajectorydrive
        {
            moveInfo.driveOption = ChassisOptionEnums::DriveStateType::FIELD_DRIVE;
        }

        //if (controller->IsButtonPressed(TeleopControl::FUNCTION_IDENTIFIER::HOLD_POSITION))
        //{
            //m_chassis.get()->DriveHoldPosition();
        //}

        auto maxSpeed = m_chassis->GetMaxSpeed();
        auto maxAngSpeed = m_chassis->GetMaxAngularSpeed();

        auto forward = controller->GetAxisValue(TeleopControlFunctions::FUNCTION::HOLONOMIC_DRIVE_FORWARD);
        auto strafe = controller->GetAxisValue(TeleopControlFunctions::FUNCTION::HOLONOMIC_DRIVE_STRAFE);
        auto rotate = controller->GetAxisValue(TeleopControlFunctions::FUNCTION::HOLONOMIC_DRIVE_ROTATE);

        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("HolonomicDrive"), string("Run"), string("axis read"));
        
        moveInfo.chassisSpeeds.vx = forward*maxSpeed;
        moveInfo.chassisSpeeds.vy = strafe*maxSpeed;
        moveInfo.chassisSpeeds.omega = rotate*maxAngSpeed;

        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("HolonomicDrive"), string("Vx"), moveInfo.chassisSpeeds.vx.to<double>());
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("HolonomicDrive"), string("Vy"), moveInfo.chassisSpeeds.vy.to<double>());

        m_chassis->Drive(moveInfo);
    }
    else
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("HolonomicDrive"), string("Run"), string("nullptr"));
    }
}

void HolonomicDrive::Exit()
{
}

/// @brief indicates that we are not at our target
/// @return bool
bool HolonomicDrive::AtTarget() const
{
    return false;
}