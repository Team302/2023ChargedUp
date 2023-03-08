//====================================================================================================================================================
// Copyright 2023 Lake Orion Robotics FIRST Team 302
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
#include <teleopcontrol/TeleopControl.h>
#include <teleopcontrol/TeleopControlFunctions.h>
#include <State.h>
#include <chassis/ChassisFactory.h>
#include <hw/factories/PigeonFactory.h>
#include <utils/logging/Logger.h>
#include <chassis/swerve/driveStates/DragonTrajectoryGenerator.h>
#include <utils/DragonField.h>
#include <DragonVision/DragonVision.h>
#include <chassis/swerve/driveStates/VisionDrive.h>

using namespace std;
using namespace frc;

/// @brief initialize the object and validate the necessary items are not nullptrs
HolonomicDrive::HolonomicDrive() : State(string("HolonomicDrive"), -1),
                                   m_chassis(ChassisFactory::GetChassisFactory()->GetIChassis()),
                                   m_swerve(ChassisFactory::GetChassisFactory()->GetSwerveChassis()),
                                   m_mecanum(ChassisFactory::GetChassisFactory()->GetMecanumChassis()),
                                   m_trajectoryGenerator(new DragonTrajectoryGenerator(ChassisFactory::GetChassisFactory()->GetSwerveChassis()->GetMaxSpeed(),
                                                                                       ChassisFactory::GetChassisFactory()->GetSwerveChassis()->GetMaxAcceleration())),
                                   m_previousDriveState(ChassisOptionEnums::DriveStateType::FIELD_DRIVE),
                                   m_generatedTrajectory(frc::Trajectory()) //,
                                                                            // m_field(DragonField::GetInstance())
{
}

/// @brief initialize the profiles for the various gamepad inputs
/// @return void
void HolonomicDrive::Init()
{
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("HolonomicDrive"), string("Initialized?"), "true");
}

/// @brief calculate the output for the wheels on the chassis from the throttle and steer components
/// @return void
void HolonomicDrive::Run()
{
    ChassisMovement moveInfo;
    moveInfo.driveOption = ChassisOptionEnums::DriveStateType::FIELD_DRIVE;
    moveInfo.controllerType = ChassisOptionEnums::AutonControllerType::HOLONOMIC;

    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("HolonomicDrive"), string("DriveOptionBEGINNING"), moveInfo.driveOption);

    auto controller = TeleopControl::GetInstance();

    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("HolonomicDrive"), string("chassis "), m_chassis != nullptr ? string("not nullptr ") : string("nullptr"));
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("HolonomicDrive"), string("controller "), controller != nullptr ? string("not nullptr ") : string("nullptr"));

    if (controller != nullptr && m_chassis != nullptr)
    {
        moveInfo.headingOption = ChassisOptionEnums::HeadingOption::MAINTAIN;

        if (controller->IsButtonPressed(TeleopControlFunctions::RESET_POSITION) && !m_hasResetPosition)
        {
            if (m_swerve != nullptr)
            {
                m_swerve->ResetPoseToVision();
            }

            m_hasResetPosition = true;
        }
        else
        {
            m_hasResetPosition = false;
        }

        if (m_swerve != nullptr)
        {
            auto wheelbase = m_swerve->GetWheelBase();
            auto track = m_swerve->GetTrack();
        }

        // Auto alignment to grid nodes
        if (controller->IsButtonPressed(TeleopControlFunctions::DRIVE_TO_CONE_NODE))
        {
            DragonVision::GetDragonVision()->setPipeline(DragonLimelight::PIPELINE_MODE::CONE_NODE);

            moveInfo.driveOption = ChassisOptionEnums::DriveStateType::VISION_DRIVE;
            moveInfo.headingOption = ChassisOptionEnums::HeadingOption::SPECIFIED_ANGLE;

            m_inVisionDrive = true;

            frc::DriverStation::Alliance alliance = FMSData::GetInstance()->GetAllianceColor();

            if (alliance == frc::DriverStation::Alliance::kBlue)
            {
                moveInfo.yawAngle = units::angle::degree_t(180.0);
            }
            else if (alliance == frc::DriverStation::Alliance::kRed)
            {
                moveInfo.yawAngle = units::angle::degree_t(0.0);
            }
        }
        else if (controller->IsButtonPressed(TeleopControlFunctions::DRIVE_TO_CUBE_NODE))
        {
            DragonVision::GetDragonVision()->setPipeline(DragonLimelight::PIPELINE_MODE::APRIL_TAG);

            moveInfo.driveOption = ChassisOptionEnums::DriveStateType::VISION_DRIVE;
            moveInfo.headingOption = ChassisOptionEnums::HeadingOption::SPECIFIED_ANGLE;

            m_inVisionDrive = true;

            frc::DriverStation::Alliance alliance = FMSData::GetInstance()->GetAllianceColor();

            if (alliance == frc::DriverStation::Alliance::kBlue)
            {
                moveInfo.yawAngle = units::angle::degree_t(180.0);
            }
            else if (alliance == frc::DriverStation::Alliance::kRed)
            {
                moveInfo.yawAngle = units::angle::degree_t(0.0);
            }
        }
        else
        {
            DragonVision::GetDragonVision()->setPipeline(DragonLimelight::PIPELINE_MODE::OFF);

            m_inVisionDrive = false;
        }

        // add button to align with substation

        if (controller->IsButtonPressed(TeleopControlFunctions::HOLD_POSITION))
        {
            moveInfo.driveOption = ChassisOptionEnums::DriveStateType::HOLD_DRIVE;
            m_previousDriveState = moveInfo.driveOption;
        }
        if (controller->IsButtonPressed(TeleopControlFunctions::AUTO_TURN_FORWARD))
        {
            moveInfo.headingOption = ChassisOptionEnums::HeadingOption::SPECIFIED_ANGLE;
            moveInfo.yawAngle = units::angle::degree_t(0.0);
        }
        if (controller->IsButtonPressed(TeleopControlFunctions::AUTO_TURN_BACKWARD))
        {
            moveInfo.headingOption = ChassisOptionEnums::HeadingOption::SPECIFIED_ANGLE;
            moveInfo.yawAngle = units::angle::degree_t(180.0);
        }

        auto maxSpeed = m_chassis->GetMaxSpeed();
        auto maxAngSpeed = m_chassis->GetMaxAngularSpeed();

        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("HolonomicDrive"), string("maxspeed "), maxSpeed.to<double>());
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("HolonomicDrive"), string("maxangspeed "), maxAngSpeed.to<double>());

        auto forward = controller->GetAxisValue(TeleopControlFunctions::HOLONOMIC_DRIVE_FORWARD);
        auto strafe = controller->GetAxisValue(TeleopControlFunctions::HOLONOMIC_DRIVE_STRAFE);
        auto rotate = controller->GetAxisValue(TeleopControlFunctions::HOLONOMIC_DRIVE_ROTATE);

        if (controller->IsButtonPressed(TeleopControlFunctions::SLOW_MODE))
        {
            forward *= m_slowModeMultiplier;
            strafe *= m_slowModeMultiplier;
            rotate *= m_slowModeMultiplier;
        }

        if ((abs(forward) > 0.05 || abs(strafe) > 0.05 || abs(rotate) > 0.05) && !m_inVisionDrive)
        {
            moveInfo.driveOption = ChassisOptionEnums::DriveStateType::FIELD_DRIVE;
            m_previousDriveState = moveInfo.driveOption;
        }

        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("HolonomicDrive"), string("Run"), string("axis read"));

        // temporary fix for resetting odomtery by apriltags, issues with rotation
        if (FMSData::GetInstance()->GetAllianceColor() == frc::DriverStation::Alliance::kRed)
        {
            forward *= -1.0;
            strafe *= -1.0;
        }

        moveInfo.chassisSpeeds.vx = forward * maxSpeed;
        moveInfo.chassisSpeeds.vy = strafe * maxSpeed;
        moveInfo.chassisSpeeds.omega = rotate * maxAngSpeed;

        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("HolonomicDrive"), string("Vx"), moveInfo.chassisSpeeds.vx.to<double>());
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("HolonomicDrive"), string("Vy"), moveInfo.chassisSpeeds.vy.to<double>());

        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("HolonomicDrive"), string("DriveOptionEND"), moveInfo.driveOption);

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