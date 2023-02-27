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

#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Pose2d.h>

// Team302 Includes
#include <chassis/swerve/driveStates/VisionDrive.h>
#include <chassis/ChassisFactory.h>
#include <DragonVision/DragonVision.h>

/// DEBUGGING
#include <utils/logging/Logger.h>

VisionDrive::VisionDrive(RobotDrive *robotDrive) : RobotDrive(),
                                                   m_robotDrive(robotDrive)
{
}

std::array<frc::SwerveModuleState, 4> VisionDrive::UpdateSwerveModuleStates(
    ChassisMovement &chassisMovement)
{
    if (DragonVision::GetDragonVision()->getTargetInfo() != nullptr)
    {
        auto targetData = DragonVision::GetDragonVision()->getTargetInfo();
        double xDistance = targetData->getXdistanceToTargetRobotFrame().to<double>();
        double yDistance = -1.0 * targetData->getYdistanceToTargetRobotFrame().to<double>();
        double horizontalangle = targetData->getHorizontalAngleToTarget().to<double>();

        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "VisionDrive", "YDistance", yDistance);
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "VisionDrive", "XDistance", xDistance);
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "VisionDrive", "HorizontalAngle", horizontalangle);

        // update chassis speeds or create new chassis speeds to move based on horizontal and depth offset given by mr muscats code
        // units::velocity::meters_per_second_t xSpeed = (m_xOffset + xDistance) * m_kP / 1_s;
        units::velocity::meters_per_second_t ySpeed = units::length::inch_t(m_yOffset.to<double>() + yDistance) * m_kP / 1_s;
        // units::angular_velocity::degrees_per_second_t omegaSpeed = units::angle::degree_t(horizontalangle * m_kAngleP) / 1_s;

        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "VisionDrive", "YSpeed", ySpeed.to<double>());
        // Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "VisionDrive", "OmegaSpeed", omegaSpeed.to<double>());

        // chassisMovement.chassisSpeeds.vx = xSpeed;
        chassisMovement.chassisSpeeds.vy = ySpeed;
        // chassisMovement.chassisSpeeds.omega = omegaSpeed;

        return m_robotDrive->UpdateSwerveModuleStates(chassisMovement);
    }
}

void VisionDrive::Init(
    ChassisMovement &chassisMovement)
{
}

void VisionDrive::UpdateOffsets(units::length::inch_t xOffset, units::length::inch_t yOffset)
{
    m_xOffset = xOffset;
    m_yOffset = yOffset;
}