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
#include <chassis/swerve/driveStates/FieldDrive.h>
#include <chassis/ChassisFactory.h>

/// DEBUGGING
#include <utils/logging/Logger.h>

FieldDrive::FieldDrive(RobotDrive *robotDrive) : RobotDrive(), m_robotDrive(robotDrive)
{
}

std::array<frc::SwerveModuleState, 4> FieldDrive::UpdateSwerveModuleStates(ChassisMovement &chassisMovement)
{
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "FieldDrive", "VxBEFORE", chassisMovement.chassisSpeeds.vx.to<double>());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "FieldDrive", "VyBEFORE", chassisMovement.chassisSpeeds.vy.to<double>());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "FieldDrive", "OmegaBEFORE", chassisMovement.chassisSpeeds.omega.to<double>());

    frc::ChassisSpeeds fieldRelativeSpeeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(chassisMovement.chassisSpeeds.vx,
                                                                                         chassisMovement.chassisSpeeds.vy,
                                                                                         chassisMovement.chassisSpeeds.omega,
                                                                                         ChassisFactory::GetChassisFactory()->GetSwerveChassis()->GetPose().Rotation());

    chassisMovement.chassisSpeeds = fieldRelativeSpeeds;
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "FieldDrive", "Odometry Rotation", ChassisFactory::GetChassisFactory()->GetSwerveChassis()->GetPose().Rotation().Degrees().to<double>());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "FieldDrive", "Pigeon Rotation", ChassisFactory::GetChassisFactory()->GetSwerveChassis()->GetYaw().to<double>());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "FieldDrive", "VxAFTER", chassisMovement.chassisSpeeds.vx.to<double>());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "FieldDrive", "VyAFTER", chassisMovement.chassisSpeeds.vy.to<double>());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "FieldDrive", "OmegaAFTER", chassisMovement.chassisSpeeds.omega.to<double>());
    return m_robotDrive->UpdateSwerveModuleStates(chassisMovement);
}

void FieldDrive::Init(ChassisMovement &chassisMovement)
{
}
