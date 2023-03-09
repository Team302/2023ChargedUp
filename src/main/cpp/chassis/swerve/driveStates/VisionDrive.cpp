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
    auto targetData = DragonVision::GetDragonVision()->getTargetInfo();

    if (targetData != nullptr)
    {
        units::length::inch_t yDistance = targetData->getYdistanceToTargetRobotFrame();
        units::length::inch_t xDistance = targetData->getXdistanceToTargetRobotFrame();

        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "VisionDrive", "YDistance", yDistance.to<double>());
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "VisionDrive", "XDistance", xDistance.to<double>());

        if (!AtTargetY())
        {
            chassisMovement.chassisSpeeds.vy = units::velocity::meters_per_second_t(m_kP_Y * yDistance.to<double>() * -1.0); // negating for robot drive
        }
        else
        {
            chassisMovement.chassisSpeeds.vy = units::velocity::meters_per_second_t(0.0);
        }

        // chassisMovement.chassisSpeeds.vx = units::velocity::meters_per_second_t(m_kP_X * xDistance.to<double>());

        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "VisionDrive", "VY", chassisMovement.chassisSpeeds.vy.to<double>());
    }

    return m_robotDrive->UpdateSwerveModuleStates(chassisMovement);
}

void VisionDrive::Init(
    ChassisMovement &chassisMovement)
{
}

bool VisionDrive::AtTargetX()
{ /*
     if (DragonVision::GetDragonVision()->getTargetInfo() != nullptr)
     {
         auto targetData = DragonVision::GetDragonVision()->getTargetInfo();
         units::angle::degree_t verticalAngle = targetData->getVerticalAngleToTarget();

         if(targetData->getTargetType() == DragonLimelight::PIPELINE_MODE::RETRO_REFLECTIVE)
         {

         }

         if (verticalAngle.to<double>() > 0.0) // looking at upper cone node, need to use a farther distance for at target
         {
             if (abs(verticalAngle.to<double>() - m_highConeDistance) < m_tolerance)
             {
                 return true;
             }
         }
         else if (verticalAngle.to<double>() < 0.0) // looking at lower cone node, need to use closer distance for at target
         {
             if (abs(verticalAngle.to<double>() - m_lowConeDistance) < m_tolerance)
             {
                 return true;
             }
         }
         else
         {
             return false;
         }
     }*/
    return false;
}

bool VisionDrive::AtTargetY()
{
    auto targetData = DragonVision::GetDragonVision()->getTargetInfo();

    if (targetData != nullptr)
    {
        units::length::inch_t yError = targetData->getYdistanceToTargetRobotFrame();

        if (abs(yError.to<double>()) < m_tolerance)
        {
            return true;
        }
    }
    return false;
}

double VisionDrive::getOffsetToTarget(RELATIVE_POSITION grid, RELATIVE_POSITION node, uint32_t AprilTagId)
{
    return 0;
}