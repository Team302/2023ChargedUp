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
#include <utils/FMSData.h>

/// DEBUGGING
#include <utils/logging/Logger.h>
#include <utils/FMSData.h>

VisionDrive::VisionDrive(RobotDrive *robotDrive) : RobotDrive(),
                                                   m_robotDrive(robotDrive),
                                                   m_chassis(ChassisFactory::GetChassisFactory()->GetSwerveChassis())
{
}

std::array<frc::SwerveModuleState, 4> VisionDrive::UpdateSwerveModuleStates(
    ChassisMovement &chassisMovement)
{
    auto targetData = DragonVision::GetDragonVision()->getTargetInfo();

    if (targetData != nullptr)
    {

        // store offsets from function
        if (m_wantReset)
        {
            m_autoAlignYPos = units::length::inch_t(getOffsetToTarget(chassisMovement.gridPosition, chassisMovement.nodePosition, targetData->getApriltagID()));

            frc::DriverStation::Alliance alliance = FMSData::GetInstance()->GetAllianceColor();

            if (alliance == frc::DriverStation::Alliance::kBlue)
            {
                m_autoAlignXPos = m_chassis->GetPose().X() - targetData->getXdistanceToTargetRobotFrame();
            }
            else
            {
                m_autoAlignXPos = m_chassis->GetPose().X() + targetData->getXdistanceToTargetRobotFrame();
            }
        }

        units::length::inch_t yError = m_autoAlignYPos - m_chassis->GetPose().Y();
        units::length::inch_t xError = m_autoAlignXPos - m_chassis->GetPose().X();

        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "VisionDrive", "YError", yError.to<double>());
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "VisionDrive", "XError", xError.to<double>());

        // once we get within threshold, switch to logic similar to below
        if (abs(yError.to<double>()) < m_autoAlignTolerance && abs(xError.to<double>()) < m_autoAlignTolerance)
        {
            // switch pipeline to Cone_Node or Cube_Node
            // override yError and xError to data from pipeline
        }

        if (!AtTargetY())
        {
            chassisMovement.chassisSpeeds.vy = units::velocity::meters_per_second_t(m_kP_Y * yError.to<double>() * -1.0); // negating for robot drive
        }
        else
        {
            chassisMovement.chassisSpeeds.vy = units::velocity::meters_per_second_t(0.0);
        }

        // chassisMovement.chassisSpeeds.vx = units::velocity::meters_per_second_t(m_kP_X * xError.to<double>());

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

/// @brief Assumes that the function is only called with a valid APril tag ID
/// @param grid
/// @param node
/// @param AprilTagId
/// @return The y offset (in inches) to the target in robot coords
double VisionDrive::getOffsetToTarget(RELATIVE_POSITION targetGrid, RELATIVE_POSITION targetNode, uint8_t AprilTagId)
{
    //         TAG      ID    Index
    // on Red   |        |      |
    //         Left      1      2
    //         Center    2      5
    //         Right     3      8
    // on Blue  |        |      |
    //         Left      6      2
    //         Center    7      5
    //         Right     8      8

    frc::DriverStation::Alliance alliance = FMSData::GetInstance()->GetAllianceColor();

    int32_t NormalizedAprilTagId = AprilTagId;
    if (alliance == frc::DriverStation::Alliance::kBlue)
    {
        NormalizedAprilTagId -= 5;
    }
    else if (alliance == frc::DriverStation::Alliance::kRed)
    {
    }

    if ((NormalizedAprilTagId >= 1) && (NormalizedAprilTagId <= 3))
    {
        uint8_t currentPositionIndex = 0; // Cone node closest to Human player is index 0
        currentPositionIndex = 2 + (NormalizedAprilTagId - 1) * 3;

        uint8_t targetPositionIndex = 0; // Cone node closest to Human player is index 0
        targetPositionIndex = 2 + (targetGrid - 1) * 3;
        targetPositionIndex += targetNode - 2;

        return (currentPositionIndex - targetPositionIndex) * 22;
    }

    return 0;
}

void VisionDrive::ResetVisionDrive()
{
    m_wantReset = true;
}