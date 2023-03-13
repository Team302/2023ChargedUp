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
#include <utils/FMSData.h>
#include <robotstate/RobotState.h>
#include <utils/FMSData.h>

/// DEBUGGING
#include <utils/logging/Logger.h>

VisionDrive::VisionDrive(RobotDrive *robotDrive) : RobotDrive(),
                                                   // m_visionVYPID(1.0, 0.05, 0.0), // kP, kI, kD
                                                   // m_visionVXPID(1.0, 0.05, 0.0), // kP, kI, kD
                                                   m_currentState(VISION_STATE::LOOKING_FOR_APRIL_TAG),
                                                   m_previousState(VISION_STATE::NORMAL_DRIVE),
                                                   m_robotDrive(robotDrive),
                                                   m_chassis(ChassisFactory::GetChassisFactory()->GetSwerveChassis()),
                                                   m_vision(DragonVision::GetDragonVision())
{
}

std::array<frc::SwerveModuleState, 4> VisionDrive::UpdateSwerveModuleStates(
    ChassisMovement &chassisMovement)
{
    switch (m_currentState)
    {
    case VISION_STATE::NORMAL_DRIVE:
        break;
    case VISION_STATE::LOOKING_FOR_APRIL_TAG:
        LookingForTag(chassisMovement);
        break;
    case VISION_STATE::FOUND_APRIL_TAG:
        FoundTag(chassisMovement);
        break;
    case VISION_STATE::DRIVE_TO_TARGET:
        DriveToTarget(chassisMovement);
        break;
    case VISION_STATE::ALIGN_RAW_VISION:
        AlignRawVision(chassisMovement);
        break;
    case VISION_STATE::ALIGNED:
        Aligned(chassisMovement);
        break;
    }

    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "VisionDrive", "CurrentState", m_currentState);
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "VisionDrive", "PreviousState", m_previousState);
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "VisionDrive", "VY", chassisMovement.chassisSpeeds.vy.to<double>());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "VisionDrive", "VX", chassisMovement.chassisSpeeds.vx.to<double>());

    // temporary disable driving
    // chassisMovement.chassisSpeeds.vy = units::meters_per_second_t(0.0);

    return m_robotDrive->UpdateSwerveModuleStates(chassisMovement);
}

void VisionDrive::Init(
    ChassisMovement &chassisMovement)
{
}

void VisionDrive::LookingForTag(ChassisMovement &chassisMovement)
{
    bool exit = false;

    // Entry
    if (m_currentState != m_previousState)
    {
        m_previousState = VISION_STATE::LOOKING_FOR_APRIL_TAG;
        m_vision->setPipeline(DragonLimelight::PIPELINE_MODE::APRIL_TAG);
    }

    // Cyclic
    auto targetData = DragonVision::GetDragonVision()->getTargetInfo();

    if (targetData != nullptr)
    {
        m_aprilTagID = targetData->getApriltagID();
        m_yDistanceToTag = targetData->getYdistanceToTargetRobotFrame();
        m_xDistanceToTag = targetData->getXdistanceToTargetRobotFrame();
        exit = true;
    }
    else
    {
        // LATER, add in timer for 0.2 seconds, if no april tag switch to align with raw vision
    }

    // Exit
    if (exit)
    {
        m_currentState = VISION_STATE::FOUND_APRIL_TAG;
    }
}

void VisionDrive::FoundTag(ChassisMovement &chassisMovement)
{
    bool exit = false;

    // Entry
    if (m_currentState != m_previousState)
    {
        m_previousState = VISION_STATE::FOUND_APRIL_TAG;
    }

    // Cyclic
    m_storedGridPos = chassisMovement.gridPosition;
    m_storedNodePos = chassisMovement.nodePosition;

    units::length::inch_t yOffset = units::length::inch_t(getOffsetToTarget(chassisMovement.gridPosition, chassisMovement.nodePosition, m_aprilTagID)) + m_yDistanceToTag;

    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "VisionDrive", "YOffsetFromFunc", getOffsetToTarget(chassisMovement.gridPosition, chassisMovement.nodePosition, m_aprilTagID));
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "VisionDrive", "YOffset", yOffset.to<double>());

    units::length::inch_t xOffset = m_xDistanceToTag - units::length::inch_t(m_robotFrameXDistCorrection);

    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "VisionDrive", "XOffset", xOffset.to<double>());

    m_yTargetPos = yOffset + m_chassis->GetPose().Y();
    m_xTargetPos = xOffset + m_chassis->GetPose().X() - units::length::inch_t(m_robotFrameGapToTag);

    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "VisionDrive", "YTargetPos", m_yTargetPos.to<double>());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "VisionDrive", "XTargetPos", m_xTargetPos.to<double>());

    // Only run once
    exit = true;

    // Exit
    if (exit)
    {
        m_currentState = VISION_STATE::DRIVE_TO_TARGET;
    }
}

void VisionDrive::DriveToTarget(ChassisMovement &chassisMovement)
{
    bool exit = false;

    // Entry
    if (m_currentState != m_previousState)
    {
        m_previousState = VISION_STATE::DRIVE_TO_TARGET;
    }

    // Cyclic
    units::length::inch_t yError = units::length::inch_t(0.0);
    units::length::inch_t xError = units::length::inch_t(0.0);

    units::length::inch_t getPos_y = m_chassis->GetPose().Y();

    yError = m_yTargetPos - m_chassis->GetPose().Y();
    xError = m_xTargetPos - m_chassis->GetPose().X();

    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "VisionDrive", "getPose_Y", getPos_y.to<double>());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "VisionDrive", "YError", yError.to<double>());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "VisionDrive", "XError", xError.to<double>());

    if (abs(xError.to<double>()) < m_driveXTolerance)
    {
        chassisMovement.chassisSpeeds.vy = units::velocity::meters_per_second_t(m_autoAlignKP_Y * yError.to<double>());
        // chassisMovement.chassisSpeeds.vy = units::velocity::meters_per_second_t(m_autoAlignKP_Y * yError.to<double>());
        if (yError.to<double>() < 0.0)
        {
            chassisMovement.chassisSpeeds.vy = units::velocity::meters_per_second_t(-1.0);
        }
        else
        {
            chassisMovement.chassisSpeeds.vy = units::velocity::meters_per_second_t(1.0);
        }
    }

    if (abs(xError.to<double>()) > m_autoAlignXTolerance)
    {
        chassisMovement.chassisSpeeds.vx = units::velocity::meters_per_second_t(1.0);
    }

    // once we get within threshold, switch to logic similar to below
    if ((abs(yError.to<double>()) < m_autoAlignYTolerance) && (abs(xError.to<double>()) < m_autoAlignXTolerance))
    {
        exit = true;
    }

    // Exit
    if (exit)
    {
        m_currentState = VISION_STATE::ALIGN_RAW_VISION;
    }
}

void VisionDrive::AlignRawVision(ChassisMovement &chassisMovement)
{
    bool exit = false;

    // Entry
    DragonLimelight::PIPELINE_MODE pipelineMode = DragonLimelight::APRIL_TAG;

    if (m_currentState != m_previousState)
    {
        m_previousState = VISION_STATE::ALIGN_RAW_VISION;
    }

    // Cyclic
    units::length::inch_t yError = units::length::inch_t(0.0);
    units::length::inch_t xError = units::length::inch_t(0.0);

    if (chassisMovement.nodePosition == ChassisOptionEnums::RELATIVE_POSITION::LEFT || chassisMovement.nodePosition == ChassisOptionEnums::RELATIVE_POSITION::RIGHT)
    {
        m_vision->setPipeline(DragonLimelight::PIPELINE_MODE::CONE_NODE);
        pipelineMode = DragonLimelight::PIPELINE_MODE::CONE_NODE;
    }
    else
    {
        m_vision->setPipeline(DragonLimelight::PIPELINE_MODE::APRIL_TAG);
        pipelineMode = DragonLimelight::PIPELINE_MODE::APRIL_TAG;
    }

    // get targetdata
    auto targetData = DragonVision::GetDragonVision()->getTargetInfo();

    if (targetData != nullptr)
    {
        // override yError and xError to data from pipeline
        yError = targetData->getYdistanceToTargetRobotFrame();
        xError = targetData->getXdistanceToTargetRobotFrame() - units::length::inch_t(m_robotFrameXDistCorrection);

        exit = (AtTargetY(targetData) && AtTargetX(targetData));
    }

    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "VisionDrive", "YError", yError.to<double>());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "VisionDrive", "XError", xError.to<double>());

    units::velocity::meters_per_second_t ySpeed = units::velocity::meters_per_second_t(0.0);
    units::velocity::meters_per_second_t xSpeed = units::velocity::meters_per_second_t(0.0);

    if ((targetData != nullptr) && (pipelineMode == targetData->getTargetType()))
    {
        ySpeed = units::length::meter_t(yError * m_visionKP_Y) / 1_s;

        if (abs(yError.to<double>()) < m_autoAlignYTolerance)
        {
            xSpeed = units::length::meter_t(xError * m_visionKP_X) / 1_s;
        }
    }

    if (abs(ySpeed.to<double>()) < m_minimumSpeed)
    {
        if (ySpeed.to<double>() < 0.0)
        {
            ySpeed = units::velocity::meters_per_second_t(m_minimumSpeed * -1.0);
        }
        else
        {
            ySpeed = units::velocity::meters_per_second_t(m_minimumSpeed);
        }
    }

    // xSpeed = units::velocity::meters_per_second_t(0);
    // ySpeed = units::velocity::meters_per_second_t(0);
    chassisMovement.chassisSpeeds.vy = ySpeed;
    chassisMovement.chassisSpeeds.vx = xSpeed;

    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "VisionDrive", "YSpeed", ySpeed.to<double>());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "VisionDrive", "XSpeed", xSpeed.to<double>());

    // Exit
    if (exit)
    {
        m_currentState = VISION_STATE::ALIGNED;
    }
}

void VisionDrive::Aligned(ChassisMovement &chassisMovement)
{
    bool exit = false;

    // Entry
    if (m_currentState != m_previousState)
    {
    }

    // Cyclic

    // set arm and extender state to automatically score

    chassisMovement.chassisSpeeds.vy = units::velocity::meters_per_second_t(0.0);
    chassisMovement.chassisSpeeds.vx = units::velocity::meters_per_second_t(0.0);

    // Exit
    if (exit)
    {
    }
}

bool VisionDrive::AtTargetX(std::shared_ptr<DragonVisionTarget> targetData)
{
    if (targetData != nullptr)
    {
        units::length::inch_t xError = targetData->getXdistanceToTargetRobotFrame();

        units::angle::degree_t verticalAngle = targetData->getVerticalAngleToTarget();

        // vertical angle is positive, so we are looking at high cone
        if (verticalAngle.to<double>() > 0.0)
        {
            if ((abs(xError.to<double>()) - m_highConeDistance) < m_tolerance)
            {
                return true;
            }
        }
        else // vert angle is negative, so we're looking at low cone
        {
            if ((abs(xError.to<double>()) - m_lowConeDistance) < m_tolerance)
            {
                return true;
            }
        }
    }
    return false;
}

bool VisionDrive::AtTargetY(std::shared_ptr<DragonVisionTarget> targetData)
{
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
double VisionDrive::getOffsetToTarget(ChassisOptionEnums::RELATIVE_POSITION targetGrid, ChassisOptionEnums::RELATIVE_POSITION targetNode, uint8_t AprilTagId)
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

        return (targetPositionIndex - currentPositionIndex) * 22;
    }

    return 0;
}

void VisionDrive::ResetVisionDrive()
{
    m_previousState = m_currentState;
    m_currentState = VISION_STATE::LOOKING_FOR_APRIL_TAG;
}