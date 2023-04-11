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

// C++
#include <string>

// FRC Includes

// 302 Includes
#include <auton/drivePrimitives/VisionDrivePrimitive.h>
#include <chassis/ChassisMovement.h>
#include <chassis/ChassisOptionEnums.h>
#include <chassis/ChassisFactory.h>

/// DEBUGGING
#include <utils/logging/Logger.h>

VisionDrivePrimitive::VisionDrivePrimitive() : m_chassis(ChassisFactory::GetChassisFactory()->GetSwerveChassis()),
                                               m_headingOption(ChassisOptionEnums::HeadingOption::MAINTAIN),
                                               m_ntName("VisionDrivePrimitive")
{
}

void VisionDrivePrimitive::Init(PrimitiveParams *params)
{
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, m_ntName, "ArrivedAtInit", true);
    m_alignmentMethod = params->GetAlignmentMethod();
    m_pipelineMode = params->GetPipelineMode();
    m_visionAlignmentXoffset_in = params->GetVisionAlignmentXoffset_in();

    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, m_ntName, "m_alignmentMethod", m_alignmentMethod);
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, m_ntName, "m_pipelineMode", m_pipelineMode);
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, m_ntName, "m_visionAlignmentXoffset_in", m_visionAlignmentXoffset_in);

    m_dragonVision = DragonVision::GetDragonVision();
    if (m_dragonVision != nullptr)
        m_dragonVision->setPipeline(m_pipelineMode);

    if (m_chassis != nullptr)
    {
        m_visionDrive = dynamic_cast<VisionDrive *>(m_chassis->GetSpecifiedDriveState(ChassisOptionEnums::DriveStateType::VISION_DRIVE));
        m_visionDrive->ResetVisionDrive();
        m_visionDrive->setVisionPipeline(m_pipelineMode);
        m_visionDrive->setAlignmentMethod(m_alignmentMethod);
        m_visionDrive->setVisionAlignmentXoffset_in(m_visionAlignmentXoffset_in);
        m_visionDrive->setInAutonMode();
    }
}

void VisionDrivePrimitive::Run()
{
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, m_ntName, "ArrivedAtRun", true);

    if (m_chassis != nullptr)
    {
        ChassisMovement moveInfo;
        moveInfo.driveOption = ChassisOptionEnums::DriveStateType::VISION_DRIVE;

        if (m_alignmentMethod == VisionDrive::ALIGNMENT_METHOD::ROTATE)
            moveInfo.headingOption = ChassisOptionEnums::HeadingOption::IGNORE;
        else
            moveInfo.headingOption = ChassisOptionEnums::HeadingOption::MAINTAIN;

        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, m_ntName, "driveOption", moveInfo.driveOption);
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, m_ntName, "headingOption", moveInfo.headingOption);

        m_chassis->Drive(moveInfo);
    }
}

bool VisionDrivePrimitive::IsDone()
{
    bool done = m_visionDrive->isAligned();

    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, m_ntName, "ArrivedAtDone", done);

    return done;
}
