
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

#pragma once

// C++ Includes
#include <string>
#include <vector>

// FRC includes

// Team 302 includes
#include <auton/PrimitiveEnums.h>
#include <chassis/ChassisOptionEnums.h>
#include <mechanisms/arm/ArmStateMgr.h>
#include <mechanisms/extender/ExtenderStateMgr.h>
#include <mechanisms/intake/IntakeStateMgr.h>
#include <dragonVision/DragonLimelight.h>
#include <chassis/swerve/driveStates/VisionDrive.h>
// @ADDMECH include for your mechanism

#include <chassis/IChassis.h>

// Third Party Includes

class PrimitiveParams
{
public:
    PrimitiveParams(
        PRIMITIVE_IDENTIFIER id,
        float time,
        float distance,
        float xLoc,
        float yLoc,
        ChassisOptionEnums::HeadingOption headingOption,
        float heading,
        float startDriveSpeed,
        float endDriveSpeed,
        std::string pathName,
        // @ADDMECH add parameter for your mechanism state
        ArmStateMgr::ARM_STATE armState,
        ExtenderStateMgr::EXTENDER_STATE extenderState,
        IntakeStateMgr::INTAKE_STATE intakeState,

        // VisionDrive settings start
        DragonLimelight::PIPELINE_MODE pipelineMode
        // VisionDrive settings end

    ); // Constructor. Takes in all parameters

    PrimitiveParams() = delete;
    virtual ~PrimitiveParams() = default; // Destructor

    // Some getters
    PRIMITIVE_IDENTIFIER GetID() const { return m_id; };
    float GetTime() const { return m_time; };
    float GetDistance() const { return m_distance; };
    float GetXLocation() const { return m_xLoc; };
    float GetYLocation() const { return m_yLoc; };
    ChassisOptionEnums::HeadingOption GetHeadingOption() const { return m_headingOption; };
    float GetHeading() const { return m_heading; };
    float GetDriveSpeed() const { return m_startDriveSpeed; };
    float GetEndDriveSpeed() const { return m_endDriveSpeed; };
    std::string GetPathName() const { return m_pathName; };
    ArmStateMgr::ARM_STATE GetArmState() const { return m_armState; };
    ExtenderStateMgr::EXTENDER_STATE GetExtenderState() const { return m_extenderState; };
    IntakeStateMgr::INTAKE_STATE GetIntakeState() const { return m_intakeState; }
    DragonLimelight::PIPELINE_MODE GetPipelineMode() const { return m_pipelineMode; }

    // @ADDMECH Add methods to get the state mgr for mechanism

    // Setters
    void
    SetDistance(float distance)
    {
        m_distance = distance;
    }
    void SetPathName(std::string path) { m_pathName = path; }

private:
    // Primitive Parameters
    PRIMITIVE_IDENTIFIER m_id; // Primitive ID
    float m_time;
    float m_distance;
    float m_xLoc;
    float m_yLoc;
    ChassisOptionEnums::HeadingOption m_headingOption;
    float m_heading;
    float m_startDriveSpeed;
    float m_endDriveSpeed;
    std::string m_pathName;
    // @ADDMECH add attribute for your mechanism state
    ArmStateMgr::ARM_STATE m_armState;
    ExtenderStateMgr::EXTENDER_STATE m_extenderState;
    IntakeStateMgr::INTAKE_STATE m_intakeState;
    DragonLimelight::PIPELINE_MODE m_pipelineMode;
};

typedef std::vector<PrimitiveParams *> PrimitiveParamsVector;
