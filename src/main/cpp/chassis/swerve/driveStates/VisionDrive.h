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

// FRC Includes
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/ChassisSpeeds.h>

// Team302 Includes
#include <chassis/swerve/driveStates/RobotDrive.h>
#include <DragonVision/DragonVision.h>

class VisionDrive : public RobotDrive
{
public:
    VisionDrive(RobotDrive *robotDrive);

    std::array<frc::SwerveModuleState, 4> UpdateSwerveModuleStates(
        ChassisMovement &chassisMovement) override;

    void Init(
        ChassisMovement &chassisMovement) override;

    bool AtTargetX();
    bool AtTargetY();

    void ResetVisionDrive();

private:
    RobotDrive *m_robotDrive;

    double m_kP_X = 0.1;
    double m_kP_Y = 0.075;

    const double m_autoAlignKP = 0.075;
    const double m_visionKP = 0.1;

    const double m_tolerance = 1.0;          // tolerance in inches
    const double m_autoAlignTolerance = 5.0; // tolerance in inches

    const double m_robotFrameXDistCorrection = 30.0;

    bool m_wantReset = false;
    bool m_inRawVisionMode = false;

    units::length::inch_t m_autoAlignYPos = units::length::inch_t(0.0);
    units::length::inch_t m_autoAlignXPos = units::length::inch_t(0.0);

    ChassisOptionEnums::RELATIVE_POSITION m_storedGridPos = ChassisOptionEnums::RELATIVE_POSITION::CENTER;
    ChassisOptionEnums::RELATIVE_POSITION m_storedNodePos = ChassisOptionEnums::RELATIVE_POSITION::CENTER;

    SwerveChassis *m_chassis;

    DragonVision *m_vision;

    double getOffsetToTarget(ChassisOptionEnums::RELATIVE_POSITION targetGrid, ChassisOptionEnums::RELATIVE_POSITION targetNode, uint8_t AprilTagId);
};