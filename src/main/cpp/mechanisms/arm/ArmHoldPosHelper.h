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

// Team 302 Includes
#include <robotstate/RobotStateChanges.h>
#include <mechanisms/intake/IntakeStateMgr.h>

class ArmHoldPosHelper
{
public:
    ArmHoldPosHelper();
    ~ArmHoldPosHelper() = default;

    static double CalculateHoldPositionTarget(double armAngle,
                                              double extenderPosition,
                                              RobotStateChanges::GamePiece gamepieceMode,
                                              IntakeStateMgr::INTAKE_STATE intakeState);

private:
    // Hold Position function components
    static constexpr double m_cubeOffset = 0.04242107;
    static constexpr double m_cubeArmComponent = 0.000106669;
    static constexpr double m_cubeExtenderComponent = 0.000125066;
    static constexpr double m_cubeArmSquaredComponent = 0.00000803241;
    static constexpr double m_cubeExtenderSquaredComponent = 0.0000511601;

    static constexpr double m_coneOffset = 0.0291106;
    static constexpr double m_coneArmComponent = 0.00123687;
    static constexpr double m_coneExtenderComponent = -0.000724541;
    static constexpr double m_coneArmSquaredComponent = 0;
    static constexpr double m_coneExtenderSquaredComponent = 0.000116897;

    static constexpr double m_fullExtensionFTerm = 0.115;

    static constexpr double m_fTermAngleThreshold = 10.0;
    static constexpr double m_maxArmAngle = 75.0;
    static constexpr double m_fullExtensionExtenderPos = 21.0;
    static constexpr double m_fullExtensionArmAngle = 40.0;
    static constexpr double m_intakeScaling = 1.04;
};