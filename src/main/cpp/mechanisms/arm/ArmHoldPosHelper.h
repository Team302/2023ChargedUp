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
#include <mechanisms/grabber/GrabberStateMgr.h>

class ArmHoldPosHelper
{
public:
    ArmHoldPosHelper();
    ~ArmHoldPosHelper() = default;

    static double CalculateHoldPositionTarget(double armAngle,
                                              double extenderPosition,
                                              RobotStateChanges::GamePiece gamepieceMode,
                                              GrabberStateMgr::GRABBER_STATE grabberState);

private:
    // Hold Position function components
    static constexpr double m_cubeOffset = 0.0446119;
    static constexpr double m_cubeArmComponent = -0.00010589;
    static constexpr double m_cubeExtenderComponent = 0.000633812;
    static constexpr double m_cubeArmSquaredComponent = 0.00000489504;
    static constexpr double m_cubeExtenderSquaredComponent = 0.00000226623;

    static constexpr double m_coneOffset = 0.0412269;
    static constexpr double m_coneArmComponent = 0.000421601;
    static constexpr double m_coneExtenderComponent = 0.000703398;
    static constexpr double m_coneArmSquaredComponent = -0.00000267649;
    static constexpr double m_coneExtenderSquaredComponent = -0.0000138281;

    static constexpr double m_fullExtensionFTerm = 0.115;

    static constexpr double m_fTermAngleThreshold = 10.0;
    static constexpr double m_maxArmAngle = 75.0;
    static constexpr double m_fullExtensionExtenderPos = 21.0;
    static constexpr double m_fullExtensionArmAngle = 40.0;
};