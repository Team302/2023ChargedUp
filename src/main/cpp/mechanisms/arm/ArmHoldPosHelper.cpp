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

// Team 302 Includes
#include <mechanisms/arm/ArmHoldPosHelper.h>
#include <mechanisms/intake/IntakeStateMgr.h>

/// DEBUGGING
#include <utils/logging/Logger.h>

ArmHoldPosHelper::ArmHoldPosHelper()
{
}

double ArmHoldPosHelper::CalculateHoldPositionTarget(double armAngle,
                                                     double extenderPos,
                                                     RobotStateChanges::GamePiece gamepieceMode,
                                                     IntakeStateMgr::INTAKE_STATE intakeState)
{
    if (armAngle > m_fTermAngleThreshold && armAngle < m_maxArmAngle)
    {
        if (extenderPos > m_fullExtensionExtenderPos && armAngle > m_fullExtensionArmAngle)
        {
            // specific f term for outlier position
            return m_fullExtensionFTerm;
        }
        else if (gamepieceMode == RobotStateChanges::GamePiece::Cone && intakeState == IntakeStateMgr::INTAKE_STATE::HOLD)
        {
            // f term function for cone
            return m_intakeScaling * (m_coneOffset + m_coneArmComponent * armAngle + m_coneExtenderComponent * extenderPos + m_coneArmSquaredComponent * pow(armAngle, 2) + m_coneExtenderSquaredComponent * pow(extenderPos, 2));
        }
        else
        {
            // f term function for cube
            return m_intakeScaling * (m_cubeOffset + m_cubeArmComponent * armAngle + m_cubeExtenderComponent * extenderPos + m_cubeArmSquaredComponent * pow(armAngle, 2) + m_cubeExtenderSquaredComponent * pow(extenderPos, 2));
        }
    }
    return 0.0;
}