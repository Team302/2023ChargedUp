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
#include <mechanisms/arm/ArmManualState.h>
#include <mechanisms/arm/ArmState.h>
#include <mechanisms/controllers/ControlData.h>
#include <teleopcontrol/TeleopControl.h>
#include <mechanisms/MechanismFactory.h>

ArmManualState::ArmManualState(std::string stateName,
                               int stateId,
                               ControlData *control0,
                               double target0) : ArmState(stateName, stateId, control0, target0),
                                                 m_arm(MechanismFactory::GetMechanismFactory()->GetArm()),
                                                 m_controller(TeleopControl::GetInstance())
{
}

void ArmManualState::Init()
{
}

void ArmManualState::Run()
{
    if (m_controller != nullptr && m_arm != nullptr)
    {
        auto percent = m_controller->GetAxisValue(TeleopControlFunctions::MANUAL_ROTATE);
        m_arm->GetMotor().get()->Set(percent);
    }
}