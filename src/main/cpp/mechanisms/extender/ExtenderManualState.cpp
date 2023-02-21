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
#include <mechanisms/extender/ExtenderManualState.h>
#include <mechanisms/extender/ExtenderState.h>
#include <mechanisms/controllers/ControlData.h>
#include <teleopcontrol/TeleopControl.h>
#include <mechanisms/MechanismFactory.h>

ExtenderManualState::ExtenderManualState(std::string stateName,
                                         int stateId,
                                         ControlData *control0,
                                         double target0) : ExtenderState(stateName, stateId, control0, target0),
                                                           m_extender(MechanismFactory::GetMechanismFactory()->GetExtender()),
                                                           m_controller(TeleopControl::GetInstance()),
                                                           m_controlData(control0)
{
}

void ExtenderManualState::Init()
{
}

void ExtenderManualState::Run()
{
    if (m_controller != nullptr && m_extender != nullptr)
    {
        auto percent = 0.5 * m_controller->GetAxisValue(TeleopControlFunctions::MANUAL_EXTEND_RETRACT);
        m_extender->SetControlConstants(0, m_controlData);
        m_extender->UpdateTarget(percent);
        m_extender->Update();
    }
}

bool ExtenderManualState::AtTarget() const
{
    return true;
}