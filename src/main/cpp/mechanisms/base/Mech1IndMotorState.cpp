
//====================================================================================================================================================
/// Copyright 2023 Lake Orion Robotics FIRST Team 302
///
/// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
/// to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
/// and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
///
/// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
///
/// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
/// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
/// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE
/// OR OTHER DEALINGS IN THE SOFTWARE.
//====================================================================================================================================================

// C++ Includes
#include <memory>
#include <string>

// FRC includes

// Team 302 includes
#include <State.h>
#include <mechanisms/base/Mech1IndMotorState.h>
#include <mechanisms/controllers/ControlData.h>
#include <mechanisms/controllers/MechanismTargetData.h>
#include <mechanisms/base/Mech1IndMotor.h>
#include <utils/logging/Logger.h>

#include <teleopcontrol/TeleopControl.h>

// Third Party Includes

using namespace std;

/// @class Mech1IndMotorState
/// @brief information about the control (open loop, closed loop position, closed loop velocity, etc.) for a mechanism state
Mech1IndMotorState::Mech1IndMotorState(
    Mech1IndMotor *mechanism,
    string stateName,
    int stateId,
    ControlData *control,
    double target) : State(stateName, stateId),
                     m_mechanism(mechanism),
                     m_control(control),
                     m_target(target),
                     m_originalTarget(target),
                     m_positionBased(false),
                     m_speedBased(false)
{
    auto ntName = string("Mech1IndMotorState");
    if (mechanism == nullptr)
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, ntName, ("Mech1IndMotorState::Mech1IndMotorState"), string("no mechanism"));
    }
    else
    {
        ntName = mechanism->GetNetworkTableName();
    }

    if (control == nullptr)
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, ntName, string("Mech1IndMotorState::Mech1IndMotorState"), string("no control data"));
    }
    else
    {
        auto mode = control->GetMode();
        switch (mode)
        {
        case ControlModes::CONTROL_TYPE::PERCENT_OUTPUT:
            m_positionBased = false;
            m_speedBased = false;
            break;

        case ControlModes::CONTROL_TYPE::VOLTAGE:
            m_positionBased = false;
            m_speedBased = false;
            break;

        case ControlModes::CONTROL_TYPE::POSITION_DEGREES:
        case ControlModes::CONTROL_TYPE::POSITION_INCH:
            m_positionBased = true;
            m_speedBased = false;
            break;

        case ControlModes::CONTROL_TYPE::VELOCITY_DEGREES:
        case ControlModes::CONTROL_TYPE::VELOCITY_INCH:
        case ControlModes::CONTROL_TYPE::VELOCITY_RPS:
            m_positionBased = false;
            m_speedBased = true;
            break;

        case ControlModes::CONTROL_TYPE::CURRENT:
            m_positionBased = false;
            m_speedBased = false;
            break;

        case ControlModes::CONTROL_TYPE::MOTION_PROFILE:
            m_positionBased = false;
            m_speedBased = false;
            break;

        case ControlModes::CONTROL_TYPE::MOTION_PROFILE_ARC:
            m_positionBased = false;
            m_speedBased = false;
            break;

        case ControlModes::CONTROL_TYPE::TRAPEZOID:
            m_positionBased = false;
            m_speedBased = false;
            break;

        default:
            m_positionBased = false;
            m_speedBased = false;
            break;
        }
    }
}

void Mech1IndMotorState::Init()
{
    if (m_mechanism != nullptr && m_control != nullptr)
    {
        m_mechanism->SetControlConstants(0, m_control);
        m_mechanism->UpdateTarget(m_target);
    }
}

void Mech1IndMotorState::Run()
{
    if (m_mechanism != nullptr)
    {
        m_mechanism->Update();
    }
}

void Mech1IndMotorState::Exit()
{
}

bool Mech1IndMotorState::AtTarget() const
{
    auto same = true;
    if (m_mechanism != nullptr)
    {
        double motorPosition = 0.0;
        if (m_control->GetMode() == ControlModes::CONTROL_TYPE::POSITION_INCH)
        {
            motorPosition = m_mechanism->GetPositionInches().to<double>();
        }
        else if (m_control->GetMode() == ControlModes::CONTROL_TYPE::POSITION_DEGREES)
        {
            motorPosition = m_mechanism->GetPositionDegrees().to<double>();
        }

        if (m_positionBased && !m_speedBased)
        {
            if (m_control->GetMode() == ControlModes::CONTROL_TYPE::POSITION_INCH)
            {
                same = (abs(m_target - motorPosition) < 0.1);
            }
            else if (m_control->GetMode() == ControlModes::CONTROL_TYPE::POSITION_DEGREES)
            {
                same = (abs(m_target - motorPosition) < 0.5);
            }
        }
        else if (!m_positionBased && m_speedBased)
        {
            same = (abs(m_target - m_mechanism->GetSpeed()) < 1.0);
        }
        else if (m_positionBased && m_speedBased)
        {
            same = ((abs(m_target - motorPosition) < 0.1) ||
                    (abs(m_target - m_mechanism->GetSpeed()) < 1.0));
        }
    }
    return same;
}

void Mech1IndMotorState::LogInformation() const
{
    if (m_mechanism != nullptr)
    {
        auto ntName = m_mechanism->GetNetworkTableName();
        auto statename = GetStateName();
        auto idStatename = string("Mech1IndMotorState") + to_string(GetStateId()) + string(" - ") + statename;
        auto idStatenameTarget = idStatename + string(" - Target");
        auto idStatenameSpeed = idStatename + string(" - Speed");

        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, ntName, idStatename, GetStateName());
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, ntName, idStatenameTarget, GetCurrentTarget());
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, ntName, idStatenameSpeed, GetRPS());
    }
}