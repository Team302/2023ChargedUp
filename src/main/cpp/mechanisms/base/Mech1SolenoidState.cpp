
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
#include <mechanisms/base/Mech1SolenoidState.h>
#include <mechanisms/base/Mech1Solenoid.h>
#include <utils/logging/Logger.h>

#include <teleopcontrol/TeleopControl.h>

// Third Party Includes

using namespace std;

/// @class Mech1SolenoidState
/// @brief information about the control (open loop, closed loop position, closed loop velocity, etc.) for a mechanism state
Mech1SolenoidState::Mech1SolenoidState(
    Mech1Solenoid *mechanism,
    string stateName,
    int stateId,
    MechanismTargetData::SOLENOID solState) : State(stateName, stateId),
                                              m_mechanism(mechanism),
                                              m_solenoidState(solState)
{
    if (mechanism == nullptr)
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, string("Mech1SolenoidState"), string("Mech1SolenoidState"), string("no mechanism"));
    }
}

void Mech1SolenoidState::Init()
{
}

void Mech1SolenoidState::Run()
{
    if (m_mechanism != nullptr)
    {
        switch (m_solenoidState)
        {
        case MechanismTargetData::SOLENOID::REVERSE:
            m_mechanism->ActivateSolenoid(false);
            break;

        case MechanismTargetData::SOLENOID::ON:
            m_mechanism->ActivateSolenoid(true);
            break;

        default:
            break;
        }
    }
}

void Mech1SolenoidState::Exit()
{
}

bool Mech1SolenoidState::AtTarget() const
{
    return true;
}
void Mech1SolenoidState::LogInformation() const
{
    if (m_mechanism != nullptr)
    {
        auto ntName = m_mechanism->GetNetworkTableName();
        auto id = GetStateName();
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, ntName, string("state name"), id);
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, ntName, string("Activated"), m_mechanism->IsSolenoidActivated());
    }
}
