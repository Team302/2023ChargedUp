
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
#include <mechanisms/base/Mech2SolenoidsState.h>
#include <mechanisms/base/Mech2Solenoids.h>
#include <utils/logging/Logger.h>

#include <teleopcontrol/TeleopControl.h>

// Third Party Includes

using namespace std;

/// @class Mech2SolenoidsState
/// @brief information about the control (open loop, closed loop position, closed loop velocity, etc.) for a mechanism state
Mech2SolenoidsState::Mech2SolenoidsState(
    Mech2Solenoids *mechanism,
    string stateName,
    int stateId,
    MechanismTargetData::SOLENOID solState,
    MechanismTargetData::SOLENOID solState2) : Mech1SolenoidState(mechanism, stateName, stateId, solState),
                                               m_mechanism(mechanism)
{
}

void Mech2SolenoidsState::Run()
{
    if (m_mechanism != nullptr)
    {
        switch (m_solenoidState2)
        {
        case MechanismTargetData::SOLENOID::REVERSE:
            m_mechanism->ActivateSolenoid2(false);
            break;

        case MechanismTargetData::SOLENOID::ON:
            m_mechanism->ActivateSolenoid2(true);
            break;

        default:
            break;
        }
    }
}

void Mech2SolenoidsState::LogInformation() const
{
    Mech1SolenoidState::LogInformation();

    if (m_mechanism != nullptr)
    {
        auto ntName = m_mechanism->GetNetworkTableName();
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, ntName, string("Activated2"), m_mechanism->IsSolenoid2Activated());
    }
}
