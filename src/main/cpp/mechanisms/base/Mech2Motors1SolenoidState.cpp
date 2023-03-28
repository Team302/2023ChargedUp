
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
#include <mechanisms/base/Mech2Motors1Solenoid.h>
#include <mechanisms/base/Mech2Motors1SolenoidState.h>
#include <mechanisms/controllers/ControlData.h>
#include <mechanisms/controllers/MechanismTargetData.h>
#include <mechanisms/base/Mech2Motors1Solenoid.h>
#include <utils/logging/Logger.h>

#include <teleopcontrol/TeleopControl.h>

// Third Party Includes

using namespace std;

/// @class Mech2Motors1SolenoidState
/// @brief information about the control (open loop, closed loop position, closed loop velocity, etc.) for a mechanism state
Mech2Motors1SolenoidState::Mech2Motors1SolenoidState(
    Mech2Motors1Solenoid *mechanism,
    string stateName,
    int stateId,
    ControlData *control,
    ControlData *control2,
    double primaryTarget,
    double secondaryTarget,
    MechanismTargetData::SOLENOID solState) : State(stateName, stateId),
                                              m_mechanism(mechanism),
                                              m_motorState(new Mech2IndMotorsState(mechanism->GetMotorMech(), stateName, stateId, control, control2, primaryTarget, secondaryTarget)),
                                              m_solenoidState(new Mech1SolenoidState(mechanism->GetSolenoidMech(), stateName, stateId, solState))

{
}

void Mech2Motors1SolenoidState::Init()
{
    string identifier("Mech2Motors1SolenoidState::Init motor state");
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("IntakeDebugging"), identifier, m_motorState != nullptr ? "not nullptr" : "nullptr");
    string identifier2("Mech2Motors1SolenoidState::Init solenoid state");
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("IntakeDebugging"), identifier2, m_solenoidState != nullptr ? "not nullptr" : "nullptr");

    if (m_motorState != nullptr)
    {
        m_motorState->Init();
    }
    if (m_solenoidState != nullptr)
    {
        m_solenoidState->Init();
    }
}

void Mech2Motors1SolenoidState::Run()
{
    string identifier("Mech2Motors1SolenoidState::Run motor state");
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("IntakeDebugging"), identifier, m_motorState != nullptr ? "not nullptr" : "nullptr");
    string identifier2("Mech2Motors1SolenoidState::Run solenoid state");
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("IntakeDebugging"), identifier2, m_solenoidState != nullptr ? "not nullptr" : "nullptr");

    if (m_motorState != nullptr)
    {
        m_motorState->Run();
    }
    if (m_solenoidState != nullptr)
    {
        m_solenoidState->Run();
    }
}

void Mech2Motors1SolenoidState::Exit()
{
    if (m_motorState != nullptr)
    {
        m_motorState->Exit();
    }
    if (m_solenoidState != nullptr)
    {
        m_solenoidState->Exit();
    }
}

bool Mech2Motors1SolenoidState::AtTarget() const
{
    auto done = false;
    if (m_motorState != nullptr)
    {
        done = m_motorState->AtTarget();
    }
    if (done && m_solenoidState != nullptr)
    {
        done = m_solenoidState->AtTarget();
    }
    return done;
}

void Mech2Motors1SolenoidState::LogInformation() const
{
    if (m_motorState != nullptr)
    {
        m_motorState->LogInformation();
    }
    if (m_solenoidState != nullptr)
    {
        m_solenoidState->LogInformation();
    }
}