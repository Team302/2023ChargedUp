
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
#include <mechanisms/base/Mech1IndMotor1SolenoidState.h>
#include <mechanisms/controllers/ControlData.h>
#include <mechanisms/controllers/MechanismTargetData.h>
#include <mechanisms/base/Mech1IndMotor1Solenoid.h>
#include <utils/logging/Logger.h>

#include <teleopcontrol/TeleopControl.h>

// Third Party Includes

using namespace std;

/// @class Mech1IndMotor1SolenoidState
/// @brief information about the control (open loop, closed loop position, closed loop velocity, etc.) for a mechanism state
Mech1IndMotor1SolenoidState::Mech1IndMotor1SolenoidState(
    Mech1IndMotor1Solenoid *mechanism,
    string stateName,
    int stateId,
    ControlData *control,
    double target,
    MechanismTargetData::SOLENOID solState

    ) : State(stateName, stateId),
        m_mechanism(mechanism),
        m_motorState(make_shared<Mech1IndMotorState>(mechanism->Get1IndMotorMech(), stateName, stateId, control, target)),
        m_solenoidState(make_shared<Mech1SolenoidState>(mechanism->GetSolenoidMech(), stateName, stateId, solState))
{
    if (control == nullptr)
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, mechanism->GetNetworkTableName(), ("Mech1IndMotor1SolenoidState::Mech1IndMotor1SolenoidState"), string("no control data"));
    }

    if (mechanism == nullptr)
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, string("Bad Pointer"), string("Mech1IndMotor1SolenoidState::Mech1IndMotor1SolenoidState"), string("no mechanism"));
    }
}

void Mech1IndMotor1SolenoidState::Init()
{
    m_motorState.get()->Init();
    m_solenoidState.get()->Init();
}

void Mech1IndMotor1SolenoidState::Run()
{
    m_motorState.get()->Run();
    m_solenoidState.get()->Run();
}

void Mech1IndMotor1SolenoidState::Exit()
{
}

bool Mech1IndMotor1SolenoidState::AtTarget() const
{
    return m_motorState.get()->AtTarget();
}

double Mech1IndMotor1SolenoidState::GetTarget() const
{
    return m_motorState.get()->GetCurrentTarget();
}

double Mech1IndMotor1SolenoidState::GetRPS() const
{
    return m_motorState.get()->GetRPS();
}

void Mech1IndMotor1SolenoidState::LogInformation() const
{
    m_motorState.get()->LogInformation();
    m_solenoidState.get()->LogInformation();
}