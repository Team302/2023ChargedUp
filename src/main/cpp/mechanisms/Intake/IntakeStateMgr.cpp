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
#include <map>
#include <string>

// FRC includes

// Team 302 includes
#include <teleopcontrol/TeleopControl.h>
#include <auton/PrimitiveParams.h>
#include <mechanisms/MechanismFactory.h>
#include <mechanisms/base/StateMgr.h>
#include <mechanisms/StateStruc.h>
#include <mechanisms/intake/Intake.h>
#include <mechanisms/intake/IntakeState.h>
#include <mechanisms/intake/IntakeStateMgr.h>
#include <robotstate/RobotState.h>
#include <robotstate/RobotStateChanges.h>
#include <utils/logging/Logger.h>
#include <driveteamfeedback/DriverFeedback.h>

//  Third Party Includes

using namespace std;

IntakeStateMgr *IntakeStateMgr::m_instance = nullptr;
IntakeStateMgr *IntakeStateMgr::GetInstance()
{
    if (IntakeStateMgr::m_instance == nullptr)
    {
        auto intake = MechanismFactory::GetMechanismFactory()->GetIntake();
        if (intake != nullptr)
        {
            IntakeStateMgr::m_instance = new IntakeStateMgr();
        }
    }
    return IntakeStateMgr::m_instance;
}

/// @brief    initialize the state manager, parse the configuration file and create the states.
IntakeStateMgr::IntakeStateMgr() : StateMgr(),
                                   m_intake(MechanismFactory::GetMechanismFactory()->GetIntake()),
                                   m_currentState(INTAKE_STATE::OFF),
                                   m_targetState(INTAKE_STATE::OFF),
                                   m_prevState(INTAKE_STATE::OFF)

{
    map<string, StateStruc> stateMap;
    stateMap[m_intakeOffXMLString] = m_offState;
    stateMap[m_intakeXMLString] = m_intakeState;
    stateMap[m_holdXMLString] = m_holdState;
    stateMap[m_releaseXMLString] = m_releaseState;
    stateMap[m_expelXMLString] = m_expelState;

    string identifier("IntakeStateMgr::IntakeStateMgr");

    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("IntakeDebugging"), identifier, "Before Init");

    Init(m_intake, stateMap);
    if (m_intake != nullptr)
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("IntakeDebugging"), identifier, "Intake Not Null");
        m_intake->AddStateMgr(this);
    }

    RobotState::GetInstance()->RegisterForStateChanges(this, RobotStateChanges::StateChange::ArmRotateState);
}

/// @brief  Get the current Parameter parm value for the state of this mechanism
/// @param PrimitiveParams* currentParams current set of primitive parameters
/// @returns int state id - -1 indicates that there is not a state to set
int IntakeStateMgr::GetCurrentStateParam(PrimitiveParams *currentParams)
{
    return static_cast<int>(currentParams->GetIntakeState());
}

/// @brief Check if driver inputs or sensors trigger a state transition
void IntakeStateMgr::CheckForStateTransition()
{
    string identifier = string("CheckForStateTransition - current state");
    string identifier2 = string("CheckForStateTransition - target state");
    string identifier3 = string("CheckForStateTransition - have intake");

    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("IntakeDebugging"), identifier, m_targetState);
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("IntakeDebugging"), identifier3, m_intake != nullptr ? "true" : "false");

    if (m_intake != nullptr)
    {
        m_currentState = static_cast<INTAKE_STATE>(GetCurrentState());
        m_targetState = m_currentState;

        CheckForSensorTransitions();
        if (m_checkGamePadTransitions)
        {
            CheckForGamepadTransitions();
        }

        if (m_targetState != m_currentState)
        {
            SetCurrentState(m_targetState, true);
            m_prevState = m_targetState;
            RobotState::GetInstance()->PublishStateChange(RobotStateChanges::IntakeState, m_targetState);
        }
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("IntakeDebugging"), identifier, m_targetState);
    }
}

/// @brief Check for sensor input to transition
void IntakeStateMgr::CheckForSensorTransitions()
{
    if (m_intake != nullptr && m_intake->IsGamePiecePresent())
    {
        m_targetState = INTAKE_STATE::HOLD;
        RobotState::GetInstance()->PublishStateChange(RobotStateChanges::HoldingGamePiece, m_wantCone ? RobotStateChanges::Cone : RobotStateChanges::Cube);
    }
}

/// @brief Check for gamepad input to transition
void IntakeStateMgr::CheckForGamepadTransitions()
{
    if (m_intake != nullptr)
    {
        auto controller = TeleopControl::GetInstance();
        if (controller != nullptr)
        {
            if (controller->IsButtonPressed(TeleopControlFunctions::RELEASE))
            {
                m_targetState = INTAKE_STATE::RELEASE;
            }
            else if (controller->IsButtonPressed(TeleopControlFunctions::INTAKE))
            {
                auto hasGamePiece = m_intake->IsGamePiecePresent();
                m_targetState = hasGamePiece ? INTAKE_STATE::HOLD : INTAKE_STATE::INTAKE;
            }
            else if (controller->IsButtonPressed(TeleopControlFunctions::EXPEL))
            {
                m_targetState = INTAKE_STATE::EXPEL;
            }
            else if (controller->IsButtonPressed(TeleopControlFunctions::HOLD))
            {
                m_targetState = INTAKE_STATE::HOLD;
            }
            else if (m_currentState != INTAKE_STATE::HOLD)
            {
                m_targetState = INTAKE_STATE::OFF;
            }
        }
    }
}

void IntakeStateMgr::Update(RobotStateChanges::StateChange change, int value)
{
    if (change == RobotStateChanges::DesiredGamePiece)
    {
        auto gamepiece = static_cast<RobotStateChanges::GamePiece>(value);
        m_wantCube = gamepiece == RobotStateChanges::Cube;
        m_wantCone = gamepiece == RobotStateChanges::Cone;
    }
}