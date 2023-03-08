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

// Third Party Includes

using namespace std;

IntakeStateMgr *IntakeStateMgr::m_instance = nullptr;
IntakeStateMgr *IntakeStateMgr::GetInstance()
{
    if (IntakeStateMgr::m_instance == nullptr)
    {
        auto grabberPtr = MechanismFactory::GetMechanismFactory()->GetGrabber();
        if (grabberPtr != nullptr)
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
                                   m_prevState(INTAKE_STATE::OFF),
                                   m_coneMode(true)

{
    map<string, StateStruc> stateMap;
    stateMap[m_intakeOffXMLString] = m_offState;
    stateMap[m_intakeConeXMLString] = m_intakeConeState;
    stateMap[m_holdConeXMLString] = m_holdConeState;
    stateMap[m_dropConeXMLString] = m_dropConeState;
    stateMap[m_expelConeXMLString] = m_expelConeState;
    stateMap[m_intakeCubeXMLString] = m_intakeCubeState;
    stateMap[m_holdCubeXMLString] = m_holdCubeState;
    stateMap[m_expelCubeXMLString] = m_expelCubeState;

    Init(m_intake, stateMap);
    if (m_intake != nullptr)
    {
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
    if (m_intake != nullptr)
    {
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
    }
}

/// @brief Check for sensor input to transition
void IntakeStateMgr::CheckForSensorTransitions()
{
    if (m_intake != nullptr)
    {
        auto hasGamePiece = m_intake->IsGamePiecePresent();
        if (m_coneMode && hasGamePiece)
        {
            m_targetState = INTAKE_STATE::HOLD_CONE;
            RobotState::GetInstance()->PublishStateChange(RobotStateChanges::HoldingGamePiece, RobotStateChanges::Cone);
        }
        else if (hasGamePiece)
        {
            m_targetState = INTAKE_STATE::HOLD_CUBE;
            RobotState::GetInstance()->PublishStateChange(RobotStateChanges::HoldingGamePiece, RobotStateChanges::Cube);
        }
        else if (!hasGamePiece)
        {
            RobotState::GetInstance()->PublishStateChange(RobotStateChanges::HoldingGamePiece, RobotStateChanges::None);
        }
    }
}

/// @brief Check for gamepad input to transition
void IntakeStateMgr::CheckForGamepadTransitions()
{
    if (m_intake != nullptr)
    {
        m_currentState = static_cast<INTAKE_STATE>(GetCurrentState());
        m_targetState = m_currentState;

        auto controller = TeleopControl::GetInstance();
        if (controller != nullptr)
        {
            if (controller->IsButtonPressed(TeleopControlFunctions::OPEN))
            {
                m_targetState = m_coneMode ? INTAKE_STATE::DROP_CONE : INTAKE_STATE::EXPEL_CUBE;
            }
            else if (controller->IsButtonPressed(TeleopControlFunctions::GRAB) && (m_currentState != INTAKE_STATE::HOLD_CONE || m_currentState != HOLD_CUBE))
            {
                m_targetState = m_coneMode ? INTAKE_STATE::INTAKE_CONE : INTAKE_STATE::INTAKE_CUBE;
            }
        }
    }
}

void IntakeStateMgr::Update(RobotStateChanges::StateChange change, int value)
{
}
