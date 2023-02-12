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
//==============================================================
// This file is auto generated by FRCrobotCodeGen302.exe Version $CODE_GENERATOR_VERSION$
// Changes to this file may cause incorrect behavior and will be lost when
// the code is regenerated, unless the changes are delimited by:
//  //========= Hand modified code start section x ========
//                    Your hand written code goes here
//	//========= Hand modified code end section x ========
//==============================================================

#include <map>

// FRC includes

// Team 302 includes
#include <teleopcontrol/TeleopControl.h>
#include <auton/PrimitiveParams.h>
#include <mechanisms/MechanismFactory.h>
#include <mechanisms/base/StateMgr.h>
#include <mechanisms/StateStruc.h>
#include <mechanisms/extender/extender.h>
#include <mechanisms/extender/extenderState.h>
#include <mechanisms/extender/extenderStateMgr.h>
#include <robotstate/RobotState.h>
#include <robotstate/RobotStateChanges.h>
#include <utils/logging/Logger.h>

// Third Party Includes

using namespace std;

ExtenderStateMgr *ExtenderStateMgr::m_instance = nullptr;
ExtenderStateMgr *ExtenderStateMgr::GetInstance()
{
    if (ExtenderStateMgr::m_instance == nullptr)
    {
        auto extenderPtr = MechanismFactory::GetMechanismFactory()->GetExtender();
        if (extenderPtr != nullptr)
        {
            ExtenderStateMgr::m_instance = new ExtenderStateMgr();
        }
    }
    return ExtenderStateMgr::m_instance;
}

/// @brief    initialize the state manager, parse the configuration file and create the states.
ExtenderStateMgr::ExtenderStateMgr() : StateMgr(),
                                       IRobotStateChangeSubscriber(),
                                       m_extender(MechanismFactory::GetMechanismFactory()->GetExtender()),
                                       m_prevState(EXTENDER_STATE::STARTING_POSITION_EXTEND),
                                       m_currentState(EXTENDER_STATE::STARTING_POSITION_EXTEND),
                                       m_targetState(EXTENDER_STATE::STARTING_POSITION_EXTEND),
                                       m_gamepieceMode(RobotStateChanges::None),
                                       m_armState(ArmStateMgr::ARM_STATE::HOLD_POSITION_ROTATE),
                                       m_extendedPosition(84320.3176) // 22.25 inches in counts for extender
{
    map<string, StateStruc> stateMap;
    stateMap["HOLD_POSITION_EXTEND"] = m_hold_position_extendState;
    stateMap["MANUAL_EXTEND_RETRACT"] = m_manual_extend_retractState;
    stateMap["CUBE_BACKROW_EXTEND"] = m_cube_backrow_extendState;
    stateMap["CONE_BACKROW_EXTEND"] = m_cone_backrow_extendState;
    stateMap["CUBE_MIDROW_EXTEND"] = m_cube_midrow_extendState;
    stateMap["CONE_MIDROW_EXTEND"] = m_cone_midrow_extendState;
    stateMap["HUMAN_PLAYER_STATION_EXTEND"] = m_human_player_station_extendState;
    stateMap["STARTING_POSITION_EXTEND"] = m_starting_position_extendState;
    stateMap["FLOOR_EXTEND"] = m_floor_extendState;

    Init(m_extender, stateMap);
    if (m_extender != nullptr)
    {
        m_extender->AddStateMgr(this);
    }

    if (m_extender != nullptr && MechanismFactory::GetMechanismFactory()->GetArm() != nullptr)
    {
        m_armToExtenderMap[ArmStateMgr::ARM_STATE::FLOOR_POSITION_ROTATE] = EXTENDER_STATE::FLOOR_EXTEND;
        m_armToExtenderMap[ArmStateMgr::ARM_STATE::HUMAN_PLAYER_STATION_ROTATE] = EXTENDER_STATE::HUMAN_PLAYER_STATION_EXTEND;
        m_armToExtenderMap[ArmStateMgr::ARM_STATE::STARTING_POSITION_ROTATE] = EXTENDER_STATE::STARTING_POSITION_EXTEND;
        m_armToExtenderMap[ArmStateMgr::ARM_STATE::CONE_BACKROW_ROTATE] = EXTENDER_STATE::CONE_BACKROW_EXTEND;
        m_armToExtenderMap[ArmStateMgr::ARM_STATE::CUBE_BACKROW_ROTATE] = EXTENDER_STATE::CUBE_BACKROW_EXTEND;
        m_armToExtenderMap[ArmStateMgr::ARM_STATE::CONE_MIDROW_ROTATE] = EXTENDER_STATE::CONE_MIDROW_EXTEND;
        m_armToExtenderMap[ArmStateMgr::ARM_STATE::CUBE_MIDROW_ROTATE] = EXTENDER_STATE::CUBE_MIDROW_EXTEND;
    }

    RobotState::GetInstance()->RegisterForStateChanges(this, RobotStateChanges::StateChange::ArmRotateState);
    RobotState::GetInstance()->RegisterForStateChanges(this, RobotStateChanges::StateChange::DesiredGamePiece);
}

/// @brief  Get the current Parameter parm value for the state of this mechanism
/// @param PrimitiveParams* currentParams current set of primitive parameters
/// @returns int state id - -1 indicates that there is not a state to settargetState
int ExtenderStateMgr::GetCurrentStateParam(
    PrimitiveParams *currentParams)
{
    // normally get the state from primitive params
    return StateMgr::GetCurrentStateParam(currentParams);
}

/// @brief Check sensors to determine target state
void ExtenderStateMgr::CheckForGamepadTransitions()
{
    if (m_extender != nullptr)
    {
        m_currentState = static_cast<EXTENDER_STATE>(GetCurrentState());
        m_targetState = m_currentState;

        auto controller = TeleopControl::GetInstance();
        if (controller != nullptr)
        {
            if (abs(controller->GetAxisValue(TeleopControlFunctions::MANUAL_EXTEND_RETRACT)) > 0.1)
            {
                m_targetState = EXTENDER_STATE::MANUAL_EXTEND_RETRACT;
                m_prevState = m_targetState;
            }
            else if (!m_goToStartingConfig)
            {
                if (controller->IsButtonPressed(TeleopControlFunctions::STARTING_POSITION))
                {
                    m_targetState = EXTENDER_STATE::STARTING_POSITION_EXTEND;
                }
                else if (controller->IsButtonPressed(TeleopControlFunctions::HUMAN_PLAYER_STATION))
                {
                    m_targetState = EXTENDER_STATE::HUMAN_PLAYER_STATION_EXTEND;
                }
                else if (controller->IsButtonPressed(TeleopControlFunctions::FLOOR_POSITION))
                {
                    m_targetState = EXTENDER_STATE::FLOOR_EXTEND;
                }
                else if (m_gamepieceMode != RobotStateChanges::Cube) // if we want cone or the gamepiece mode hasn't been updated
                {
                    CheckForConeGamepadTransitions(controller);
                }
                else if (m_gamepieceMode == RobotStateChanges::Cube)
                {
                    CheckForCubeGamepadTransitions(controller);
                }
            }
            else
            {
                m_targetState = EXTENDER_STATE::STARTING_POSITION_EXTEND;
            }

            /// DEBUGGING, all of these functions are needed for testing
            /*else if (controller->IsButtonPressed(TeleopControlFunctions::CUBE_BACKROW_EXTEND))
            {
                m_targetState = EXTENDER_STATE::CUBE_BACKROW_EXTEND;
                m_prevState = m_targetState;
            }
            else if (controller->IsButtonPressed(TeleopControlFunctions::HOLD_POSITION_EXTEND))
            {
                m_targetState = EXTENDER_STATE::HOLD_POSITION_EXTEND;
                m_prevState = m_targetState;
            }
            else if (controller->IsButtonPressed(TeleopControlFunctions::CONE_BACKROW_EXTEND))
            {
                m_targetState = EXTENDER_STATE::CONE_BACKROW_EXTEND;
                m_prevState = m_targetState;
            }
            else if (controller->IsButtonPressed(TeleopControlFunctions::CUBE_MIDROW_EXTEND))
            {
                m_targetState = EXTENDER_STATE::CUBE_MIDROW_EXTEND;
                m_prevState = m_targetState;
            }
            else if (controller->IsButtonPressed(TeleopControlFunctions::CONE_MIDROW_EXTEND))
            {
                m_targetState = EXTENDER_STATE::CONE_MIDROW_EXTEND;
                m_prevState = m_targetState;
            }
            else if (controller->IsButtonPressed(TeleopControlFunctions::HUMAN_PLAYER_STATION_EXTEND))
            {
                m_targetState = EXTENDER_STATE::HUMAN_PLAYER_STATION_EXTEND;
                m_prevState = m_targetState;
            }
            else if (controller->IsButtonPressed(TeleopControlFunctions::STARTING_POSITION_EXTEND))
            {
                m_targetState = EXTENDER_STATE::STARTING_POSITION_EXTEND;
                m_prevState = m_targetState;
            }
            else if (controller->IsButtonPressed(TeleopControlFunctions::FLOOR_EXTEND))
            {
                m_targetState = EXTENDER_STATE::FLOOR_EXTEND;
                m_prevState = m_targetState;
            }
            else
            {
                m_targetState = EXTENDER_STATE::HOLD_POSITION_EXTEND;
            }*/
            /// DEBUGGING, all of these functions are needed for testing
        }
    }
}

void ExtenderStateMgr::CheckForConeGamepadTransitions(TeleopControl *controller)
{
    if (m_gamepieceMode == RobotStateChanges::None)
    {
        m_gamepieceMode = RobotStateChanges::Cone;
        RobotState::GetInstance()->PublishStateChange(RobotStateChanges::DesiredGamePiece, m_gamepieceMode);
    }

    if (controller != nullptr)
    {
        if (controller->IsButtonPressed(TeleopControlFunctions::BACKROW))
        {
            m_targetState = EXTENDER_STATE::CONE_BACKROW_EXTEND;
            m_prevState = m_targetState;
        }
        else if (controller->IsButtonPressed(TeleopControlFunctions::MIDROW))
        {
            m_targetState = EXTENDER_STATE::CONE_MIDROW_EXTEND;
            m_prevState = m_targetState;
        }
        else
        {
            m_targetState = EXTENDER_STATE::HOLD_POSITION_EXTEND;
        }
    }
}

void ExtenderStateMgr::CheckForCubeGamepadTransitions(TeleopControl *controller)
{
    if (controller != nullptr)
    {
        if (controller->IsButtonPressed(TeleopControlFunctions::BACKROW))
        {
            m_targetState = EXTENDER_STATE::CUBE_BACKROW_EXTEND;
            m_prevState = m_targetState;
        }
        else if (controller->IsButtonPressed(TeleopControlFunctions::MIDROW))
        {
            m_targetState = EXTENDER_STATE::CUBE_MIDROW_EXTEND;
            m_prevState = m_targetState;
        }
        else
        {
            m_targetState = EXTENDER_STATE::HOLD_POSITION_EXTEND;
        }
    }
}
/// @brief Check driver input to determine target state
void ExtenderStateMgr::CheckForSensorTransitions()
{
    if (m_extender != nullptr)
    {
        // If we are hitting limit switches, reset position
        m_extender->ResetIfFullyExtended(m_extendedPosition);
        m_extender->ResetIfFullyRetracted();
    }
}

/// @brief Check if driver inputs or sensors trigger a state transition
void ExtenderStateMgr::CheckForStateTransition()
{
    CheckForSensorTransitions();

    if (m_checkGamePadTransitions)
    {
        CheckForGamepadTransitions();
    }

    /// DEBUGGING
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ExtenderMgr"), string("Current State"), m_targetState);

    if (m_extender != nullptr)
    {
        if (m_targetState != m_currentState || m_targetState == EXTENDER_STATE::MANUAL_EXTEND_RETRACT)
        {
            SetCurrentState(m_targetState, true);
            RobotState::GetInstance()->PublishStateChange(RobotStateChanges::ArmExtenderState, m_targetState);

            if (m_targetState == EXTENDER_STATE::HOLD_POSITION_EXTEND)
            {
                m_extender->UpdateTarget(dynamic_cast<Mech1IndMotorState *>(GetStateVector()[m_prevState])->GetCurrentTarget()); // Get the target of the previous state by referencing the state vector
            }
        }
    }
}

void ExtenderStateMgr::Update(RobotStateChanges::StateChange change, int value)
{
    if (change == RobotStateChanges::StateChange::ArmRotateState)
    {
        m_armState = static_cast<ArmStateMgr::ARM_STATE>(value);
        if (m_armState != ArmStateMgr::ARM_STATE::HOLD_POSITION_ROTATE)
        {
            m_goToStartingConfig = true;
        }
        else
        {
            m_goToStartingConfig = false;
        }
    }
    else if (change == RobotStateChanges::DesiredGamePiece)
    {
        m_gamepieceMode = static_cast<RobotStateChanges::GamePiece>(value);
    }
}
