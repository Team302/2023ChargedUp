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
#include <algorithm>
#include <map>
#include <string>

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

// Third Party Includes

//========= Hand modified code start section 0 ========

//========= Hand modified code end section 0 ========

using std::abs;
using std::allocator;
using std::less;
using std::map;
using std::pair;
using std::string;

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
                                       m_extender(MechanismFactory::GetMechanismFactory()->GetExtender())
                                       //========= Hand modified code start section 1 ========
                                       ,
                                       m_desiredState(EXTENDER_STATE::INITIALIZE),
                                       m_currentState(EXTENDER_STATE::INITIALIZE),
                                       m_targetState(EXTENDER_STATE::INITIALIZE),
                                       m_gamepieceMode(RobotStateChanges::None),
                                       m_initializationTimer(new frc::Timer()),
                                       m_extendedPosition(84320.3176), // 22.25 inches in counts for extender
                                       m_armState(ArmStateMgr::ARM_STATE::HOLD_POSITION_ROTATE)
//========= Hand modified code end section 1 ========

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
    stateMap["INITIALIZE"] = m_initializeState;

    Init(m_extender, stateMap);
    if (m_extender != nullptr)
    {
        m_extender->AddStateMgr(this);
    }

    //========= Hand modified code start section 2 ========
    RobotState::GetInstance()->RegisterForStateChanges(this, RobotStateChanges::StateChange::ArmRotateState);
    RobotState::GetInstance()->RegisterForStateChanges(this, RobotStateChanges::StateChange::ArmRotateAngleTarget);
    RobotState::GetInstance()->RegisterForStateChanges(this, RobotStateChanges::StateChange::ArmRotateAngle);
    RobotState::GetInstance()->RegisterForStateChanges(this, RobotStateChanges::StateChange::DesiredGamePiece);

    m_initializationTimer->Reset();
    //========= Hand modified code end section 2 ========
}

/// @brief  Get the current Parameter parm value for the state of this mechanism
/// @param PrimitiveParams* currentParams current set of primitive parameters
/// @returns int state id - -1 indicates that there is not a state to set
int ExtenderStateMgr::GetCurrentStateParam(PrimitiveParams *currentParams)
{
    auto state = currentParams->GetExtenderState();
    m_desiredState = state;
    return static_cast<int>(state);
}

/// @brief Check if driver inputs or sensors trigger a state transition
void ExtenderStateMgr::CheckForStateTransition()
{
    //========= Hand modified code start section 3 ========
    if (m_gamepieceMode == RobotStateChanges::None)
    {
        m_gamepieceMode = RobotStateChanges::Cone;
        RobotState::GetInstance()->PublishStateChange(RobotStateChanges::DesiredGamePiece, m_gamepieceMode);
    }

    m_currentState = static_cast<EXTENDER_STATE>(GetCurrentState());

    if (!m_timerStarted)
    {
        m_initializationTimer->Start();
        m_timerStarted = true;
    }

    // initalize extender
    if (!m_hasInitialized)
    {
        m_targetState = EXTENDER_STATE::INITIALIZE;
    }

    CheckForSensorTransitions();
    if (m_checkGamePadTransitions)
    {
        CheckForGamepadTransitions();
    }

    auto armInsideFrame = m_armAngle < m_armFloorTolerance;
    auto armAtTarget = abs(m_armAngle - (m_armTargetAngle / 11.37777777)) < m_armAngleTolerance;

    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "Extender", "Arm Angle", m_armAngle);
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "Extender", "Arm Target Angle", m_armTargetAngle);
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "Extender", "Arm Error", abs(m_armAngle - (m_armTargetAngle / 11.37777777)));
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "Extender", "Extender Pos", m_extender->GetPositionInches().to<double>());

    if ((armInsideFrame || !armAtTarget) &&
        m_targetState != EXTENDER_STATE::MANUAL_EXTEND_RETRACT &&
        m_targetState != EXTENDER_STATE::INITIALIZE &&
        m_armState != ArmStateMgr::ARM_STATE::MANUAL_ROTATE)
    {
        m_targetState = EXTENDER_STATE::STARTING_POSITION_EXTEND;
    }
    else if (m_desiredState != EXTENDER_STATE::INITIALIZE)
    {
        m_targetState = m_desiredState;
    }

    if (m_targetState != m_currentState)
    {
        SetCurrentState(m_targetState, true);
        RobotState::GetInstance()->PublishStateChange(RobotStateChanges::ArmExtenderState, m_targetState);
    }

    //========= Hand modified code end section 3 ========
}

//========= Hand modified code start section 4 ========
/// @brief Check sensors to determine target state
void ExtenderStateMgr::CheckForGamepadTransitions()
{
    auto controller = TeleopControl::GetInstance();
    if (controller != nullptr)
    {
        if (abs(controller->GetAxisValue(TeleopControlFunctions::MANUAL_EXTEND_RETRACT)) > 0.1)
        {
            m_desiredState = EXTENDER_STATE::MANUAL_EXTEND_RETRACT;
        }
        else if (controller->IsButtonPressed(TeleopControlFunctions::STARTING_POSITION))
        {
            m_desiredState = EXTENDER_STATE::STARTING_POSITION_EXTEND;
        }
        else if (controller->IsButtonPressed(TeleopControlFunctions::HUMAN_PLAYER_STATION))
        {
            m_desiredState = EXTENDER_STATE::HUMAN_PLAYER_STATION_EXTEND;
        }
        else if (controller->IsButtonPressed(TeleopControlFunctions::FLOOR_POSITION))
        {
            m_desiredState = EXTENDER_STATE::FLOOR_EXTEND;
        }
        else if (m_gamepieceMode == RobotStateChanges::Cone) // if we want cone or the gamepiece mode hasn't been updated
        {
            CheckForConeGamepadTransitions(controller);
        }
        else
        {
            CheckForCubeGamepadTransitions(controller);
        }
    }
}

void ExtenderStateMgr::CheckForConeGamepadTransitions(TeleopControl *controller)
{
    if (controller != nullptr)
    {
        if (controller->IsButtonPressed(TeleopControlFunctions::BACKROW))
        {
            m_desiredState = EXTENDER_STATE::CONE_BACKROW_EXTEND;
        }
        else if (controller->IsButtonPressed(TeleopControlFunctions::MIDROW))
        {
            m_desiredState = EXTENDER_STATE::CONE_MIDROW_EXTEND;
        }
    }
}

void ExtenderStateMgr::CheckForCubeGamepadTransitions(TeleopControl *controller)
{
    if (controller != nullptr)
    {
        if (controller->IsButtonPressed(TeleopControlFunctions::BACKROW))
        {
            m_desiredState = EXTENDER_STATE::CUBE_BACKROW_EXTEND;
        }
        else if (controller->IsButtonPressed(TeleopControlFunctions::MIDROW))
        {
            m_desiredState = EXTENDER_STATE::CUBE_MIDROW_EXTEND;
        }
    }
}
/// @brief Check driver input to determine target state
void ExtenderStateMgr::CheckForSensorTransitions()
{
    if (m_extender != nullptr)
    {
        // If we are hitting limit switches, reset position
        bool hittingLimitSwitch = m_extender->ResetIfFullyExtended(m_extendedPosition);
        hittingLimitSwitch = !hittingLimitSwitch ? m_extender->ResetIfFullyRetracted() : hittingLimitSwitch;

        if (!m_hasInitialized && (hittingLimitSwitch || m_initializationTimer->HasElapsed(units::time::second_t(0.45))))
        {
            m_hasInitialized = true;
            m_targetState = EXTENDER_STATE::STARTING_POSITION_EXTEND;
        }
    }
}

void ExtenderStateMgr::Update(RobotStateChanges::StateChange change, int value)
{
    if (change == RobotStateChanges::StateChange::ArmRotateAngle)
    {
        m_armAngle = value;
    }
    else if (change == RobotStateChanges::StateChange::ArmRotateAngleTarget)
    {
        m_armTargetAngle = value;
    }
    else if (change == RobotStateChanges::StateChange::ArmRotateState)
    {
        m_armState = static_cast<ArmStateMgr::ARM_STATE>(value);
    }
    else if (change == RobotStateChanges::StateChange::DesiredGamePiece)
    {
        m_gamepieceMode = static_cast<RobotStateChanges::GamePiece>(value);
    }
}
//========= Hand modified code end section 4 ========
