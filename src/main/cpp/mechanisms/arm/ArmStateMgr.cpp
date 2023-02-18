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
#include <mechanisms/arm/arm.h>
#include <mechanisms/arm/armState.h>
#include <mechanisms/arm/armStateMgr.h>
#include <robotstate/RobotState.h>
#include <robotstate/RobotStateChanges.h>
#include <utils/logging/Logger.h>

// Third Party Includes

//========= Hand modified code start section 0 ========
//========= Hand modified code end section 0 ========

using namespace std;

ArmStateMgr *ArmStateMgr::m_instance = nullptr;
ArmStateMgr *ArmStateMgr::GetInstance()
{
    if (ArmStateMgr::m_instance == nullptr)
    {
        auto armPtr = MechanismFactory::GetMechanismFactory()->GetArm();
        if (armPtr != nullptr)
        {
            ArmStateMgr::m_instance = new ArmStateMgr();
        }
    }
    return ArmStateMgr::m_instance;
}

/// @brief    initialize the state manager, parse the configuration file and create the states.
ArmStateMgr::ArmStateMgr() : StateMgr(),
                             IRobotStateChangeSubscriber(),
                             m_arm(MechanismFactory::GetMechanismFactory()->GetArm()),
                             //========= Hand modified code start section 1 ========
                             m_prevState(ARM_STATE::STARTING_POSITION_ROTATE),
                             m_currentState(ARM_STATE::STARTING_POSITION_ROTATE),
                             m_targetState(ARM_STATE::STARTING_POSITION_ROTATE),
                             m_gamepieceMode(RobotStateChanges::None)
//========= Hand modified code end section 1 ========

{
    map<string, StateStruc> stateMap;
    stateMap["HOLD_POSITION_ROTATE"] = m_hold_position_rotateState;
    stateMap["MANUAL_ROTATE"] = m_manual_rotateState;
    stateMap["CUBE_BACKROW_ROTATE"] = m_cube_backrow_rotateState;
    stateMap["CONE_BACKROW_ROTATE"] = m_cone_backrow_rotateState;
    stateMap["CUBE_MIDROW_ROTATE_UP"] = m_cube_midrow_rotate_upState;
    stateMap["CUBE_MIDROW_ROTATE_DOWN"] = m_cube_midrow_rotate_downState;
    stateMap["CONE_MIDROW_ROTATE_UP"] = m_cone_midrow_rotate_upState;
    stateMap["CONE_MIDROW_ROTATE_DOWN"] = m_cone_midrow_rotate_downState;
    stateMap["HUMAN_PLAYER_STATION_ROTATE"] = m_human_player_station_rotateState;
    stateMap["STARTING_POSITION_ROTATE"] = m_starting_position_rotateState;
    stateMap["FLOOR_POSITION_ROTATE"] = m_floor_position_rotateState;

    RobotState::GetInstance()->RegisterForStateChanges(this, RobotStateChanges::StateChange::DesiredGamePiece);
    RobotState::GetInstance()->RegisterForStateChanges(this, RobotStateChanges::StateChange::GrabberState);

    Init(m_arm, stateMap);
    if (m_arm != nullptr)
    {
        m_arm->AddStateMgr(this);
    }

    //========= Hand modified code start section 2 ========
    RobotState::GetInstance()->RegisterForStateChanges(this, RobotStateChanges::StateChange::DesiredGamePiece);
    //========= Hand modified code end section 2 ========
}

/// @brief  Get the current Parameter parm value for the state of this mechanism
/// @param PrimitiveParams* currentParams current set of primitive parameters
/// @returns int state id - -1 indicates that there is not a state to set
int ArmStateMgr::GetCurrentStateParam(
    PrimitiveParams *currentParams)
{
    // normally get the state from primitive params
    return StateMgr::GetCurrentStateParam(currentParams);
}

/// @brief Check if driver inputs or sensors trigger a state transition
void ArmStateMgr::CheckForStateTransition()
{
    //========= Hand modified code start section 3 ========
    CheckForSensorTransitions();
    if (m_checkGamePadTransitions)
    {
        CheckForGamepadTransitions();
    }
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArmMgr"), string("Current State"), m_targetState);

    if (m_targetState != m_currentState)
    {
        SetCurrentState(m_targetState, true);
        RobotState::GetInstance()->PublishStateChange(RobotStateChanges::ArmRotateState, m_targetState);

        if (m_targetState == ARM_STATE::HOLD_POSITION_ROTATE)
        {
            double armAngle = m_arm->GetPositionDegrees().to<double>();
            double extenderPos = MechanismFactory::GetMechanismFactory()->GetExtender()->GetPositionInches().to<double>();
            // holding currently based on just "F term" Created surface map function based on arm and extender position
            if (m_arm->GetPositionDegrees().to<double>() > m_fTermAngleThreshold)
            {
                if (extenderPos > m_fullExtensionExtenderPos && armAngle > m_fullExtensionArmAngle)
                {
                    // specific f term for outlier position
                    m_arm->UpdateTarget(0.115);
                }
                else if (m_gamepieceMode == RobotStateChanges::GamePiece::Cube || m_grabberState == GrabberStateMgr::GRABBER_STATE::OPEN)
                {
                    // f term function for cube
                    m_arm->UpdateTarget(m_cubeOffset + m_cubeArmComponent * armAngle + m_cubeExtenderComponent * extenderPos + m_cubeArmSquaredComponent * pow(armAngle, 2) + m_cubeExtenderSquaredComponent * pow(extenderPos, 2));
                }
                else if (m_gamepieceMode == RobotStateChanges::GamePiece::Cone && m_grabberState == GrabberStateMgr::GRABBER_STATE::GRAB)
                {
                    // f term function for cone
                    m_arm->UpdateTarget(m_coneOffset + m_coneArmComponent * armAngle + m_coneExtenderComponent * extenderPos + m_coneArmSquaredComponent * pow(armAngle, 2) + m_coneExtenderSquaredComponent * pow(extenderPos, 2));
                }
            }
        }
    }
    //========= Hand modified code end section 3 ========
}

//========= Hand modified code start section 4 ========
/// @brief Check driver inputs for a state transition
void ArmStateMgr::CheckForGamepadTransitions()
{
    if (m_gamepieceMode == RobotStateChanges::None)
    {
        m_gamepieceMode = RobotStateChanges::Cone;
        RobotState::GetInstance()->PublishStateChange(RobotStateChanges::DesiredGamePiece, m_gamepieceMode);
    }
    if (m_arm != nullptr)
    {
        auto controller = TeleopControl::GetInstance();
        if (controller != nullptr)
        {
            m_currentState = static_cast<ARM_STATE>(GetCurrentState());
            m_targetState = m_currentState;

            if (abs(controller->GetAxisValue(TeleopControlFunctions::MANUAL_ROTATE)) > 0.05)
            {
                m_targetState = ARM_STATE::MANUAL_ROTATE;
                m_prevState = m_targetState;
            }
            else if (m_gamepieceMode == RobotStateChanges::Cone)
            {
                CheckForConeGamepadTransitions(controller);
            }
            else if (m_gamepieceMode == RobotStateChanges::Cube)
            {
                CheckForCubeGamepadTransitions(controller);
            }

            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArmMgr"), string("Target"), m_arm->GetTarget());
            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArmMgr"), string("Target State"), m_targetState);
            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArmMgr"), string("Previous State"), m_prevState);
            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArmMgr"), string("Position (Degrees)"), m_arm->GetPositionDegrees().to<double>());
            // If arm is at target and the prev state hasn't changed then stay in hold
            if (abs(m_arm->GetPositionDegrees().to<double>() - m_arm->GetTarget()) < 1.0 && m_arm->GetPositionDegrees().to<double>() > 1.0 && m_prevState == m_targetState)
            {
                m_targetState = ARM_STATE::HOLD_POSITION_ROTATE;
            }
        }
    }
}

void ArmStateMgr::CheckForConeGamepadTransitions(TeleopControl *controller)
{
    if (controller != nullptr)
    {
        if (controller->IsButtonPressed(TeleopControlFunctions::BACKROW))
        {
            m_targetState = ARM_STATE::CONE_BACKROW_ROTATE;
            m_prevState = m_targetState;
        }
        else if (controller->IsButtonPressed(TeleopControlFunctions::MIDROW))
        {
            if (m_arm->GetPositionDegrees().to<double>() > dynamic_cast<ArmState *>(GetSpecifiedState(ARM_STATE::CONE_MIDROW_ROTATE_DOWN))->GetCurrentTarget())
            {
                m_targetState = ARM_STATE::CONE_MIDROW_ROTATE_DOWN;
            }
            else
            {
                m_targetState = ARM_STATE::CONE_MIDROW_ROTATE_UP;
            }

            m_prevState = m_targetState;
        }
        else if (controller->IsButtonPressed(TeleopControlFunctions::FLOOR_POSITION))
        {
            m_targetState = ARM_STATE::FLOOR_POSITION_ROTATE;
            m_prevState = m_targetState;
        }
        else if (controller->IsButtonPressed(TeleopControlFunctions::HUMAN_PLAYER_STATION))
        {
            m_targetState = ARM_STATE::HUMAN_PLAYER_STATION_ROTATE;
            m_prevState = m_targetState;
        }
        else
        {
            m_targetState = ARM_STATE::HOLD_POSITION_ROTATE;
        }
    }
}

void ArmStateMgr::CheckForCubeGamepadTransitions(TeleopControl *controller)
{
    if (controller != nullptr)
    {
        if (controller->IsButtonPressed(TeleopControlFunctions::BACKROW))
        {
            m_targetState = ARM_STATE::CUBE_BACKROW_ROTATE;
            m_prevState = m_targetState;
        }
        else if (controller->IsButtonPressed(TeleopControlFunctions::MIDROW))
        {
            if (m_arm->GetPositionDegrees().to<double>() > dynamic_cast<ArmState *>(GetSpecifiedState(ARM_STATE::CUBE_MIDROW_ROTATE_DOWN))->GetCurrentTarget())
            {
                m_targetState = ARM_STATE::CUBE_MIDROW_ROTATE_DOWN;
            }
            else
            {
                m_targetState = ARM_STATE::CUBE_MIDROW_ROTATE_UP;
            }

            m_prevState = m_targetState;
        }
        else if (controller->IsButtonPressed(TeleopControlFunctions::FLOOR_POSITION))
        {
            m_targetState = ARM_STATE::FLOOR_POSITION_ROTATE;
            m_prevState = m_targetState;
        }
        else if (controller->IsButtonPressed(TeleopControlFunctions::HUMAN_PLAYER_STATION))
        {
            m_targetState = ARM_STATE::HUMAN_PLAYER_STATION_ROTATE;
            m_prevState = m_targetState;
        }
        else
        {
            m_targetState = ARM_STATE::HOLD_POSITION_ROTATE;
        }
    }
}

/// @brief Check sensors for a state transition
void ArmStateMgr::CheckForSensorTransitions()
{
    if (m_arm != nullptr)
    {
        m_currentState = static_cast<ARM_STATE>(GetCurrentState());
        m_targetState = m_currentState;

        // If we are hitting limit switch, reset position
        m_arm->ResetIfArmDown();

        // Check arm angle and run any states dependent on it
    }
}

void ArmStateMgr::Update(RobotStateChanges::StateChange change, int state)
{
    if (change == RobotStateChanges::DesiredGamePiece)
    {
        m_gamepieceMode = static_cast<RobotStateChanges::GamePiece>(state);
    }
    else if (change == RobotStateChanges::GrabberState)
    {
        m_grabberState = static_cast<GrabberStateMgr::GRABBER_STATE>(state);
    }
}
//========= Hand modified code end section 4 ========
