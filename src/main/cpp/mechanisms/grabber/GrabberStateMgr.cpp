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
#include <mechanisms/grabber/grabber.h>
#include <mechanisms/grabber/grabberState.h>
#include <mechanisms/grabber/grabberStateMgr.h>
#include <robotstate/RobotState.h>
#include <robotstate/RobotStateChanges.h>
#include <utils/logging/Logger.h>

// Third Party Includes

//========= Hand modified code start section 0 ========

//========= Hand modified code end section 0 ========

using namespace std;

GrabberStateMgr *GrabberStateMgr::m_instance = nullptr;
GrabberStateMgr *GrabberStateMgr::GetInstance()
{
    if (GrabberStateMgr::m_instance == nullptr)
    {
        auto grabberPtr = MechanismFactory::GetMechanismFactory()->GetGrabber();
        if (grabberPtr != nullptr)
        {
            GrabberStateMgr::m_instance = new GrabberStateMgr();
        }
    }
    return GrabberStateMgr::m_instance;
}

/// @brief    initialize the state manager, parse the configuration file and create the states.
GrabberStateMgr::GrabberStateMgr() : StateMgr(),
                                     m_grabber(MechanismFactory::GetMechanismFactory()->GetGrabber())
                                     //========= Hand modified code start section 1 ========
                                     ,
                                     IRobotStateChangeSubscriber(),
                                     m_currentState(GRABBER_STATE::GRAB),
                                     m_targetState(GRABBER_STATE::GRAB)
//========= Hand modified code end section 1 ========

{
    map<string, StateStruc> stateMap;
    stateMap["OPEN"] = m_openState;
    stateMap["GRAB"] = m_grabState;

    Init(m_grabber, stateMap);
    if (m_grabber != nullptr)
    {
        m_grabber->AddStateMgr(this);
    }

    //========= Hand modified code start section 2 ========
    RobotState::GetInstance()->RegisterForStateChanges(this, RobotStateChanges::StateChange::ArmRotateState);
    //========= Hand modified code end section 2 ========
}

/// @brief  Get the current Parameter parm value for the state of this mechanism
/// @param PrimitiveParams* currentParams current set of primitive parameters
/// @returns int state id - -1 indicates that there is not a state to set
int GrabberStateMgr::GetCurrentStateParam(
    PrimitiveParams *currentParams)
{
    // normally get the state from primitive params
    return StateMgr::GetCurrentStateParam(currentParams);
}

/// @brief Check if driver inputs or sensors trigger a state transition
void GrabberStateMgr::CheckForStateTransition()
{
    //========= Hand modified code start section 3 ========
    CheckForSensorTransitions();
    if (m_grabber != nullptr)
    {
        // if (!m_followOtherMechs)
        //{
        CheckForGamepadTransitions();
        //}

        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, std::string("GrabberStateMgr"), std::string("m_targetState"), m_targetState);

        if (m_targetState != m_currentState)
        {
            SetCurrentState(m_targetState, true);
            RobotState::GetInstance()->PublishStateChange(RobotStateChanges::GrabberState, m_targetState);
            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, std::string("GrabberStateMgr"), std::string("Changing state to: "), m_targetState);
        }
    }
    //========= Hand modified code end section 3 ========
}

//========= Hand modified code start section 4 ========
/// @brief Check for sensor input to transition
void GrabberStateMgr::CheckForSensorTransitions()
{
    if (m_grabber != nullptr)
    {
        // look at banner sensor to determine target state
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, std::string("GrabberStateMgr"), std::string("DigitalInputValue"), m_grabber->IsGamePiecePresent());
        if (m_grabber->IsGamePiecePresent())
        {
            m_targetState = GRABBER_STATE::GRAB;
        }
    }
}

/// @brief Check for gamepad input to transition
void GrabberStateMgr::CheckForGamepadTransitions()
{
    if (m_grabber != nullptr)
    {
        m_currentState = static_cast<GRABBER_STATE>(GetCurrentState());
        // m_targetState = m_currentState;

        auto controller = TeleopControl::GetInstance();
        if (controller != nullptr)
        {
            if (controller->IsButtonPressed(TeleopControlFunctions::OPEN))
            {
                m_targetState = GRABBER_STATE::OPEN;
            }
            else if (controller->IsButtonPressed(TeleopControlFunctions::GRAB))
            {
                m_targetState = GRABBER_STATE::GRAB;
            }
        }
    }
}

void GrabberStateMgr::Update(RobotStateChanges::StateChange change, int value)
{
    if (change == RobotStateChanges::StateChange::ArmRotateState)
    {
        // If we are going to the HP substation, open the grabber, otherwise follow the sensors or gamepad input
        // don't update m_followOtherMechs in HOLD_POSITION because we will go to that after reaching HP target, then open the grabber <- We dont want this
        ArmStateMgr::ARM_STATE armState = static_cast<ArmStateMgr::ARM_STATE>(value);
        if (armState == ArmStateMgr::ARM_STATE::HUMAN_PLAYER_STATION_ROTATE)
        {
            m_targetState = GRABBER_STATE::OPEN;
            m_followOtherMechs = true;
        }
        else if (armState != ArmStateMgr::ARM_STATE::HOLD_POSITION_ROTATE)
        {
            m_followOtherMechs = false;
        }
    }
}
//========= Hand modified code end section 4 ========
