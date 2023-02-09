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

// Third Party Includes

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
                                     IRobotStateChangeSubscriber(),
                                     m_grabber(MechanismFactory::GetMechanismFactory()->GetGrabber()),
                                     m_currentState(GRABBER_STATE::OPEN),
                                     m_targetState(GRABBER_STATE::OPEN)
{
    map<string, StateStruc> stateMap;
    stateMap["OPEN"] = m_openState;
    stateMap["GRABBING_CONE"] = m_grabbing_coneState;
    stateMap["GRABBING_CUBE"] = m_grabbing_cubeState;
    stateMap["HOLDING_CONE"] = m_holding_coneState;
    stateMap["HOLDING_CUBE"] = m_holding_cubeState;
    stateMap["RELEASE"] = m_releaseState;

    Init(m_grabber, stateMap);
    if (m_grabber != nullptr)
    {
        m_grabber->AddStateMgr(this);
    }
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

/// @brief Check for sensor input to transition
void GrabberStateMgr::CheckForSensorTransitions()
{
    if (m_grabber != nullptr)
    {
        // look at banner sensor to determine target state
    }
}

/// @brief Check for gamepad input to transition
void GrabberStateMgr::CheckForGamepadTransitions()
{
    if (m_grabber != nullptr)
    {
        m_currentState = static_cast<GRABBER_STATE>(GetCurrentState());
        m_targetState = m_currentState;

        auto controller = TeleopControl::GetInstance();
        if (controller != nullptr)
        {
            if (controller->IsButtonPressed(TeleopControlFunctions::OPEN))
            {
                m_targetState = GRABBER_STATE::OPEN;
            }
            else if (controller->IsButtonPressed(TeleopControlFunctions::GRABBING_CONE))
            {
                m_targetState = GRABBER_STATE::GRABBING_CONE;
            }
            else if (controller->IsButtonPressed(TeleopControlFunctions::GRABBING_CUBE))
            {
                m_targetState = GRABBER_STATE::GRABBING_CUBE;
            }
            else if (controller->IsButtonPressed(TeleopControlFunctions::HOLDING_CONE))
            {
                m_targetState = GRABBER_STATE::HOLDING_CONE;
            }
            else if (controller->IsButtonPressed(TeleopControlFunctions::HOLDING_CUBE))
            {
                m_targetState = GRABBER_STATE::HOLDING_CUBE;
            }
            else if (controller->IsButtonPressed(TeleopControlFunctions::RELEASE))
            {
                m_targetState = GRABBER_STATE::RELEASE;
            }
            else
            {
                m_targetState = GRABBER_STATE::OPEN;
            }
        }
    }
}

/// @brief Check if driver inputs or sensors trigger a state transition
void GrabberStateMgr::CheckForStateTransition()
{

    if (m_grabber != nullptr)
    {
        if (m_targetState != m_currentState)
        {
            SetCurrentState(m_targetState, true);
            RobotState::GetInstance()->PublishStateChange(RobotStateChanges::GrabberState, m_targetState);
        }
    }
}
