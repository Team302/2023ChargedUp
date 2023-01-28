
//====================================================================================================================================================
/// Copyright 2022 Lake Orion Robotics FIRST Team 302 
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
#include <map>

// FRC includes

// Team 302 includes
#include <TeleopControl.h>
#include <auton/PrimitiveParams.h>
#include <mechanisms/MechanismFactory.h>
#include <mechanisms/base/StateMgr.h>
#include <mechanisms/StateStruc.h>
#include <mechanisms/grabber/Grabber.h>
#include <mechanisms/grabber/GrabberState.h>
#include <mechanisms/grabber/GrabberStatemgr.h>

// Third Party Includes

using namespace std;


GrabberStateMgr* GrabberStateMgr::m_instance = nullptr;
GrabberStateMgr* GrabberStateMgr::GetInstance()
{
	if ( GrabberStateMgr::m_instance == nullptr )
	{
        auto grabberPtr = MechanismFactory::GetMechanismFactory()->Getgrabber();
        if (grabberPtr != nullptr)
        {
            GrabberStateMgr::m_instance = new GrabberStateMgr();
        }
	}
	return GrabberStateMgr::m_instance;
    
}


/// @brief    initialize the state manager, parse the configuration file and create the states.
GrabberStateMgr::GrabberStateMgr() : StateMgr(),
                                     m_grabber(MechanismFactory::GetMechanismFactory()->Getgrabber())
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
int GrabberStateMgr::GetCurrentStateParam
(
    PrimitiveParams*    currentParams
) 
{
    // normally get the state from primitive params
    return StateMgr::GetCurrentStateParam(currentParams);
}

/// @brief Check if driver inputs or sensors trigger a state transition
void GrabberStateMgr::CheckForStateTransition()
{

    if ( m_grabber != nullptr )
    {    
        auto currentState = static_cast<GRABBER_STATE>(GetCurrentState());
        auto targetState = currentState;

        //========= Do not erase this line and the one below it. They are used by the code generator ========		
		//========= Hand modified code start section 0 ========
	
     	// Write logic here. See example below
		/*
        auto controller = TeleopControl::GetInstance();
        auto isForwardSelected   = controller != nullptr ? controller->IsButtonPressed(TeleopControl::FUNCTION_IDENTIFIER::EXAMPLE_FORWARD) : false;
        auto isReverseSelected   = controller != nullptr ? controller->IsButtonPressed(TeleopControl::FUNCTION_IDENTIFIER::EXAMPLE_REVERSE) : false;

        if (isForwardSelected)
        {
            targetState = EXAMPLE_STATE::FORWARD;
        }
        else if (isReverseSelected)
        {
            targetState = EXAMPLE_STATE::REVERSE;
        }
        else
        {
            targetState = EXAMPLE_STATE::OFF;
        }

        if (targetState != currentState)
        {
            SetCurrentState(targetState, true);
        }
        */

		//========= Hand modified code end section 0 ========
        //========= Do not erase this line and the one above it. They are used by the code generator =======
    }
}


