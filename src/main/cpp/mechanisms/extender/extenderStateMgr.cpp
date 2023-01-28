
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
#include <mechanisms/extender/Extender.h>
#include <mechanisms/extender/ExtenderState.h>
#include <mechanisms/extender/ExtenderStateMgr.h>

// Third Party Includes

using namespace std;


ExtenderStateMgr* ExtenderStateMgr::m_instance = nullptr;
ExtenderStateMgr* ExtenderStateMgr::GetInstance()
{
	if ( ExtenderStateMgr::m_instance == nullptr )
	{
        auto extenderPtr = MechanismFactory::GetMechanismFactory()->Getextender();
        if (extenderPtr != nullptr)
        {
            ExtenderStateMgr::m_instance = new ExtenderStateMgr();
        }
	}
	return ExtenderStateMgr::m_instance;
    
}


/// @brief    initialize the state manager, parse the configuration file and create the states.
ExtenderStateMgr::ExtenderStateMgr() : StateMgr(),
                                     m_extender(MechanismFactory::GetMechanismFactory()->Getextender())
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
}   

/// @brief  Get the current Parameter parm value for the state of this mechanism
/// @param PrimitiveParams* currentParams current set of primitive parameters
/// @returns int state id - -1 indicates that there is not a state to set
int ExtenderStateMgr::GetCurrentStateParam
(
    PrimitiveParams*    currentParams
) 
{
    // normally get the state from primitive params
    return StateMgr::GetCurrentStateParam(currentParams);
}

void ExtenderStateMgr::CheckForSensorTransitions()
{
	//========= Do not erase this line and the one below it. They are used by the code generator ========		
	//========= Hand modified code start section 0 ========
	
        // override this method if sensors could change states 
	
    //========= Hand modified code end section 0 ========
	//========= Do not erase this line and the one above it. They are used by the code generator =======
}

void ExtenderStateMgr::CheckForGamepadTransitions()
{
	//========= Do not erase this line and the one below it. They are used by the code generator ========		
	//========= Hand modified code start section 1 ========

    // override this method if joystick inputs could change states;  Format 
    // would look something like:
    //    auto controller = TeleopControl::GetInstance();
    //    if ( controller != nullptr )
    //    {
    //          code here that checks the inputs
    //    }

	//========= Hand modified code end section 1 ========
	//========= Do not erase this line and the one above it. They are used by the code generator =======
}

