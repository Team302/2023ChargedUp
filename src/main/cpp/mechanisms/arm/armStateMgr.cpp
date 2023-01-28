
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
#include <mechanisms/arm/Arm.h>
#include <mechanisms/arm/ArmState.h>
#include <mechanisms/arm/ArmStateMgr.h>

// Third Party Includes

using namespace std;


ArmStateMgr* ArmStateMgr::m_instance = nullptr;
ArmStateMgr* ArmStateMgr::GetInstance()
{
	if ( ArmStateMgr::m_instance == nullptr )
	{
        auto armPtr = MechanismFactory::GetMechanismFactory()->Getarm();
        if (armPtr != nullptr)
        {
            ArmStateMgr::m_instance = new ArmStateMgr();
        }
	}
	return ArmStateMgr::m_instance;
    
}


/// @brief    initialize the state manager, parse the configuration file and create the states.
ArmStateMgr::ArmStateMgr() : StateMgr(),
                                     m_arm(MechanismFactory::GetMechanismFactory()->Getarm())
{
    map<string, StateStruc> stateMap;
	stateMap["HOLD_POSITION_ROTATE"] = m_hold_position_rotateState;
stateMap["MANUAL_ROTATE"] = m_manual_rotateState;
stateMap["CUBE_BACKROW_ROTATE"] = m_cube_backrow_rotateState;
stateMap["CONE_BACKROW_ROTATE"] = m_cone_backrow_rotateState;
stateMap["CUBE_MIDROW_ROTATE"] = m_cube_midrow_rotateState;
stateMap["CONE_MIDROW_ROTATE"] = m_cone_midrow_rotateState;
stateMap["HUMAN_PLAYER_STATION_ROTATE"] = m_human_player_station_rotateState;
stateMap["STARTING_POSITION_ROTATE"] = m_starting_position_rotateState;
stateMap["FLOOR_POSITION_ROTATE"] = m_floor_position_rotateState;


    Init(m_arm, stateMap);
    if (m_arm != nullptr)
    {
        m_arm->AddStateMgr(this);
    }
}   

/// @brief  Get the current Parameter parm value for the state of this mechanism
/// @param PrimitiveParams* currentParams current set of primitive parameters
/// @returns int state id - -1 indicates that there is not a state to set
int ArmStateMgr::GetCurrentStateParam
(
    PrimitiveParams*    currentParams
) 
{
    // normally get the state from primitive params
    return StateMgr::GetCurrentStateParam(currentParams);
}

void ArmStateMgr::CheckForSensorTransitions()
{
	//========= Do not erase this line and the one below it. They are used by the code generator ========		
	//========= Hand modified code start section 0 ========
	
     	/// override this method if sensors could change states 

    //========= Hand modified code end section 0 ========
	//========= Do not erase this line and the one above it. They are used by the code generator =======
}

void ArmStateMgr::CheckForGamepadTransitions()
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

