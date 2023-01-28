
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

#pragma once

// C++ Includes
#include <string>

// FRC includes

// Team 302 includes
#include <mechanisms/base/StateMgr.h>
#include <mechanisms/Extender/extender.h>
#include <mechanisms/StateStruc.h>

// Third Party Includes

class ExtenderStateMgr : public StateMgr
{
    public:
	    /// @enum the various states of the Intake
        enum EXTENDER_STATE
        {
            HOLD_POSITION_EXTEND,
MANUAL_EXTEND_RETRACT,
CUBE_BACKROW_EXTEND,
CONE_BACKROW_EXTEND,
CUBE_MIDROW_EXTEND,
CONE_MIDROW_EXTEND,
HUMAN_PLAYER_STATION_EXTEND,
STARTING_POSITION_EXTEND,
FLOOR_EXTEND
        };
		
        const std::map<const std::string, EXTENDER_STATE> m_extenderXmlStringToStateEnumMap
        {   
			{"HOLD_POSITION_EXTEND", EXTENDER_STATE::HOLD_POSITION_EXTEND},
{"MANUAL_EXTEND_RETRACT", EXTENDER_STATE::MANUAL_EXTEND_RETRACT},
{"CUBE_BACKROW_EXTEND", EXTENDER_STATE::CUBE_BACKROW_EXTEND},
{"CONE_BACKROW_EXTEND", EXTENDER_STATE::CONE_BACKROW_EXTEND},
{"CUBE_MIDROW_EXTEND", EXTENDER_STATE::CUBE_MIDROW_EXTEND},
{"CONE_MIDROW_EXTEND", EXTENDER_STATE::CONE_MIDROW_EXTEND},
{"HUMAN_PLAYER_STATION_EXTEND", EXTENDER_STATE::HUMAN_PLAYER_STATION_EXTEND},
{"STARTING_POSITION_EXTEND", EXTENDER_STATE::STARTING_POSITION_EXTEND},
{"FLOOR_EXTEND", EXTENDER_STATE::FLOOR_EXTEND}
        };
        
		/// @brief  Find or create the state manmanager
		static ExtenderStateMgr* GetInstance();

        /// @brief  Get the current Parameter parm value for the state of this mechanism
        /// @param PrimitiveParams* currentParams current set of primitive parameters
        /// @returns int state id - -1 indicates that there is not a state to set
        int GetCurrentStateParam
        (
            PrimitiveParams*    currentParams
        ) override;

        void CheckForStateTransition() override;
		
    private:
	
        ExtenderStateMgr();
        ~ExtenderStateMgr() = default;
        
        Extender*                                m_extender;

		static ExtenderStateMgr*	m_instance;

		const StateStruc m_hold_position_extendState = { EXTENDER_STATE::HOLD_POSITION_EXTEND, "HOLD_POSITION_EXTEND", StateType::EXTENDER_STATE, true };
const StateStruc m_manual_extend_retractState = { EXTENDER_STATE::MANUAL_EXTEND_RETRACT, "MANUAL_EXTEND_RETRACT", StateType::EXTENDER_STATE, true };
const StateStruc m_cube_backrow_extendState = { EXTENDER_STATE::CUBE_BACKROW_EXTEND, "CUBE_BACKROW_EXTEND", StateType::EXTENDER_STATE, true };
const StateStruc m_cone_backrow_extendState = { EXTENDER_STATE::CONE_BACKROW_EXTEND, "CONE_BACKROW_EXTEND", StateType::EXTENDER_STATE, true };
const StateStruc m_cube_midrow_extendState = { EXTENDER_STATE::CUBE_MIDROW_EXTEND, "CUBE_MIDROW_EXTEND", StateType::EXTENDER_STATE, true };
const StateStruc m_cone_midrow_extendState = { EXTENDER_STATE::CONE_MIDROW_EXTEND, "CONE_MIDROW_EXTEND", StateType::EXTENDER_STATE, true };
const StateStruc m_human_player_station_extendState = { EXTENDER_STATE::HUMAN_PLAYER_STATION_EXTEND, "HUMAN_PLAYER_STATION_EXTEND", StateType::EXTENDER_STATE, true };
const StateStruc m_starting_position_extendState = { EXTENDER_STATE::STARTING_POSITION_EXTEND, "STARTING_POSITION_EXTEND", StateType::EXTENDER_STATE, true };
const StateStruc m_floor_extendState = { EXTENDER_STATE::FLOOR_EXTEND, "FLOOR_EXTEND", StateType::EXTENDER_STATE, true };

};

