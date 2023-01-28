
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
#include <mechanisms/Grabber/grabber.h>
#include <mechanisms/StateStruc.h>

// Third Party Includes

class GrabberStateMgr : public StateMgr
{
    public:
	    /// @enum the various states of the Intake
        enum GRABBER_STATE
        {
            OPEN,
GRABBING_CONE,
GRABBING_CUBE,
HOLDING_CONE,
HOLDING_CUBE,
RELEASE
        };
		
        const std::map<const std::string, GRABBER_STATE> m_grabberXmlStringToStateEnumMap
        {   
			{"OPEN", GRABBER_STATE::OPEN},
{"GRABBING_CONE", GRABBER_STATE::GRABBING_CONE},
{"GRABBING_CUBE", GRABBER_STATE::GRABBING_CUBE},
{"HOLDING_CONE", GRABBER_STATE::HOLDING_CONE},
{"HOLDING_CUBE", GRABBER_STATE::HOLDING_CUBE},
{"RELEASE", GRABBER_STATE::RELEASE}
        };
        
		/// @brief  Find or create the state manmanager
		static GrabberStateMgr* GetInstance();

        /// @brief  Get the current Parameter parm value for the state of this mechanism
        /// @param PrimitiveParams* currentParams current set of primitive parameters
        /// @returns int state id - -1 indicates that there is not a state to set
        int GetCurrentStateParam
        (
            PrimitiveParams*    currentParams
        ) override;

		void CheckForSensorTransitions() override;
		void CheckForGamepadTransitions() override;

    private:
	
        GrabberStateMgr();
        ~GrabberStateMgr() = default;
        
        Grabber*                                m_grabber;

		static GrabberStateMgr*	m_instance;

		const StateStruc m_openState = { GRABBER_STATE::OPEN, "OPEN", StateType::GRABBER_STATE, true };
const StateStruc m_grabbing_coneState = { GRABBER_STATE::GRABBING_CONE, "GRABBING_CONE", StateType::GRABBER_STATE, true };
const StateStruc m_grabbing_cubeState = { GRABBER_STATE::GRABBING_CUBE, "GRABBING_CUBE", StateType::GRABBER_STATE, true };
const StateStruc m_holding_coneState = { GRABBER_STATE::HOLDING_CONE, "HOLDING_CONE", StateType::GRABBER_STATE, true };
const StateStruc m_holding_cubeState = { GRABBER_STATE::HOLDING_CUBE, "HOLDING_CUBE", StateType::GRABBER_STATE, true };
const StateStruc m_releaseState = { GRABBER_STATE::RELEASE, "RELEASE", StateType::GRABBER_STATE, true };

};

