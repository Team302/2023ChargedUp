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

#pragma once

// C++ Includes
#include <string>

// FRC includes

// Team 302 includes
#include <mechanisms/base/StateMgr.h>
#include <mechanisms/Grabber/grabber.h>
#include <mechanisms/StateStruc.h>

//========= Hand modified code start section 0 ========
#include <robotstate/IRobotStateChangeSubscriber.h>
//========= Hand modified code end section 0 ========

// Third Party Includes

//========= Hand modified code start section 1 ========

//========= Hand modified code end section 1 ========

class GrabberStateMgr : public StateMgr
    //========= Hand modified code start section 2 ========
    ,
                        public IRobotStateChangeSubscriber
//========= Hand modified code end section 2 ========
{
public:
    /// @enum the various states of the Intake
    enum GRABBER_STATE
    {
        OPEN,
        GRAB
    };

    const std::map<const std::string, GRABBER_STATE> m_grabberXmlStringToStateEnumMap{
        {"OPEN", GRABBER_STATE::OPEN},
        {"GRAB", GRABBER_STATE::GRAB}};

    /// @brief  Find or create the state manmanager
    static GrabberStateMgr *GetInstance();

    /// @brief  Get the current Parameter parm value for the state of this mechanism
    /// @param PrimitiveParams* currentParams current set of primitive parameters
    /// @returns int state id - -1 indicates that there is not a state to set
    int GetCurrentStateParam(
        PrimitiveParams *currentParams) override;

    void CheckForStateTransition() override;
    //========= Hand modified code start section 3 ========
    void CheckForSensorTransitions() override;
    void CheckForGamepadTransitions() override;

    // RobotState override
    void Update(RobotStateChanges::StateChange change, int value) override;
    //========= Hand modified code end section 3 ========

private:
    GrabberStateMgr();
    ~GrabberStateMgr() = default;

    //========= Hand modified code start section 4 ========

    //========= Hand modified code end section 4 ========

    Grabber *m_grabber;

    //========= Hand modified code start section 5 ========
    GRABBER_STATE m_currentState;
    GRABBER_STATE m_targetState;

    bool m_followOtherMechs = false;
    //========= Hand modified code end section 5 ========

    static GrabberStateMgr *m_instance;

    const StateStruc m_openState = {GRABBER_STATE::OPEN, "OPEN", StateType::GRABBER_STATE, false};
    const StateStruc m_grabState = {GRABBER_STATE::GRAB, "GRAB", StateType::GRABBER_STATE, true};
};
