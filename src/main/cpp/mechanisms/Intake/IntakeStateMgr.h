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
#pragma once

// C++ Includes
#include <string>

// FRC includes

// Team 302 includes
#include <mechanisms/base/StateMgr.h>
#include <mechanisms/intake/Intake.h>
#include <mechanisms/StateStruc.h>

#include <robotstate/IRobotStateChangeSubscriber.h>

// Third Party Includes

class IntakeStateMgr : public StateMgr, public IRobotStateChangeSubscriber
{
public:
    /// @enum the various states of the Intake
    enum INTAKE_STATE
    {
        OFF,
        INTAKE,
        HOLD,
        RELEASE,
        EXPEL,
        INTAKE_EXPEL_LOW,
        HP_CONE_INTAKE,
        HOLD_CUBE
    };
    const std::string m_intakeOffXMLString{"INTAKE_OFF"};
    const std::string m_intakeXMLString{"INTAKE"};
    const std::string m_holdXMLString{"INTAKE_HOLD"};
    const std::string m_releaseXMLString{"INTAKE_RELEASE"};
    const std::string m_expelXMLString{"INTAKE_EXPEL"};
    const std::string m_expelLowXMLString{"INTAKE_EXPEL_LOW"};
    const std::string m_hpconeintakeXMLString{"HP_CONE_INTAKE"};
    const std::string m_holdCubeXMLString{"INTAKE_HOLD_CUBE"};

    const std::map<const std::string, INTAKE_STATE> m_intakeXmlStringToStateEnumMap{
        {m_intakeOffXMLString, INTAKE_STATE::OFF},
        {m_intakeXMLString, INTAKE_STATE::INTAKE},
        {m_holdXMLString, INTAKE_STATE::HOLD},
        {m_releaseXMLString, INTAKE_STATE::RELEASE},
        {m_expelXMLString, INTAKE_STATE::EXPEL},
        {m_expelLowXMLString, INTAKE_STATE::INTAKE_EXPEL_LOW},
        {m_hpconeintakeXMLString, INTAKE_STATE::HP_CONE_INTAKE},
        {m_holdCubeXMLString, INTAKE_STATE::HOLD_CUBE}};

    /// @brief  Find or create the state manmanager
    static IntakeStateMgr *GetInstance();

    /// @brief  Get the current Parameter parm value for the state of this mechanism
    /// @param PrimitiveParams* currentParams current set of primitive parameters
    /// @returns int state id - -1 indicates that there is not a state to set
    int GetCurrentStateParam(PrimitiveParams *currentParams) override;

    void CheckForStateTransition() override;
    void CheckForSensorTransitions() override;
    void CheckForGamepadTransitions() override;

    // RobotState override
    void Update(RobotStateChanges::StateChange change, int value) override;

private:
    IntakeStateMgr();
    ~IntakeStateMgr() = default;

    Intake *m_intake;

    INTAKE_STATE m_currentState;
    INTAKE_STATE m_targetState;
    INTAKE_STATE m_prevState;

    bool m_coneMode = true;
    bool m_isHP = false;
    int m_armAngle = 0;
    static IntakeStateMgr *m_instance;

    static constexpr int HUMAN_PLAYER_STATION_ANGLE = 55;
    static constexpr int LOW_RELEASE_ANGLE = 60;

    const StateStruc m_offState = {INTAKE_STATE::OFF, m_intakeOffXMLString, StateType::INTAKE_STATE, true};
    const StateStruc m_intakeState = {INTAKE_STATE::INTAKE, m_intakeXMLString, StateType::INTAKE_STATE, false};
    const StateStruc m_holdState = {INTAKE_STATE::HOLD, m_holdXMLString, StateType::INTAKE_STATE, false};
    const StateStruc m_releaseState = {INTAKE_STATE::RELEASE, m_releaseXMLString, StateType::INTAKE_STATE, false};
    const StateStruc m_expelState = {INTAKE_STATE::EXPEL, m_expelXMLString, StateType::INTAKE_STATE, false};
    const StateStruc m_expelLowState = {INTAKE_STATE::INTAKE_EXPEL_LOW, m_expelLowXMLString, StateType::INTAKE_STATE, false};
    const StateStruc m_hpconeintakeState = {INTAKE_STATE::HP_CONE_INTAKE, m_hpconeintakeXMLString, StateType::INTAKE_STATE, false};
    const StateStruc m_holdCubeState = {INTAKE_STATE::HOLD_CUBE, m_holdCubeXMLString, StateType::INTAKE_STATE, false};
};
