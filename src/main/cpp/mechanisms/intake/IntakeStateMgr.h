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
        INTAKE_CONE,
        HOLD_CONE,
        DROP_CONE,
        EXPEL_CONE,
        INTAKE_CUBE,
        HOLD_CUBE,
        EXPEL_CUBE
    };
    const std::string m_intakeOffXMLString{"INTAKE_OFF"};
    const std::string m_intakeConeXMLString{"INTAKE_CONE"};
    const std::string m_holdConeXMLString{"HOLD_CONE"};
    const std::string m_dropConeXMLString{"DROP_CONE"};
    const std::string m_expelConeXMLString{"EXPEL_CONE"};
    const std::string m_intakeCubeXMLString{"INTAKE_CUBE"};
    const std::string m_holdCubeXMLString{"HOLD_CUBE"};
    const std::string m_expelCubeXMLString{"EXPEL_CUBE"};

    const std::map<const std::string, INTAKE_STATE> m_intakeXmlStringToStateEnumMap{
        {m_intakeOffXMLString, INTAKE_STATE::OFF},
        {m_intakeConeXMLString, INTAKE_STATE::INTAKE_CONE},
        {m_holdConeXMLString, INTAKE_STATE::HOLD_CONE},
        {m_dropConeXMLString, INTAKE_STATE::DROP_CONE},
        {m_expelConeXMLString, INTAKE_STATE::EXPEL_CONE},
        {m_intakeCubeXMLString, INTAKE_STATE::INTAKE_CUBE},
        {m_holdCubeXMLString, INTAKE_STATE::HOLD_CUBE},
        {m_expelCubeXMLString, INTAKE_STATE::EXPEL_CUBE}};

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

    bool m_coneMode;

    static IntakeStateMgr *m_instance;

    const StateStruc m_offState = {INTAKE_STATE::OFF, m_intakeOffXMLString, StateType::INTAKE_STATE, true};
    const StateStruc m_intakeConeState = {INTAKE_STATE::INTAKE_CONE, m_intakeConeXMLString, StateType::INTAKE_STATE, false};
    const StateStruc m_holdConeState = {INTAKE_STATE::HOLD_CONE, m_holdConeXMLString, StateType::INTAKE_STATE, false};
    const StateStruc m_dropConeState = {INTAKE_STATE::DROP_CONE, m_dropConeXMLString, StateType::INTAKE_STATE, false};
    const StateStruc m_expelConeState = {INTAKE_STATE::EXPEL_CONE, m_expelConeXMLString, StateType::INTAKE_STATE, false};
    const StateStruc m_intakeCubeState = {INTAKE_STATE::INTAKE_CUBE, m_intakeCubeXMLString, StateType::INTAKE_STATE, false};
    const StateStruc m_holdCubeState = {INTAKE_STATE::HOLD_CUBE, m_holdCubeXMLString, StateType::INTAKE_STATE, false};
    const StateStruc m_expelCubeState = {INTAKE_STATE::EXPEL_CUBE, m_expelCubeXMLString, StateType::INTAKE_STATE, false};
};
