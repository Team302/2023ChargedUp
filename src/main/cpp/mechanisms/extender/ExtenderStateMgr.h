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
#include <frc/Timer.h>

// Team 302 includes
#include <mechanisms/base/StateMgr.h>
#include <mechanisms/Extender/extender.h>
#include <mechanisms/StateStruc.h>

//========= Hand modified code start section 0 ========
#include <robotstate/IRobotStateChangeSubscriber.h>
#include <mechanisms/arm/ArmStateMgr.h>
//========= Hand modified code end section 0 ========

// Third Party Includes

//========= Hand modified code start section 1 ========

//========= Hand modified code end section 1 ========

class ExtenderStateMgr : public StateMgr
    //========= Hand modified code start section 2 ========
    ,
                         public IRobotStateChangeSubscriber
//========= Hand modified code end section 2 ========
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
        FLOOR_EXTEND,
        INITIALIZE
    };

    const std::map<const std::string, EXTENDER_STATE> m_extenderXmlStringToStateEnumMap{
        {"HOLD_POSITION_EXTEND", EXTENDER_STATE::HOLD_POSITION_EXTEND},
        {"MANUAL_EXTEND_RETRACT", EXTENDER_STATE::MANUAL_EXTEND_RETRACT},
        {"CUBE_BACKROW_EXTEND", EXTENDER_STATE::CUBE_BACKROW_EXTEND},
        {"CONE_BACKROW_EXTEND", EXTENDER_STATE::CONE_BACKROW_EXTEND},
        {"CUBE_MIDROW_EXTEND", EXTENDER_STATE::CUBE_MIDROW_EXTEND},
        {"CONE_MIDROW_EXTEND", EXTENDER_STATE::CONE_MIDROW_EXTEND},
        {"HUMAN_PLAYER_STATION_EXTEND", EXTENDER_STATE::HUMAN_PLAYER_STATION_EXTEND},
        {"STARTING_POSITION_EXTEND", EXTENDER_STATE::STARTING_POSITION_EXTEND},
        {"FLOOR_EXTEND", EXTENDER_STATE::FLOOR_EXTEND},
        {"INITIALIZE", EXTENDER_STATE::INITIALIZE}};

    /// @brief  Find or create the state manmanager
    static ExtenderStateMgr *GetInstance();

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
    ExtenderStateMgr();
    ~ExtenderStateMgr() = default;

    //========= Hand modified code start section 4 ========
    void CheckForConeGamepadTransitions(TeleopControl *controller);
    void CheckForCubeGamepadTransitions(TeleopControl *controller);
    //========= Hand modified code end section 4 ========

    Extender *m_extender;

    //========= Hand modified code start section 5 ========
    EXTENDER_STATE m_prevState;
    EXTENDER_STATE m_currentState;
    EXTENDER_STATE m_targetState;

    RobotStateChanges::GamePiece m_gamepieceMode;

    bool m_canAutomaticallyMove = false;
    bool m_goToStartingConfig = true;

    bool m_hasInitialized = false;

    frc::Timer *m_initializationTimer;

    double m_extendedPosition;
    int m_armAngle = 0.0;
    int m_armTargetAngle = 0.0;
    ArmStateMgr::ARM_STATE m_armState = ArmStateMgr::ARM_STATE::HOLD_POSITION_ROTATE;
    //========= Hand modified code end section 5 ========

    static ExtenderStateMgr *m_instance;

    const StateStruc m_hold_position_extendState = {EXTENDER_STATE::HOLD_POSITION_EXTEND, "HOLD_POSITION_EXTEND", StateType::EXTENDER_STATE, false};
    const StateStruc m_manual_extend_retractState = {EXTENDER_STATE::MANUAL_EXTEND_RETRACT, "MANUAL_EXTEND_RETRACT", StateType::MANUAL_EXTENDER_STATE, false};
    const StateStruc m_cube_backrow_extendState = {EXTENDER_STATE::CUBE_BACKROW_EXTEND, "CUBE_BACKROW_EXTEND", StateType::EXTENDER_STATE, false};
    const StateStruc m_cone_backrow_extendState = {EXTENDER_STATE::CONE_BACKROW_EXTEND, "CONE_BACKROW_EXTEND", StateType::EXTENDER_STATE, false};
    const StateStruc m_cube_midrow_extendState = {EXTENDER_STATE::CUBE_MIDROW_EXTEND, "CUBE_MIDROW_EXTEND", StateType::EXTENDER_STATE, false};
    const StateStruc m_cone_midrow_extendState = {EXTENDER_STATE::CONE_MIDROW_EXTEND, "CONE_MIDROW_EXTEND", StateType::EXTENDER_STATE, false};
    const StateStruc m_human_player_station_extendState = {EXTENDER_STATE::HUMAN_PLAYER_STATION_EXTEND, "HUMAN_PLAYER_STATION_EXTEND", StateType::EXTENDER_STATE, false};
    const StateStruc m_starting_position_extendState = {EXTENDER_STATE::STARTING_POSITION_EXTEND, "STARTING_POSITION_EXTEND", StateType::EXTENDER_STATE, true};
    const StateStruc m_floor_extendState = {EXTENDER_STATE::FLOOR_EXTEND, "FLOOR_EXTEND", StateType::EXTENDER_STATE, false};
    const StateStruc m_initializeState = {EXTENDER_STATE::INITIALIZE, "INITIALIZE", StateType::EXTENDER_STATE, false};

    const double m_armAngleTolerance = 10.0;
    const double m_armFloorTolerance = 11.0;
};
