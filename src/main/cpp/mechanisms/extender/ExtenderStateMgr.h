
#pragma once

// C++ Includes
#include <string>

// Team 302 includes
#include <mechanisms/base/StateMgr.h>
#include <mechanisms/Extender/extender.h>
#include <mechanisms/StateStruc.h>

#include <mechanisms/arm/ArmStateMgr.h>
//========= Hand modified code end section 0 ========

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

private:
    ExtenderStateMgr();
    ~ExtenderStateMgr() = default;

    Extender *m_extender;

    EXTENDER_STATE m_currentState;
    EXTENDER_STATE m_targetState;

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
};
