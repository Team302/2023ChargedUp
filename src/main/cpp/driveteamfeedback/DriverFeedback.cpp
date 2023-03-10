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

#include <frc/DriverStation.h>
#include <driveteamfeedback/DriverFeedback.h>
#include <hw/factories/CompressorFactory.h>
#include <robotstate/RobotState.h>
#include <robotstate/RobotStateChanges.h>
#include <robotstate/IRobotStateChangeSubscriber.h>
#include <mechanisms/grabber/GrabberStateMgr.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <utils/logging/LoggerData.h>
#include <string>
#include <utils/logging/Logger.h>

using frc::DriverStation;
using namespace std;

DriverFeedback *DriverFeedback::m_instance = nullptr;

DriverFeedback *DriverFeedback::GetInstance()
{
    if (DriverFeedback::m_instance == nullptr)
    {
        DriverFeedback::m_instance = new DriverFeedback();
    }
    return DriverFeedback::m_instance;
}

void DriverFeedback::UpdateFeedback()
{
    UpdateLEDStates();
    UpdateCompressorState();
    CheckControllers();
    DisplayPressure();
}
void DriverFeedback::UpdateCompressorState()
{
    if (m_controllerCounter == 0)
    {
        auto table = nt::NetworkTableInstance::GetDefault().GetTable("Compressor");
        table.get()->PutBoolean(std::string("Compressor on"), m_compressorOn);
    }
}
void DriverFeedback::DisplayPressure()
{
    auto table = nt::NetworkTableInstance::GetDefault().GetTable("Compressor");
    table.get()->GetNumber(std::string("Pressure"), CompressorFactory::GetFactory()->GetCurrentPressure().to<double>());
}
void DriverFeedback::UpdateLEDStates()
{
    if (DriverFeedback::m_AlignedWithConeNode)
    {
        if (m_gamePieceState != DriverFeedbackStates::ALIGNED_WITH_CONE_NODE)
        {
            m_LEDStates->ResetVariables();
        }
        m_LEDStates->ClosingInChaserPattern(DragonLeds::YELLOW);
        m_gamePieceState = DriverFeedbackStates::ALIGNED_WITH_CONE_NODE;
    }
    else if (DriverFeedback::m_AlignedWithCubeNode)
    {
        if (m_gamePieceState != DriverFeedbackStates::ALIGNED_WITH_CUBE_NODE)
        {
            m_LEDStates->ResetVariables();
        }
        m_LEDStates->ClosingInChaserPattern(DragonLeds::PURPLE);
        m_gamePieceState = DriverFeedbackStates::ALIGNED_WITH_CUBE_NODE;
    }
    else if (DriverFeedback::m_GamePieceInGrabber)
    {

        if (m_gamePieceState != DriverFeedbackStates::GAME_PIECE_IN_GRABBER)
        {
            m_LEDStates->ResetVariables();
        }
        m_LEDStates->AlternatingColorBlinkingPattern(DragonLeds::YELLOW, DragonLeds::PURPLE);
        m_gamePieceState = DriverFeedbackStates::ALIGNED_WITH_CUBE_NODE;
    }
    else if (DriverFeedback::m_WantCube)
    {
        if (m_gamePieceState != DriverFeedbackStates::WANT_CUBE)
        {
            m_LEDStates->ResetVariables();
            m_gamePieceState = DriverFeedbackStates::WANT_CUBE;
        }
        if (m_grabberStateChanged)
        {
            if (m_GrabberIsOpen)
                m_LEDStates->BlinkingPattern(DragonLeds::PURPLE);
            else
                m_LEDStates->SolidColorPattern(DragonLeds::PURPLE);
        }
    }
    else if (DriverFeedback::m_WantCone)
    {
        if (m_gamePieceState != DriverFeedbackStates::WANT_CONE)
        {
            m_LEDStates->ResetVariables();
            m_gamePieceState = DriverFeedbackStates::WANT_CONE;
        }
        if (m_grabberStateChanged)
        {
            if (m_GrabberIsOpen)
                m_LEDStates->BlinkingPattern(DragonLeds::YELLOW);
            else
                m_LEDStates->SolidColorPattern(DragonLeds::YELLOW);
        }
        m_LEDStates->SolidColorPattern(DragonLeds::YELLOW);
        m_gamePieceState = DriverFeedbackStates::WANT_CONE;
    }
    else if (DriverFeedback::m_GamePieceReadyToPickUp)
    {
        if (m_gamePieceState != DriverFeedbackStates::GAME_PIECE_READY_TO_PICK_UP)
        {
            m_LEDStates->ResetVariables();
            m_LEDStates->SolidColorPattern(DragonLeds::GREEN);
            m_gamePieceState = DriverFeedbackStates::GAME_PIECE_READY_TO_PICK_UP;
        }
    }
    else
    {
        if (m_gamePieceState != DriverFeedbackStates::NONE)
        {
            m_LEDStates->ResetVariables();
            m_LEDStates->SolidColorPattern(DragonLeds::GREEN);
            m_gamePieceState = DriverFeedbackStates::NONE;
        }
    }
}

void DriverFeedback::resetRequests(void)
{
    m_GrabberIsOpen = false;
    m_WantCube = false;
    m_WantCone = false;
    m_GamePieceReadyToPickUp = false;
    m_GamePieceInGrabber = false;
    m_AlignedWithConeNode = false;
    m_AlignedWithCubeNode = false;

    m_grabberStateChanged = true;
}

DriverFeedback::DriverFeedback() : IRobotStateChangeSubscriber()
{
    RobotState::GetInstance()->RegisterForStateChanges(this, RobotStateChanges::StateChange::GrabberState);
    RobotState::GetInstance()->RegisterForStateChanges(this, RobotStateChanges::StateChange::DesiredGamePiece);
    RobotState::GetInstance()->RegisterForStateChanges(this, RobotStateChanges::StateChange::GameState);
    RobotState::GetInstance()->RegisterForStateChanges(this, RobotStateChanges::StateChange::CompressorChange);
    RobotState::GetInstance()->RegisterForStateChanges(this, RobotStateChanges::StateChange::FMSConection);
}
void DriverFeedback::Update(RobotStateChanges::StateChange change, int value)
{
    if (change == RobotStateChanges::DesiredGamePiece)
    {
        auto gamepiece = static_cast<RobotStateChanges::GamePiece>(value);
        m_WantCube = gamepiece == RobotStateChanges::Cube;
        m_WantCone = gamepiece == RobotStateChanges::Cone;
    }
    else if (change == RobotStateChanges::GrabberState)
    {
        auto state = static_cast<GrabberStateMgr::GRABBER_STATE>(value);
        bool newState = state == GrabberStateMgr::GRABBER_STATE::OPEN;

        if (m_GrabberIsOpen != newState)
        {
            m_grabberStateChanged = true;
            m_GrabberIsOpen = newState;
        }
    }
    else if (change == RobotStateChanges::GameState)
    {
        auto state = static_cast<RobotStateChanges::GamePeriod>(value);
        m_AutonomousEnabled = state == RobotStateChanges::Auton;
        m_TeleopEnabled = state == RobotStateChanges::Teleop;

        resetRequests();
    }
    else if (change == RobotStateChanges::StateChange::CompressorChange)
    {
        auto compressor = static_cast<RobotStateChanges::CompressorState>(value);
        m_compressorOn = compressor == RobotStateChanges::CompressorOn;
    }

    else if (change == RobotStateChanges::StateChange::FMSConection)
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("AutonomousInit"), string("FMS Connected"));
    }
}

void DriverFeedback::CheckControllers()
{
    if (m_controllerCounter == 0)
    {
        auto table = nt::NetworkTableInstance::GetDefault().GetTable("XBOX Controller");
        for (auto i = 0; i < DriverStation::kJoystickPorts; ++i)
        {
            table.get()->PutBoolean(std::string("Controller") + std::to_string(i), DriverStation::GetJoystickIsXbox(i));
        }
    }
    m_controllerCounter++;
    if (m_controllerCounter > 25)
    {
        m_controllerCounter = 0;
    }
}