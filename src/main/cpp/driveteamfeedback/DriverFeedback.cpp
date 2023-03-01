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

#include <driveteamfeedback/DriverFeedback.h>
#include <robotstate/RobotState.h>
#include <robotstate/RobotStateChanges.h>
#include <robotstate/IRobotStateChangeSubscriber.h>

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
}
void DriverFeedback::UpdateCompressorState()
{
    if (RobotStateChanges::CompressorState::CompressorOn)
    {
        m_compressorState = DriverFeedbackStates::COMPRESSOR_ON;
    }

    else if (RobotStateChanges::CompressorState::CompressorOff)
    {
        m_compressorState = DriverFeedbackStates::COMPRESSOR_OFF;
    }
}
void DriverFeedback::UpdateLEDStates()
{
    if (DriverFeedback::m_AlignedWithConeNode)
    {

        if (m_gamePieceState != DriverFeedbackStates::ALIGNED_WITH_CONE_NODE)
        {
            m_LEDStates->ResetVariables();
        }
        m_LEDStates->ClosingInChaserPattern(LED::YELLOW);
        m_gamePieceState = DriverFeedbackStates::ALIGNED_WITH_CONE_NODE;
    }
    else if (DriverFeedback::m_AlignedWithCubeNode)
    {

        if (m_gamePieceState != DriverFeedbackStates::ALIGNED_WITH_CUBE_NODE)
        {
            m_LEDStates->ResetVariables();
        }
        m_LEDStates->ClosingInChaserPattern(LED::PURPLE);
        m_gamePieceState = DriverFeedbackStates::ALIGNED_WITH_CUBE_NODE;
    }
    else if (DriverFeedback::m_GamePieceInGrabber)
    {

        if (m_gamePieceState != DriverFeedbackStates::GAME_PIECE_IN_GRABBER)
        {
            m_LEDStates->ResetVariables();
        }
        m_LEDStates->AlternatingBlinkingPattern(LED::YELLOW, LED::PURPLE);
        m_gamePieceState = DriverFeedbackStates::ALIGNED_WITH_CUBE_NODE;
    }
    else if (DriverFeedback::m_WantCube)
    {

        if (m_gamePieceState != DriverFeedbackStates::WANT_CUBE)
        {
            m_LEDStates->ResetVariables();
        }
        m_LEDStates->SolidColorPattern(LED::PURPLE);
        m_gamePieceState = DriverFeedbackStates::WANT_CUBE;
    }
    else if (DriverFeedback::m_WantCone)
    {

        if (m_gamePieceState != DriverFeedbackStates::WANT_CONE)
        {
            m_LEDStates->ResetVariables();
        }
        m_LEDStates->SolidColorPattern(LED::YELLOW);
        m_gamePieceState = DriverFeedbackStates::WANT_CONE;
    }
    else if (DriverFeedback::m_GamePieceReadyToPickUp)
    {

        if (m_gamePieceState != DriverFeedbackStates::GAME_PIECE_READY_TO_PICK_UP)
        {
            m_LEDStates->ResetVariables();
        }
        m_LEDStates->SolidColorPattern(LED::GREEN);
        m_gamePieceState = DriverFeedbackStates::GAME_PIECE_READY_TO_PICK_UP;
    }
    else
    {

        if (m_gamePieceState != DriverFeedbackStates::NONE)
        {
            m_LEDStates->ResetVariables();
        }
        m_LEDStates->SolidColorPattern(LED::GREEN);
        m_gamePieceState = DriverFeedbackStates::NONE;
    }
}

DriverFeedback::DriverFeedback() : IRobotStateChangeSubscriber()
{
    RobotState::GetInstance()->RegisterForStateChanges(this, RobotStateChanges::StateChange::DesiredGamePiece);
    RobotState::GetInstance()->RegisterForStateChanges(this, RobotStateChanges::StateChange::GameState);
    RobotState::GetInstance()->RegisterForStateChanges(this, RobotStateChanges::StateChange::CompressorChange);
}
void DriverFeedback::Update(RobotStateChanges::StateChange change, int value)
{
    if (change == RobotStateChanges::DesiredGamePiece)
    {
        auto gamepiece = static_cast<RobotStateChanges::GamePiece>(value);
        m_WantCube = gamepiece == RobotStateChanges::Cube;
        m_WantCone = gamepiece == RobotStateChanges::Cone;
    }
    else if (change == RobotStateChanges::GameState)
    {
        auto state = static_cast<RobotStateChanges::GamePeriod>(value);
        m_AutonomousEnabled = state == RobotStateChanges::Auton;
        m_TeleopEnabled = state == RobotStateChanges::Teleop;
    }
    else if (change == RobotStateChanges::StateChange::CompressorChange)
    {
        auto compressor = static_cast<RobotStateChanges::CompressorState>(value);
    }
}
