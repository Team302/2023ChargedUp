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
}
void DriverFeedback::UpdateLEDStates()
{
    if (DriverFeedback::m_AlignedWithConeNode)
    {

        if (currentState != DriverFeedbackStates::ALIGNED_WITH_CONE_NODE)
        {
            m_LEDStates->ResetVariables();
        }
        m_LEDStates->BlinkingPattern(LED::YELLOW);
        currentState = DriverFeedbackStates::ALIGNED_WITH_CONE_NODE;
    }
    else if (DriverFeedback::m_AlignedWithCubeNode)
    {

        if (currentState != DriverFeedbackStates::ALIGNED_WITH_CUBE_NODE)
        {
            m_LEDStates->ResetVariables();
        }
        m_LEDStates->BlinkingPattern(LED::PURPLE);
        currentState = DriverFeedbackStates::ALIGNED_WITH_CUBE_NODE;
    }
    else if (DriverFeedback::m_GamePieceInGrabber)
    {

        if (currentState != DriverFeedbackStates::GAME_PIECE_IN_GRABBER)
        {
            m_LEDStates->ResetVariables();
        }
        m_LEDStates->AlternatingBlinkingPattern(LED::YELLOW, LED::PURPLE);
        currentState = DriverFeedbackStates::ALIGNED_WITH_CUBE_NODE;
    }
    else if (DriverFeedback::m_WantCube)
    {

        if (currentState != DriverFeedbackStates::WANT_CUBE)
        {
            m_LEDStates->ResetVariables();
        }
        m_LEDStates->SolidColorPattern(LED::PURPLE);
        currentState = DriverFeedbackStates::WANT_CUBE;
    }
    else if (DriverFeedback::m_WantCone)
    {

        if (currentState != DriverFeedbackStates::WANT_CONE)
        {
            m_LEDStates->ResetVariables();
        }
        m_LEDStates->SolidColorPattern(LED::YELLOW);
        currentState = DriverFeedbackStates::WANT_CONE;
    }
    else if (DriverFeedback::m_GamePieceReadyToPickUp)
    {

        if (currentState != DriverFeedbackStates::GAME_PIECE_READY_TO_PICK_UP)
        {
            m_LEDStates->ResetVariables();
        }
        m_LEDStates->SolidColorPattern(LED::GREEN);
        currentState = DriverFeedbackStates::GAME_PIECE_READY_TO_PICK_UP;
    }
    else
    {

        if (currentState != DriverFeedbackStates::NONE)
        {
            m_LEDStates->ResetVariables();
        }
        m_LEDStates->SolidColorPattern(LED::GREEN);
        currentState = DriverFeedbackStates::NONE;
    }
}

DriverFeedback::DriverFeedback() : IRobotStateChangeSubscriber()
{
    RobotState::GetInstance()->RegisterForStateChanges(this, RobotStateChanges::StateChange::DesiredGamePiece);
    RobotState::GetInstance()->RegisterForStateChanges(this, RobotStateChanges::StateChange::GameState);
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
}
