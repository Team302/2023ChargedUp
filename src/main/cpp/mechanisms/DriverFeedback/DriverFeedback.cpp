
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

#include <mechanisms/DriverFeedback/DriverFeedback.h>

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
    if (TeleopEnabled)
    {
        if (DriverFeedback::m_AlignedWithConeNode)
        {

            if (currentState != DriverFeedbackStates::ALIGNED_WITH_CONE_NODE)
            {
                m_LEDStates->ResetVariables();
            }
            m_LEDStates->ClosingInChaserPattern(LED::YELLOW);
            currentState = DriverFeedbackStates::ALIGNED_WITH_CONE_NODE;
        }
        else if (DriverFeedback::m_AlignedWithCubeNode)
        {

            if (currentState != DriverFeedbackStates::ALIGNED_WITH_CUBE_NODE)
            {
                m_LEDStates->ResetVariables();
            }
            m_LEDStates->ClosingInChaserPattern(LED::PURPLE);
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
            m_LEDStates->SolidColorPattern(LED::BLACK);
            currentState = DriverFeedbackStates::NONE;
        }
    }
    else if(AutonomousEnabled)
    {
        m_LEDStates->ChaserPattern(LED::GREEN); 
        currentState = DriverFeedbackStates::NONE;
    }
}
void DriverFeedback::AlignedWithConeNode(bool AlignedWithConeNode)
{
    DriverFeedback::m_AlignedWithConeNode = AlignedWithConeNode;
}
void DriverFeedback::AlignedWithCubeNode(bool AlignedWithCubeNode)
{
    DriverFeedback::m_AlignedWithCubeNode = AlignedWithCubeNode;
}
void DriverFeedback::WantCone(bool WantCone)
{
    DriverFeedback::m_WantCone = WantCone;
}
void DriverFeedback::WantCube(bool WantCube)
{
    DriverFeedback::m_WantCube = WantCube;
}
void DriverFeedback::GamePieceInGrabber(bool GamePieceInGrabber)
{
    DriverFeedback::m_GamePieceInGrabber = GamePieceInGrabber;
}
void DriverFeedback::GamePieceReadyToPickUp(bool GamePieceReadyToPickUp)
{
    DriverFeedback::m_GamePieceReadyToPickUp = GamePieceReadyToPickUp;
}
