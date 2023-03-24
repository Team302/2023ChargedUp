
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

// Team302 Includes
#include <driveteamfeedback/LEDStates.h>
#include <robotstate/IRobotStateChangeSubscriber.h>
#include <utils/DragonField.h>
#include <auton/AutonPreviewer.h>
#include <auton/CyclePrimitives.h>

class SwerveChassis;

class DriverFeedback : public IRobotStateChangeSubscriber
{
public:
    void UpdateFeedback();

    static DriverFeedback *GetInstance();

    static DriverFeedback *GetInstance(CyclePrimitives *cyclePrims);

    void UpdateLEDStates();

    void UpdateCompressorState();

    void Update(RobotStateChanges::StateChange change, int value) override;

    DragonField *GetField() const { return m_field; }

private:
    void CheckControllers();
    void DisplayPressure();
    DriverFeedback();
    ~DriverFeedback() = default;

    bool m_AutonomousEnabled = false;
    bool m_TeleopEnabled = false;

    enum DriverFeedbackStates
    {
        ALIGNED_WITH_CONE_NODE,
        ALIGNED_WITH_CUBE_NODE,
        GAME_PIECE_IN_GRABBER,
        WANT_CUBE,
        WANT_CONE,
        GAME_PIECE_READY_TO_PICK_UP,
        COMPRESSOR_ON,
        COMPRESSOR_OFF,
        NONE
    };

    LEDStates *m_LEDStates = LEDStates::GetInstance();
    bool m_GrabberIsOpen = false;
    bool m_WantCube = false;
    bool m_WantCone = false;
    bool m_GamePieceReadyToPickUp = false;
    bool m_GamePieceInGrabber = false;
    bool m_AlignedWithConeNode = false;
    bool m_AlignedWithCubeNode = false;
    int m_controllerCounter = 0;
    bool m_compressorOn = true;

    static DriverFeedback *m_instance;

    static CyclePrimitives *m_cyclePrims;
    bool m_hasCreatedPreviewer = false;

    DriverFeedbackStates m_gamePieceState = DriverFeedbackStates::NONE;
    DriverFeedbackStates m_compressorState = DriverFeedbackStates::NONE;
    bool m_grabberStateChanged = true;

    DragonField *m_field;
    AutonPreviewer *m_previewer = nullptr;
    SwerveChassis *m_chassis;

    void resetRequests(void);
};
