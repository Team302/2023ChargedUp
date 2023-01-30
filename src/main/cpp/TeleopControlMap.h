
//====================================================================================================================================================
// Copyright 2022 Lake Orion Robotics FIRST Team 302 
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

// FRC includes

// Team 302 includes
#include <gamepad/IDragonGamePad.h>
#include <TeleopControlAxis.h>
#include <TeleopControlButton.h>
#include <TeleopControlFunctions.h>

#include <RobinHood/robin_hood.h>


class TeleopControlMap
{
    public:

        const TeleopControlButton driverAButton = {TeleopControlMappingEnums::DRIVER, TeleopControlMappingEnums::A_BUTTON, TeleopControlMappingEnums::STANDARD};
        const TeleopControlButton driverBButton = {TeleopControlMappingEnums::DRIVER, TeleopControlMappingEnums::B_BUTTON, TeleopControlMappingEnums::STANDARD};
        const TeleopControlButton driverXButton = {TeleopControlMappingEnums::DRIVER, TeleopControlMappingEnums::X_BUTTON, TeleopControlMappingEnums::STANDARD};
        const TeleopControlButton driverYButton = {TeleopControlMappingEnums::DRIVER, TeleopControlMappingEnums::Y_BUTTON, TeleopControlMappingEnums::STANDARD};
        const TeleopControlButton driverLBumper = {TeleopControlMappingEnums::DRIVER, TeleopControlMappingEnums::LEFT_BUMPER, TeleopControlMappingEnums::STANDARD};
        const TeleopControlButton driverRBumper = {TeleopControlMappingEnums::DRIVER, TeleopControlMappingEnums::RIGHT_BUMPER, TeleopControlMappingEnums::STANDARD};
        const TeleopControlButton driverDPad0   = {TeleopControlMappingEnums::DRIVER, TeleopControlMappingEnums::POV_0, TeleopControlMappingEnums::STANDARD};
        const TeleopControlButton driverDPad90  = {TeleopControlMappingEnums::DRIVER, TeleopControlMappingEnums::POV_90, TeleopControlMappingEnums::STANDARD};
        const TeleopControlButton driverDPad180 = {TeleopControlMappingEnums::DRIVER, TeleopControlMappingEnums::POV_180, TeleopControlMappingEnums::STANDARD};
        const TeleopControlButton driverDPad270 = {TeleopControlMappingEnums::DRIVER, TeleopControlMappingEnums::POV_270, TeleopControlMappingEnums::STANDARD};

        const TeleopControlButton copilotAButton = {TeleopControlMappingEnums::CO_PILOT, TeleopControlMappingEnums::A_BUTTON, TeleopControlMappingEnums::STANDARD};
        const TeleopControlButton copilotBButton = {TeleopControlMappingEnums::CO_PILOT, TeleopControlMappingEnums::B_BUTTON, TeleopControlMappingEnums::STANDARD};
        const TeleopControlButton copilotXButton = {TeleopControlMappingEnums::CO_PILOT, TeleopControlMappingEnums::X_BUTTON, TeleopControlMappingEnums::STANDARD};
        const TeleopControlButton copilotYButton = {TeleopControlMappingEnums::CO_PILOT, TeleopControlMappingEnums::Y_BUTTON, TeleopControlMappingEnums::STANDARD};
        const TeleopControlButton copil9tLBumper = {TeleopControlMappingEnums::CO_PILOT, TeleopControlMappingEnums::LEFT_BUMPER, TeleopControlMappingEnums::STANDARD};
        const TeleopControlButton copilotRBumper = {TeleopControlMappingEnums::CO_PILOT, TeleopControlMappingEnums::RIGHT_BUMPER, TeleopControlMappingEnums::STANDARD};
        const TeleopControlButton copilotDPad0   = {TeleopControlMappingEnums::CO_PILOT, TeleopControlMappingEnums::POV_0, TeleopControlMappingEnums::STANDARD};
        const TeleopControlButton copilotDPad90  = {TeleopControlMappingEnums::CO_PILOT, TeleopControlMappingEnums::POV_90, TeleopControlMappingEnums::STANDARD};
        const TeleopControlButton copilotDPad180 = {TeleopControlMappingEnums::CO_PILOT, TeleopControlMappingEnums::POV_180, TeleopControlMappingEnums::STANDARD};
        const TeleopControlButton copilotDPad270 = {TeleopControlMappingEnums::CO_PILOT, TeleopControlMappingEnums::POV_270, TeleopControlMappingEnums::STANDARD};

        const TeleopControlButton extra1AButton = {TeleopControlMappingEnums::EXTRA1 , TeleopControlMappingEnums::A_BUTTON, TeleopControlMappingEnums::STANDARD};
        const TeleopControlButton extra1BButton = {TeleopControlMappingEnums::EXTRA1 , TeleopControlMappingEnums::B_BUTTON, TeleopControlMappingEnums::STANDARD};
        const TeleopControlButton extra1XButton = {TeleopControlMappingEnums::EXTRA1 , TeleopControlMappingEnums::X_BUTTON, TeleopControlMappingEnums::STANDARD};
        const TeleopControlButton extra1YButton = {TeleopControlMappingEnums::EXTRA1 , TeleopControlMappingEnums::Y_BUTTON, TeleopControlMappingEnums::STANDARD};
        const TeleopControlButton extra1DPad0   = {TeleopControlMappingEnums::EXTRA1 , TeleopControlMappingEnums::POV_0, TeleopControlMappingEnums::STANDARD};
        const TeleopControlButton extra1DPad90  = {TeleopControlMappingEnums::EXTRA1 , TeleopControlMappingEnums::POV_90, TeleopControlMappingEnums::STANDARD};
        const TeleopControlButton extra1DPad180 = {TeleopControlMappingEnums::EXTRA1 , TeleopControlMappingEnums::POV_180, TeleopControlMappingEnums::STANDARD};
        const TeleopControlButton extra1DPad270 = {TeleopControlMappingEnums::EXTRA1 , TeleopControlMappingEnums::POV_270, TeleopControlMappingEnums::STANDARD};

        const TeleopControlButton extra2AButton = {TeleopControlMappingEnums::EXTRA2 , TeleopControlMappingEnums::A_BUTTON, TeleopControlMappingEnums::STANDARD};
        const TeleopControlButton extra2BButton = {TeleopControlMappingEnums::EXTRA2 , TeleopControlMappingEnums::B_BUTTON, TeleopControlMappingEnums::STANDARD};
        const TeleopControlButton extra2XButton = {TeleopControlMappingEnums::EXTRA2 , TeleopControlMappingEnums::X_BUTTON, TeleopControlMappingEnums::STANDARD};
        const TeleopControlButton extra2YButton = {TeleopControlMappingEnums::EXTRA2 , TeleopControlMappingEnums::Y_BUTTON, TeleopControlMappingEnums::STANDARD};
        const TeleopControlButton extra2DPad0   = {TeleopControlMappingEnums::EXTRA2 , TeleopControlMappingEnums::POV_0, TeleopControlMappingEnums::STANDARD};
        const TeleopControlButton extra2DPad90  = {TeleopControlMappingEnums::EXTRA2 , TeleopControlMappingEnums::POV_90, TeleopControlMappingEnums::STANDARD};
        const TeleopControlButton extra2DPad180 = {TeleopControlMappingEnums::EXTRA2 , TeleopControlMappingEnums::POV_180, TeleopControlMappingEnums::STANDARD};
        const TeleopControlButton extra2DPad270 = {TeleopControlMappingEnums::EXTRA2 , TeleopControlMappingEnums::POV_270, TeleopControlMappingEnums::STANDARD};

        const TeleopControlButton extra3AButton = {TeleopControlMappingEnums::EXTRA2 , TeleopControlMappingEnums::A_BUTTON, TeleopControlMappingEnums::STANDARD};
        const TeleopControlButton extra3BButton = {TeleopControlMappingEnums::EXTRA3 , TeleopControlMappingEnums::B_BUTTON, TeleopControlMappingEnums::STANDARD};
        const TeleopControlButton extra3XButton = {TeleopControlMappingEnums::EXTRA2 , TeleopControlMappingEnums::X_BUTTON, TeleopControlMappingEnums::STANDARD};
        const TeleopControlButton extra3YButton = {TeleopControlMappingEnums::EXTRA3 , TeleopControlMappingEnums::Y_BUTTON, TeleopControlMappingEnums::STANDARD};
        const TeleopControlButton extra3DPad0   = {TeleopControlMappingEnums::EXTRA3 , TeleopControlMappingEnums::POV_0, TeleopControlMappingEnums::STANDARD};
        const TeleopControlButton extra3DPad90  = {TeleopControlMappingEnums::EXTRA3 , TeleopControlMappingEnums::POV_90, TeleopControlMappingEnums::STANDARD};
        const TeleopControlButton extra3DPad180 = {TeleopControlMappingEnums::EXTRA3 , TeleopControlMappingEnums::POV_180, TeleopControlMappingEnums::STANDARD};
        const TeleopControlButton extra3DPad270 = {TeleopControlMappingEnums::EXTRA3 , TeleopControlMappingEnums::POV_270, TeleopControlMappingEnums::STANDARD};

        const TeleopControlButton extra4AButton = {TeleopControlMappingEnums::EXTRA4 , TeleopControlMappingEnums::A_BUTTON, TeleopControlMappingEnums::STANDARD};
        const TeleopControlButton extra4BButton = {TeleopControlMappingEnums::EXTRA4 , TeleopControlMappingEnums::B_BUTTON, TeleopControlMappingEnums::STANDARD};
        const TeleopControlButton extra4XButton = {TeleopControlMappingEnums::EXTRA4 , TeleopControlMappingEnums::X_BUTTON, TeleopControlMappingEnums::STANDARD};
        const TeleopControlButton extra4YButton = {TeleopControlMappingEnums::EXTRA4 , TeleopControlMappingEnums::Y_BUTTON, TeleopControlMappingEnums::STANDARD};
        const TeleopControlButton extra4DPad0   = {TeleopControlMappingEnums::EXTRA4 , TeleopControlMappingEnums::POV_0, TeleopControlMappingEnums::STANDARD};
        const TeleopControlButton extra4DPad90  = {TeleopControlMappingEnums::EXTRA4 , TeleopControlMappingEnums::POV_90, TeleopControlMappingEnums::STANDARD};
        const TeleopControlButton extra4DPad180 = {TeleopControlMappingEnums::EXTRA4 , TeleopControlMappingEnums::POV_180, TeleopControlMappingEnums::STANDARD};
        const TeleopControlButton extra4DPad270 = {TeleopControlMappingEnums::EXTRA4 , TeleopControlMappingEnums::POV_270, TeleopControlMappingEnums::STANDARD};


        robin_hood::unordered_map<TeleopControlFunctions::FUNCTION, const TeleopControlButton> buttonMap
        {
            {TeleopControlFunctions::FUNCTION::FINDTARGET, driverLBumper},
            {TeleopControlFunctions::FUNCTION::HOLONOMIC_ROTATE_FRONT, driverDPad0},
            {TeleopControlFunctions::FUNCTION::HOLONOMIC_ROTATE_BACK, driverDPad180},
            {TeleopControlFunctions::FUNCTION::HOLONOMIC_ROTATE_LEFT, driverDPad270},
            {TeleopControlFunctions::FUNCTION::HOLONOMIC_ROTATE_RIGHT, driverDPad90},
            {TeleopControlFunctions::FUNCTION::DRIVE_TO_COL_ONE, driverXButton},
            {TeleopControlFunctions::FUNCTION::DRIVE_TO_COL_TWO, driverYButton},
            {TeleopControlFunctions::FUNCTION::DRIVE_TO_COL_THREE, driverBButton},
            {TeleopControlFunctions::FUNCTION::REZERO_PIGEON, driverAButton},
            {TeleopControlFunctions::FUNCTION::HOLD_POSITION, driverRBumper},

            {TeleopControlFunctions::FUNCTION::HOLD_POSITION_ROTATE, extra1AButton},
            {TeleopControlFunctions::FUNCTION::CUBE_BACKROW_ROTATE, extra1XButton},
            {TeleopControlFunctions::FUNCTION::CUBE_MIDROW_ROTATE, extra1BButton},
            {TeleopControlFunctions::FUNCTION::CONE_BACKROW_ROTATE, extra1YButton},
            {TeleopControlFunctions::FUNCTION::CONE_MIDROW_ROTATE, extra1DPad0},
            {TeleopControlFunctions::FUNCTION::HUMAN_PLAYER_STATION_ROTATE, extra1DPad90},
            {TeleopControlFunctions::FUNCTION::FLOOR_POSITION_ROTATE, extra1DPad270},
            {TeleopControlFunctions::FUNCTION::STARTING_POSITION_ROTATE, extra1DPad180},

            {TeleopControlFunctions::FUNCTION::HOLD_POSITION_EXTEND, extra2AButton},
            {TeleopControlFunctions::FUNCTION::CUBE_BACKROW_EXTEND, extra2XButton},
            {TeleopControlFunctions::FUNCTION::CUBE_MIDROW_EXTEND, extra2BButton},
            {TeleopControlFunctions::FUNCTION::CONE_BACKROW_EXTEND, extra2YButton},
            {TeleopControlFunctions::FUNCTION::CONE_MIDROW_EXTEND, extra2DPad0},
            {TeleopControlFunctions::FUNCTION::HUMAN_PLAYER_STATION_EXTEND, extra2DPad90},
            {TeleopControlFunctions::FUNCTION::FLOOR_EXTEND, extra2DPad270},
            {TeleopControlFunctions::FUNCTION::STARTING_POSITION_EXTEND, extra2DPad180},

            {TeleopControlFunctions::FUNCTION::OPEN, extra3YButton},
            {TeleopControlFunctions::FUNCTION::HOLDING_CUBE, extra3DPad0},
            {TeleopControlFunctions::FUNCTION::HOLDING_CONE, extra3BButton},
            {TeleopControlFunctions::FUNCTION::GRABBING_CUBE, extra3DPad180},
            {TeleopControlFunctions::FUNCTION::GRABBING_CONE , extra3DPad90},
            {TeleopControlFunctions::FUNCTION::RELEASE, extra3DPad270}
        };


        const TeleopControlAxis driverLJoystickX = {TeleopControlMappingEnums::DRIVER, TeleopControlMappingEnums::LEFT_JOYSTICK_X, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::REVERSED, 0.1, 0.6};
        const TeleopControlAxis driverLJoystickY = {TeleopControlMappingEnums::DRIVER, TeleopControlMappingEnums::LEFT_JOYSTICK_Y, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::REVERSED, 0.1, 0.6};
        const TeleopControlAxis driverRJoystickX = {TeleopControlMappingEnums::DRIVER, TeleopControlMappingEnums::RIGHT_JOYSTICK_X, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::SYNCED, 0.1, 0.5};
        const TeleopControlAxis driverRJoystickY = {TeleopControlMappingEnums::DRIVER, TeleopControlMappingEnums::RIGHT_JOYSTICK_X, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::REVERSED, 0.1, 1.0};
        const TeleopControlAxis driverLTrigger   = {TeleopControlMappingEnums::DRIVER, TeleopControlMappingEnums::LEFT_TRIGGER, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::SYNCED, 0.1, 1.0};
        const TeleopControlAxis driverRTrigger   = {TeleopControlMappingEnums::DRIVER, TeleopControlMappingEnums::RIGHT_TRIGGER, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::SYNCED, 0.1, 1.0};

        const TeleopControlAxis copilotLJoystickX = {TeleopControlMappingEnums::CO_PILOT, TeleopControlMappingEnums::LEFT_JOYSTICK_X, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::SYNCED, 0.1, 1.0};
        const TeleopControlAxis copilotLJoystickY = {TeleopControlMappingEnums::CO_PILOT, TeleopControlMappingEnums::LEFT_JOYSTICK_Y, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::REVERSED, 0.1, 1.0};
        const TeleopControlAxis copilotRJoystickX = {TeleopControlMappingEnums::CO_PILOT, TeleopControlMappingEnums::RIGHT_JOYSTICK_X, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::SYNCED, 0.1, 1.0};
        const TeleopControlAxis copilotRJoystickY = {TeleopControlMappingEnums::CO_PILOT, TeleopControlMappingEnums::RIGHT_JOYSTICK_X, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::REVERSED, 0.1, 1.0};
        const TeleopControlAxis copilotLTrigger   = {TeleopControlMappingEnums::CO_PILOT, TeleopControlMappingEnums::LEFT_TRIGGER, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::SYNCED, 0.1, 1.0};
        const TeleopControlAxis copilotRTrigger   = {TeleopControlMappingEnums::CO_PILOT, TeleopControlMappingEnums::RIGHT_TRIGGER, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::SYNCED, 0.1, 1.0};

        const TeleopControlAxis extra1LJoystickX = {TeleopControlMappingEnums::EXTRA1, TeleopControlMappingEnums::LEFT_JOYSTICK_X, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::SYNCED, 0.1, 1.0};
        const TeleopControlAxis extra1LJoystickY = {TeleopControlMappingEnums::EXTRA1, TeleopControlMappingEnums::LEFT_JOYSTICK_Y, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::REVERSED, 0.1, 1.0};
        const TeleopControlAxis extra1RJoystickX = {TeleopControlMappingEnums::EXTRA1, TeleopControlMappingEnums::RIGHT_JOYSTICK_X, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::SYNCED, 0.1, 1.0};
        const TeleopControlAxis extra1RJoystickY = {TeleopControlMappingEnums::EXTRA1, TeleopControlMappingEnums::RIGHT_JOYSTICK_X, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::REVERSED, 0.1, 1.0};
        const TeleopControlAxis extra1LTrigger   = {TeleopControlMappingEnums::EXTRA1, TeleopControlMappingEnums::LEFT_TRIGGER, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::SYNCED, 0.1, 1.0};
        const TeleopControlAxis extra1RTrigger   = {TeleopControlMappingEnums::EXTRA1, TeleopControlMappingEnums::RIGHT_TRIGGER, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::SYNCED, 0.1, 1.0};

        const TeleopControlAxis extra2LJoystickX = {TeleopControlMappingEnums::EXTRA2, TeleopControlMappingEnums::LEFT_JOYSTICK_X, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::SYNCED, 0.1, 1.0};
        const TeleopControlAxis extra2LJoystickY = {TeleopControlMappingEnums::EXTRA2, TeleopControlMappingEnums::LEFT_JOYSTICK_Y, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::REVERSED, 0.1, 1.0};
        const TeleopControlAxis extra2RJoystickX = {TeleopControlMappingEnums::EXTRA2, TeleopControlMappingEnums::RIGHT_JOYSTICK_X, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::SYNCED, 0.1, 1.0};
        const TeleopControlAxis extra2RJoystickY = {TeleopControlMappingEnums::EXTRA2, TeleopControlMappingEnums::RIGHT_JOYSTICK_X, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::REVERSED, 0.1, 1.0};
        const TeleopControlAxis extra2LTrigger   = {TeleopControlMappingEnums::EXTRA2, TeleopControlMappingEnums::LEFT_TRIGGER, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::SYNCED, 0.1, 1.0};
        const TeleopControlAxis extra2RTrigger   = {TeleopControlMappingEnums::EXTRA2, TeleopControlMappingEnums::RIGHT_TRIGGER, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::SYNCED, 0.1, 1.0};

        const TeleopControlAxis extra3LJoystickX = {TeleopControlMappingEnums::EXTRA3, TeleopControlMappingEnums::LEFT_JOYSTICK_X, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::SYNCED, 0.1, 1.0};
        const TeleopControlAxis extra3LJoystickY = {TeleopControlMappingEnums::EXTRA3, TeleopControlMappingEnums::LEFT_JOYSTICK_Y, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::REVERSED, 0.1, 1.0};
        const TeleopControlAxis extra3RJoystickX = {TeleopControlMappingEnums::EXTRA3, TeleopControlMappingEnums::RIGHT_JOYSTICK_X, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::SYNCED, 0.1, 1.0};
        const TeleopControlAxis extra3RJoystickY = {TeleopControlMappingEnums::EXTRA3, TeleopControlMappingEnums::RIGHT_JOYSTICK_X, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::REVERSED, 0.1, 1.0};
        const TeleopControlAxis extra3LTrigger   = {TeleopControlMappingEnums::EXTRA3, TeleopControlMappingEnums::LEFT_TRIGGER, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::SYNCED, 0.1, 1.0};
        const TeleopControlAxis extra3RTrigger   = {TeleopControlMappingEnums::EXTRA3, TeleopControlMappingEnums::RIGHT_TRIGGER, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::SYNCED, 0.1, 1.0};

        const TeleopControlAxis extra4LJoystickX = {TeleopControlMappingEnums::EXTRA4, TeleopControlMappingEnums::LEFT_JOYSTICK_X, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::SYNCED, 0.1, 1.0};
        const TeleopControlAxis extra4LJoystickY = {TeleopControlMappingEnums::EXTRA4, TeleopControlMappingEnums::LEFT_JOYSTICK_Y, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::REVERSED, 0.1, 1.0};
        const TeleopControlAxis extra4RJoystickX = {TeleopControlMappingEnums::EXTRA4, TeleopControlMappingEnums::RIGHT_JOYSTICK_X, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::SYNCED, 0.1, 1.0};
        const TeleopControlAxis extra4RJoystickY = {TeleopControlMappingEnums::EXTRA4, TeleopControlMappingEnums::RIGHT_JOYSTICK_X, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::REVERSED, 0.1, 1.0};
        const TeleopControlAxis extra4LTrigger   = {TeleopControlMappingEnums::EXTRA4, TeleopControlMappingEnums::LEFT_TRIGGER, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::SYNCED, 0.1, 1.0};
        const TeleopControlAxis extra4RTrigger   = {TeleopControlMappingEnums::EXTRA4, TeleopControlMappingEnums::RIGHT_TRIGGER, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::SYNCED, 0.1, 1.0};

        robin_hood::unordered_map<TeleopControlFunctions::FUNCTION, const TeleopControlAxis> axisMap
        {
            {TeleopControlFunctions::FUNCTION::HOLONOMIC_DRIVE_FORWARD, driverLJoystickY},
            {TeleopControlFunctions::FUNCTION::HOLONOMIC_DRIVE_STRAFE, driverLJoystickX},
            {TeleopControlFunctions::FUNCTION::HOLONOMIC_DRIVE_ROTATE, driverRJoystickX},
            {TeleopControlFunctions::FUNCTION::MANUAL_ROTATE, extra1LJoystickY},
            {TeleopControlFunctions::FUNCTION::MANUAL_EXTEND_RETRACT, extra2LJoystickY}
        };



};
