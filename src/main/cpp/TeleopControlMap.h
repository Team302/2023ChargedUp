
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

        const TeleopControlButton findTarget = {IDragonGamePad::CONTROLLER::DRIVER, IDragonGamePad::CONTROL_MODE::ALL, IDragonGamePad::BUTTON_IDENTIFIER::LEFT_BUMPER, IDragonGamePad::BUTTON_MODE::STANDARD};
        const TeleopControlButton rotateFront = {IDragonGamePad::CONTROLLER::DRIVER, IDragonGamePad::CONTROL_MODE::ALL, IDragonGamePad::BUTTON_IDENTIFIER::POV_0, IDragonGamePad::BUTTON_MODE::STANDARD};
        const TeleopControlButton rotateBack {IDragonGamePad::CONTROLLER::DRIVER, IDragonGamePad::CONTROL_MODE::ALL, IDragonGamePad::BUTTON_IDENTIFIER::POV_180, IDragonGamePad::BUTTON_MODE::STANDARD};
        const TeleopControlButton rotateLeft = {IDragonGamePad::CONTROLLER::DRIVER, IDragonGamePad::CONTROL_MODE::ALL, IDragonGamePad::BUTTON_IDENTIFIER::POV_270, IDragonGamePad::BUTTON_MODE::STANDARD};
        const TeleopControlButton rotateRight = {IDragonGamePad::CONTROLLER::DRIVER, IDragonGamePad::CONTROL_MODE::ALL, IDragonGamePad::BUTTON_IDENTIFIER::POV_90, IDragonGamePad::BUTTON_MODE::STANDARD};
        const TeleopControlButton driveToShootingSpot = {IDragonGamePad::CONTROLLER::DRIVER, IDragonGamePad::CONTROL_MODE::ALL, IDragonGamePad::BUTTON_IDENTIFIER::A_BUTTON, IDragonGamePad::BUTTON_MODE::STANDARD};
        const TeleopControlButton rezeroPigeon = {IDragonGamePad::CONTROLLER::DRIVER, IDragonGamePad::CONTROL_MODE::ALL, IDragonGamePad::BUTTON_IDENTIFIER::B_BUTTON, IDragonGamePad::BUTTON_MODE::STANDARD};
        const TeleopControlButton holdPosition = {IDragonGamePad::CONTROLLER::DRIVER, IDragonGamePad::CONTROL_MODE::ALL, IDragonGamePad::BUTTON_IDENTIFIER::X_BUTTON, IDragonGamePad::BUTTON_MODE::STANDARD};
        const TeleopControlButton exampleForward = {IDragonGamePad::CONTROLLER::EXTRA1, IDragonGamePad::CONTROL_MODE::ALL, IDragonGamePad::BUTTON_IDENTIFIER::A_BUTTON, IDragonGamePad::BUTTON_MODE::STANDARD};
        const TeleopControlButton exampleReverse = {IDragonGamePad::CONTROLLER::EXTRA1, IDragonGamePad::CONTROL_MODE::ALL, IDragonGamePad::BUTTON_IDENTIFIER::B_BUTTON, IDragonGamePad::BUTTON_MODE::STANDARD};

        robin_hood::unordered_map<TeleopControlFunctions::FUNCTION, const TeleopControlButton> buttonMap
        {
            {TeleopControlFunctions::FUNCTION::FINDTARGET, findTarget},
            {TeleopControlFunctions::FUNCTION::HOLONOMIC_ROTATE_FRONT, rotateFront},
            {TeleopControlFunctions::FUNCTION::HOLONOMIC_ROTATE_BACK, rotateBack},
            {TeleopControlFunctions::FUNCTION::HOLONOMIC_ROTATE_LEFT, rotateLeft},
            {TeleopControlFunctions::FUNCTION::HOLONOMIC_ROTATE_RIGHT, rotateRight},
            {TeleopControlFunctions::FUNCTION::DRIVE_TO_SHOOTING_SPOT, driveToShootingSpot},
            {TeleopControlFunctions::FUNCTION::REZERO_PIGEON, rezeroPigeon},
            {TeleopControlFunctions::FUNCTION::HOLD_POSITION, holdPosition},
            {TeleopControlFunctions::FUNCTION::EXAMPLE_FORWARD, exampleForward},
            {TeleopControlFunctions::FUNCTION::EXAMPLE_REVERSE, exampleReverse}
        };

        const TeleopControlAxis arcadeDrive = {IDragonGamePad::CONTROLLER::DRIVER, IDragonGamePad::CONTROL_MODE::ALL, IDragonGamePad::AXIS_IDENTIFIER::LEFT_JOYSTICK_Y, IDragonGamePad::AXIS_DEADBAND::APPLY_STANDARD_DEADBAND, IDragonGamePad::AXIS_PROFILE::CUBED, IDragonGamePad::AXIS_DIRECTION::REVERSED, 0.1, 1.0};
        const TeleopControlAxis arcadeSteer = {IDragonGamePad::CONTROLLER::DRIVER, IDragonGamePad::CONTROL_MODE::ALL, IDragonGamePad::AXIS_IDENTIFIER::RIGHT_JOYSTICK_X, IDragonGamePad::AXIS_DEADBAND::APPLY_STANDARD_DEADBAND, IDragonGamePad::AXIS_PROFILE::CUBED, IDragonGamePad::AXIS_DIRECTION::REVERSED, 0.1, 1.0};
        const TeleopControlAxis holonomicForward = {IDragonGamePad::CONTROLLER::DRIVER, IDragonGamePad::CONTROL_MODE::ALL, IDragonGamePad::AXIS_IDENTIFIER::LEFT_JOYSTICK_Y, IDragonGamePad::AXIS_DEADBAND::APPLY_STANDARD_DEADBAND, IDragonGamePad::AXIS_PROFILE::CUBED, IDragonGamePad::AXIS_DIRECTION::REVERSED, 0.1, 0.6};
        const TeleopControlAxis holonomicStrafe  = {IDragonGamePad::CONTROLLER::DRIVER, IDragonGamePad::CONTROL_MODE::ALL, IDragonGamePad::AXIS_IDENTIFIER::LEFT_JOYSTICK_X, IDragonGamePad::AXIS_DEADBAND::APPLY_STANDARD_DEADBAND, IDragonGamePad::AXIS_PROFILE::CUBED, IDragonGamePad::AXIS_DIRECTION::REVERSED, 0.1, 0.6};
        const TeleopControlAxis holonomicRotate  = {IDragonGamePad::CONTROLLER::DRIVER, IDragonGamePad::CONTROL_MODE::ALL, IDragonGamePad::AXIS_IDENTIFIER::RIGHT_JOYSTICK_X, IDragonGamePad::AXIS_DEADBAND::APPLY_STANDARD_DEADBAND, IDragonGamePad::AXIS_PROFILE::CUBED, IDragonGamePad::AXIS_DIRECTION::SYNCED, 0.1, 0.5};
        robin_hood::unordered_map<TeleopControlFunctions::FUNCTION, const TeleopControlAxis> axisMap
        {
            {TeleopControlFunctions::FUNCTION::ARCADE_THROTTLE, arcadeDrive},
            {TeleopControlFunctions::FUNCTION::ARCADE_STEER, arcadeSteer},
            {TeleopControlFunctions::FUNCTION::HOLONOMIC_DRIVE_FORWARD, holonomicForward},
            {TeleopControlFunctions::FUNCTION::HOLONOMIC_DRIVE_STRAFE, holonomicStrafe},
            {TeleopControlFunctions::FUNCTION::HOLONOMIC_DRIVE_ROTATE, holonomicRotate}
        };



};
