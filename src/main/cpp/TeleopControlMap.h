
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

        const TeleopControlButton findTarget = {TeleopControlMappingEnums::CONTROLLER::DRIVER, TeleopControlMappingEnums::CONTROL_MODE::ALL, TeleopControlMappingEnums::BUTTON_IDENTIFIER::LEFT_BUMPER, TeleopControlMappingEnums::BUTTON_MODE::STANDARD};
        const TeleopControlButton rotateFront = {TeleopControlMappingEnums::CONTROLLER::DRIVER, TeleopControlMappingEnums::CONTROL_MODE::ALL, TeleopControlMappingEnums::BUTTON_IDENTIFIER::POV_0, TeleopControlMappingEnums::BUTTON_MODE::STANDARD};
        const TeleopControlButton rotateBack {TeleopControlMappingEnums::CONTROLLER::DRIVER, TeleopControlMappingEnums::CONTROL_MODE::ALL, TeleopControlMappingEnums::BUTTON_IDENTIFIER::POV_180, TeleopControlMappingEnums::BUTTON_MODE::STANDARD};
        const TeleopControlButton rotateLeft = {TeleopControlMappingEnums::CONTROLLER::DRIVER, TeleopControlMappingEnums::CONTROL_MODE::ALL, TeleopControlMappingEnums::BUTTON_IDENTIFIER::POV_270, TeleopControlMappingEnums::BUTTON_MODE::STANDARD};
        const TeleopControlButton rotateRight = {TeleopControlMappingEnums::CONTROLLER::DRIVER, TeleopControlMappingEnums::CONTROL_MODE::ALL, TeleopControlMappingEnums::BUTTON_IDENTIFIER::POV_90, TeleopControlMappingEnums::BUTTON_MODE::STANDARD};
        const TeleopControlButton driveToShootingSpot = {TeleopControlMappingEnums::CONTROLLER::DRIVER, TeleopControlMappingEnums::CONTROL_MODE::ALL, TeleopControlMappingEnums::BUTTON_IDENTIFIER::A_BUTTON, TeleopControlMappingEnums::BUTTON_MODE::STANDARD};
        const TeleopControlButton rezeroPigeon = {TeleopControlMappingEnums::CONTROLLER::DRIVER, TeleopControlMappingEnums::CONTROL_MODE::ALL, TeleopControlMappingEnums::BUTTON_IDENTIFIER::B_BUTTON, TeleopControlMappingEnums::BUTTON_MODE::STANDARD};
        const TeleopControlButton holdPosition = {TeleopControlMappingEnums::CONTROLLER::DRIVER, TeleopControlMappingEnums::CONTROL_MODE::ALL, TeleopControlMappingEnums::BUTTON_IDENTIFIER::X_BUTTON, TeleopControlMappingEnums::BUTTON_MODE::STANDARD};
        const TeleopControlButton exampleForward = {TeleopControlMappingEnums::CONTROLLER::EXTRA1, TeleopControlMappingEnums::CONTROL_MODE::ALL, TeleopControlMappingEnums::BUTTON_IDENTIFIER::A_BUTTON, TeleopControlMappingEnums::BUTTON_MODE::STANDARD};
        const TeleopControlButton exampleReverse = {TeleopControlMappingEnums::CONTROLLER::EXTRA1, TeleopControlMappingEnums::CONTROL_MODE::ALL, TeleopControlMappingEnums::BUTTON_IDENTIFIER::B_BUTTON, TeleopControlMappingEnums::BUTTON_MODE::STANDARD};

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

        const TeleopControlAxis arcadeDrive = {TeleopControlMappingEnums::CONTROLLER::DRIVER, TeleopControlMappingEnums::CONTROL_MODE::ALL, TeleopControlMappingEnums::AXIS_IDENTIFIER::LEFT_JOYSTICK_Y, TeleopControlMappingEnums::AXIS_DEADBAND::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::AXIS_PROFILE::CUBED, TeleopControlMappingEnums::AXIS_DIRECTION::REVERSED, 0.1, 1.0};
        const TeleopControlAxis arcadeSteer = {TeleopControlMappingEnums::CONTROLLER::DRIVER, TeleopControlMappingEnums::CONTROL_MODE::ALL, TeleopControlMappingEnums::AXIS_IDENTIFIER::RIGHT_JOYSTICK_X, TeleopControlMappingEnums::AXIS_DEADBAND::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::AXIS_PROFILE::CUBED, TeleopControlMappingEnums::AXIS_DIRECTION::REVERSED, 0.1, 1.0};
        const TeleopControlAxis holonomicForward = {TeleopControlMappingEnums::CONTROLLER::DRIVER, TeleopControlMappingEnums::CONTROL_MODE::ALL, TeleopControlMappingEnums::AXIS_IDENTIFIER::LEFT_JOYSTICK_Y, TeleopControlMappingEnums::AXIS_DEADBAND::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::AXIS_PROFILE::CUBED, TeleopControlMappingEnums::AXIS_DIRECTION::REVERSED, 0.1, 0.6};
        const TeleopControlAxis holonomicStrafe  = {TeleopControlMappingEnums::CONTROLLER::DRIVER, TeleopControlMappingEnums::CONTROL_MODE::ALL, TeleopControlMappingEnums::AXIS_IDENTIFIER::LEFT_JOYSTICK_X, TeleopControlMappingEnums::AXIS_DEADBAND::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::AXIS_PROFILE::CUBED, TeleopControlMappingEnums::AXIS_DIRECTION::REVERSED, 0.1, 0.6};
        const TeleopControlAxis holonomicRotate  = {TeleopControlMappingEnums::CONTROLLER::DRIVER, TeleopControlMappingEnums::CONTROL_MODE::ALL, TeleopControlMappingEnums::AXIS_IDENTIFIER::RIGHT_JOYSTICK_X, TeleopControlMappingEnums::AXIS_DEADBAND::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::AXIS_PROFILE::CUBED, TeleopControlMappingEnums::AXIS_DIRECTION::SYNCED, 0.1, 0.5};
        robin_hood::unordered_map<TeleopControlFunctions::FUNCTION, const TeleopControlAxis> axisMap
        {
            {TeleopControlFunctions::FUNCTION::ARCADE_THROTTLE, arcadeDrive},
            {TeleopControlFunctions::FUNCTION::ARCADE_STEER, arcadeSteer},
            {TeleopControlFunctions::FUNCTION::HOLONOMIC_DRIVE_FORWARD, holonomicForward},
            {TeleopControlFunctions::FUNCTION::HOLONOMIC_DRIVE_STRAFE, holonomicStrafe},
            {TeleopControlFunctions::FUNCTION::HOLONOMIC_DRIVE_ROTATE, holonomicRotate}
        };



};
