
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

struct TeleopControlAxis
{
    IDragonGamePad::CONTROLLER          controllerNumber=IDragonGamePad::CONTROLLER::UNKNOWN_CONTROLLER;
    IDragonGamePad::CONTROL_MODE        mode=IDragonGamePad::CONTROL_MODE::ALL;
    IDragonGamePad::AXIS_IDENTIFIER     axisId=IDragonGamePad::AXIS_IDENTIFIER::UNDEFINED_AXIS;
    IDragonGamePad::AXIS_DEADBAND       deadbandType=IDragonGamePad::AXIS_DEADBAND::NONE;
    IDragonGamePad::AXIS_PROFILE        profile=IDragonGamePad::AXIS_PROFILE::CUBED;
    IDragonGamePad::AXIS_DIRECTION      direction=IDragonGamePad::AXIS_DIRECTION::SYNCED;
    double                              deadband=0.1;
    double                              scaleFactor=1.0;
};


