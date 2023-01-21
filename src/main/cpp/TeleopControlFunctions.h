
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

class TeleopControlFunctions
{
    public:
        enum FUNCTION
        {
            UNKNOWN_FUNCTION,
            ROBOT_ORIENTED_DRIVE,
            HOLONOMIC_DRIVE_FORWARD,
            HOLONOMIC_DRIVE_ROTATE,
            HOLONOMIC_DRIVE_STRAFE,
            HOLONOMIC_ROTATE_FRONT,
            HOLONOMIC_ROTATE_BACK,
            HOLONOMIC_ROTATE_LEFT,
            HOLONOMIC_ROTATE_RIGHT,
            REZERO_PIGEON,
            HOLD_POSITION,
            FINDTARGET,        
            DRIVE_TO_SHOOTING_SPOT,
            ARCADE_THROTTLE,
            ARCADE_STEER,            
		    // @ADDMECH add functions here for robot
            EXAMPLE_FORWARD,
            EXAMPLE_REVERSE,
            MAX_FUNCTIONS
        };
};
