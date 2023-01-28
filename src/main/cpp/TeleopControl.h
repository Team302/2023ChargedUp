
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
#include <memory>
#include <utility>
#include <vector>

// FRC includes
#include <frc/Driverstation.h>

// Team 302 includes
#include <gamepad/IDragonGamepad.h>

class TeleopControl
{
    public:
// TODO:REMOVE UNUSED IDENTIFIERS ??
        enum FUNCTION_IDENTIFIER
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
            // EXAMPLE_FORWARD,
            // EXAMPLE_REVERSE,
            MAX_FUNCTIONS
        };


        //----------------------------------------------------------------------------------
        // Method:      GetInstance
        // Description: If there isn't an instance of this class, it will create one.  The
        //              single class instance will be returned.
        // Returns:     OperatorInterface*  instance of this class
        //----------------------------------------------------------------------------------
        static TeleopControl* GetInstance();


        //------------------------------------------------------------------
        // Method:      SetScaleFactor
        // Description: Allow the range of values to be set smaller than
        //              -1.0 to 1.0.  By providing a scale factor between 0.0
        //              and 1.0, the range can be made smaller.  If a value
        //              outside the range is provided, then the value will
        //              be set to the closest bounding value (e.g. 1.5 will
        //              become 1.0)
        // Returns:     void
        //------------------------------------------------------------------
        void SetAxisScaleFactor
        (
            TeleopControl::FUNCTION_IDENTIFIER  axis,          // <I> - axis number to update
            double                              scaleFactor    // <I> - scale factor used to limit the range
        );

        void SetDeadBand
        (
            TeleopControl::FUNCTION_IDENTIFIER  axis,
            IDragonGamePad::AXIS_DEADBAND       deadband
        );

        //------------------------------------------------------------------
        // Method:      SetAxisProfile
        // Description: Sets the axis profile for the specifed axis
        // Returns:     void
        //------------------------------------------------------------------
        void SetAxisProfile
        (
            TeleopControl::FUNCTION_IDENTIFIER      axis,       // <I> - axis number to update
			IDragonGamePad::AXIS_PROFILE			profile     // <I> - profile to use
        );

        //------------------------------------------------------------------
        // Method:      GetAxisValue
        // Description: Reads the joystick axis, removes any deadband (small
        //              value) and then scales as requested.
        // Returns:     double   -  scaled axis value
        //------------------------------------------------------------------
        double GetAxisValue
        (
            TeleopControl::FUNCTION_IDENTIFIER     axis // <I> - axis number to update
        ) const;

        //------------------------------------------------------------------
        // Method:      GetRawButton
        // Description: Reads the button value.  Also allows POV, bumpers,
        //              and triggers to be treated as buttons.
        // Returns:     bool   -  scaled axis value
        //------------------------------------------------------------------
        bool IsButtonPressed
        (
            TeleopControl::FUNCTION_IDENTIFIER button   // <I> - button number to query
        ) const;

        void SetRumble
        (
            TeleopControl::FUNCTION_IDENTIFIER  button,         // <I> - controller with this function
            bool                                leftRumble,     // <I> - rumble left
            bool                                rightRumble     // <I> - rumble right
        ) const;

        void SetRumble
        (
            int                                 controller,     // <I> - controller to rumble
            bool                                leftRumble,     // <I> - rumble left
            bool                                rightRumble     // <I> - rumble right
        ) const;

    private:
        //----------------------------------------------------------------------------------
        // Method:      OperatorInterface <<constructor>>
        // Description: This will construct and initialize the object
        //----------------------------------------------------------------------------------
        TeleopControl();

        //----------------------------------------------------------------------------------
        // Method:      ~OperatorInterface <<destructor>>
        // Description: This will clean up the object
        //----------------------------------------------------------------------------------
        virtual ~TeleopControl() = default;

        void Initialize() const;
        bool IsInitialized() const;

        std::pair<IDragonGamePad*, IDragonGamePad::AXIS_IDENTIFIER> GetAxisInfo
        (
            TeleopControl::FUNCTION_IDENTIFIER  function          // <I> - controller with this function
        ) const;

        std::pair<IDragonGamePad*, IDragonGamePad::BUTTON_IDENTIFIER> GetButtonInfo
        (
            TeleopControl::FUNCTION_IDENTIFIER  function          // <I> - controller with this function
        ) const;

        //----------------------------------------------------------------------------------
        // Attributes
        //----------------------------------------------------------------------------------
        static TeleopControl*               m_instance;  // Singleton instance of this class

        mutable std::vector<IDragonGamePad::AXIS_IDENTIFIER>    m_axisIDs;
        mutable std::vector<IDragonGamePad::BUTTON_IDENTIFIER>  m_buttonIDs;
        mutable std::vector<int>							    m_controllerIndex;

        mutable int                                             m_numControllers;

        mutable IDragonGamePad*			                        m_controller[frc::DriverStation::kJoystickPorts];
};

