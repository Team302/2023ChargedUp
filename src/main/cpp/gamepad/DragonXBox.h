

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

// #include <map>

// FRC includes
#include <frc/XboxController.h>

// Team 302 includes
#include <gamepad/IDragonGamePad.h>
#include <teleopcontrol/TeleopControlMappingEnums.h>

// forward declares
class AnalogAxis;
class IButton;

class DragonXBox : public IDragonGamePad
{
public:
    DragonXBox(
        int port);

    ~DragonXBox();

    // getters
    ///-------------------------------------------------------------------------------------------------
    /// Method:      GetAxisValue
    /// Description: Return the current value (between -1.0 and 1.0) for the requested axis.
    /// Returns:     double   - current axis value
    ///-------------------------------------------------------------------------------------------------
    double GetAxisValue(
        TeleopControlMappingEnums::AXIS_IDENTIFIER axis // <I> - axis identifier to read
    ) const override;

    ///-------------------------------------------------------------------------------------------------
    /// Method:      IsButtonPressed
    /// Description: Return whether the requested button is selected (true) or not (false)
    /// Returns:     bool    true  - button is pressed
    ///                      false - button is not pressed
    ///-------------------------------------------------------------------------------------------------
    bool IsButtonPressed(
        TeleopControlMappingEnums::BUTTON_IDENTIFIER button // <I> - button to check
    ) const override;

    //==================================================================================
    /// <summary>
    /// Method:         WasButtonReleased
    /// Description:    Read whether the button was released since the last query.  This
    ///                 is only valid for digital buttons (normal buttons and bumpers).
    /// </summary>
    //==================================================================================
    bool WasButtonReleased(
        TeleopControlMappingEnums::BUTTON_IDENTIFIER button // <I> - button to check
    ) const override;

    //==================================================================================
    /// <summary>
    /// Method:         WasButtonPressed
    /// Description:    Read whether the button was pressed since the last query.  This
    ///                 is only valid for digital buttons (normal buttons and bumpers).
    /// </summary>
    //==================================================================================
    bool WasButtonPressed(
        TeleopControlMappingEnums::BUTTON_IDENTIFIER button // <I> - button to check
    ) const override;

    // setters

    //==================================================================================
    /// <summary>
    /// Method:         SetAxisDeadband
    /// Description:    Specify what deadband behavior is desired such as none, standard,
    ///                 standard with scaling.
    /// Returns:        void
    /// </summary>
    //==================================================================================
    void SetAxisDeadband(
        TeleopControlMappingEnums::AXIS_IDENTIFIER axis, /// <I> - axis to modify
        TeleopControlMappingEnums::AXIS_DEADBAND type    /// <I> - deadband option
        ) override;

    ///-------------------------------------------------------------------------------------------------
    /// Method:      SetProfile
    /// Description: Specify the profile curve used for setting the sensitivity of the axis.  By default,
    ///              this is linear, but a cubic curve would give more control when the axis is barely
    ///              pressed (e.g. if it were moved 50% forward, instead of returning 0.5, it would
    ///              return 0.5 * 0.5 * 0.5 or .125, but when the axis was moved all the way forward,
    ///              it would return the same value -- 1.0.  Since it is cubed, it retains the sign.
    ///
    ///              This affects values returned from GetAxis calls.
    /// Returns:     void
    ///-------------------------------------------------------------------------------------------------
    void SetAxisProfile(
        TeleopControlMappingEnums::AXIS_IDENTIFIER axis, // <I> - axis identifier to modify
        TeleopControlMappingEnums::AXIS_PROFILE curve    // <I> - the definition of the sensitivity
        ) override;

    ///-------------------------------------------------------------------------------------------------
    /// Method:      SetScale
    /// Description: Scale the returned value to a range between the specified negative scale factor and
    ///              the scale factor.  This is used to reduce the maximum value returned.
    ///
    ///              This affects values returned from GetAxis calls.
    /// Returns:     void
    ///-------------------------------------------------------------------------------------------------
    void SetAxisScale(
        TeleopControlMappingEnums::AXIS_IDENTIFIER axis, // <I> - axis identifier to modify
        double scaleFactor                               // <I> - value  (0 < scale <= 1.0) to scale the axis value
        ) override;

    void SetAxisFlipped(
        TeleopControlMappingEnums::AXIS_IDENTIFIER axis, // <I> - axis identifier to modify
        bool isFlipped                                   // <I> - true - invert axis, false - no inversion
        ) override;

    //==================================================================================
    /// <summary>
    /// Method:         SetButtonMode
    /// Description:    Specify how the button should behave.  Examples include (but
    ///                 not limited to):
    ///                 - pressed / not pressed
    ///                 - toggle
    /// Returns:        void
    /// </summary>
    //==================================================================================
    void SetButtonMode(
        TeleopControlMappingEnums::BUTTON_IDENTIFIER button, /// <I> - button to check
        TeleopControlMappingEnums::BUTTON_MODE mode          /// <I> - button behavior
        ) override;

    void SetRumble(
        bool leftRumble, // <I> - rumble left
        bool rightRumble // <I> - rumble right
    ) const override;

private:
    frc::XboxController *m_xbox;
    AnalogAxis *m_axis[TeleopControlMappingEnums::MAX_AXIS];
    IButton *m_button[TeleopControlMappingEnums::MAX_BUTTONS];

    DragonXBox() = delete;
};
