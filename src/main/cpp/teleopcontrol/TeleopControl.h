
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
#include <array>
#include <memory>
#include <utility>
#include <vector>

// FRC includes
#include <frc/Driverstation.h>

// Team 302 includes
#include <gamepad/IDragonGamePad.h>
#include <teleopcontrol/TeleopControlAxis.h>
#include <teleopcontrol/TeleopControlButton.h>
#include <teleopcontrol/TeleopControlFunctions.h>
#include <teleopcontrol/TeleopControlMappingEnums.h>
#include <utils/logging/LoggableItem.h>

// third part
#include <robinHood/robin_hood.h>

class TeleopControl : LoggableItem
{
public:
    //----------------------------------------------------------------------------------
    // Method:      GetInstance
    // Description: If there isn't an instance of this class, it will create one.  The
    //              single class instance will be returned.
    // Returns:     OperatorInterface*  instance of this class
    //----------------------------------------------------------------------------------
    static TeleopControl *GetInstance();

    //------------------------------------------------------------------
    // Method:      GetAxisValue
    // Description: Reads the joystick axis, removes any deadband (small
    //              value) and then scales as requested.
    // Returns:     double   -  scaled axis value
    //------------------------------------------------------------------
    double GetAxisValue(
        TeleopControlFunctions::FUNCTION axis // <I> - axis number to update
    );

    //------------------------------------------------------------------
    // Method:      GetRawButton
    // Description: Reads the button value.  Also allows POV, bumpers,
    //              and triggers to be treated as buttons.
    // Returns:     bool   -  scaled axis value
    //------------------------------------------------------------------
    bool IsButtonPressed(
        TeleopControlFunctions::FUNCTION button // <I> - button number to query
    );

    void SetRumble(
        TeleopControlFunctions::FUNCTION button, // <I> - controller with this function
        bool leftRumble,                         // <I> - rumble left
        bool rightRumble                         // <I> - rumble right
    );

    void SetRumble(
        int controller,  // <I> - controller to rumble
        bool leftRumble, // <I> - rumble left
        bool rightRumble // <I> - rumble right
    );

    void LogInformation() const override;

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

    void Initialize();
    bool IsInitialized() const;

    void InitializeControllers();
    void InitializeController(int port);
    void InitializeAxes(int port);
    void InitializeButtons(int port);

    std::vector<TeleopControlFunctions::FUNCTION> GetAxisFunctionsOnController(int controller);
    std::vector<TeleopControlFunctions::FUNCTION> GetButtonFunctionsOnController(int controller);

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
    void SetAxisScaleFactor(
        TeleopControlFunctions::FUNCTION axis, // <I> - axis number to update
        double scaleFactor                     // <I> - scale factor used to limit the range
    );

    void SetDeadBand(
        TeleopControlFunctions::FUNCTION axis,
        TeleopControlMappingEnums::AXIS_DEADBAND deadband);

    //------------------------------------------------------------------
    // Method:      SetAxisProfile
    // Description: Sets the axis profile for the specifed axis
    // Returns:     void
    //------------------------------------------------------------------
    void SetAxisProfile(
        TeleopControlFunctions::FUNCTION axis,          // <I> - axis number to update
        TeleopControlMappingEnums::AXIS_PROFILE profile // <I> - profile to use
    );

    std::pair<IDragonGamePad *, TeleopControlMappingEnums::AXIS_IDENTIFIER> GetAxisInfo(
        TeleopControlFunctions::FUNCTION function // <I> - controller with this function
    );

    std::pair<IDragonGamePad *, TeleopControlMappingEnums::BUTTON_IDENTIFIER> GetButtonInfo(
        TeleopControlFunctions::FUNCTION function // <I> - controller with this function
    );

    //----------------------------------------------------------------------------------
    // Attributes
    //----------------------------------------------------------------------------------
    static TeleopControl *m_instance; // Singleton instance of this class

    std::array<IDragonGamePad *, frc::DriverStation::kJoystickPorts> m_controller;

    int m_numControllers;

    const TeleopControlButton driverAButton = {TeleopControlMappingEnums::DRIVER, TeleopControlMappingEnums::A_BUTTON, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton driverBButton = {TeleopControlMappingEnums::DRIVER, TeleopControlMappingEnums::B_BUTTON, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton driverXButton = {TeleopControlMappingEnums::DRIVER, TeleopControlMappingEnums::X_BUTTON, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton driverYButton = {TeleopControlMappingEnums::DRIVER, TeleopControlMappingEnums::Y_BUTTON, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton driverLBumper = {TeleopControlMappingEnums::DRIVER, TeleopControlMappingEnums::LEFT_BUMPER, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton driverRBumper = {TeleopControlMappingEnums::DRIVER, TeleopControlMappingEnums::RIGHT_BUMPER, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton driverSelectButton = {TeleopControlMappingEnums::DRIVER, TeleopControlMappingEnums::SELECT_BUTTON, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton driverStartButton = {TeleopControlMappingEnums::DRIVER, TeleopControlMappingEnums::START_BUTTON, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton driverLStickPressed = {TeleopControlMappingEnums::DRIVER, TeleopControlMappingEnums::LEFT_STICK_PRESSED, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton driverRStickPressed = {TeleopControlMappingEnums::DRIVER, TeleopControlMappingEnums::RIGHT_STICK_PRESSED, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton driverLTriggerPressed = {TeleopControlMappingEnums::DRIVER, TeleopControlMappingEnums::LEFT_TRIGGER_PRESSED, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton driverRTriggerPressed = {TeleopControlMappingEnums::DRIVER, TeleopControlMappingEnums::RIGHT_TRIGGER_PRESSED, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton driverDPad0 = {TeleopControlMappingEnums::DRIVER, TeleopControlMappingEnums::POV_0, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton driverDPad45 = {TeleopControlMappingEnums::DRIVER, TeleopControlMappingEnums::POV_45, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton driverDPad90 = {TeleopControlMappingEnums::DRIVER, TeleopControlMappingEnums::POV_90, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton driverDPad135 = {TeleopControlMappingEnums::DRIVER, TeleopControlMappingEnums::POV_135, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton driverDPad180 = {TeleopControlMappingEnums::DRIVER, TeleopControlMappingEnums::POV_180, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton driverDPad225 = {TeleopControlMappingEnums::DRIVER, TeleopControlMappingEnums::POV_225, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton driverDPad270 = {TeleopControlMappingEnums::DRIVER, TeleopControlMappingEnums::POV_270, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton driverDPad315 = {TeleopControlMappingEnums::DRIVER, TeleopControlMappingEnums::POV_315, TeleopControlMappingEnums::STANDARD};

    const TeleopControlButton copilotAButton = {TeleopControlMappingEnums::CO_PILOT, TeleopControlMappingEnums::A_BUTTON, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton copilotBButton = {TeleopControlMappingEnums::CO_PILOT, TeleopControlMappingEnums::B_BUTTON, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton copilotXButton = {TeleopControlMappingEnums::CO_PILOT, TeleopControlMappingEnums::X_BUTTON, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton copilotYButton = {TeleopControlMappingEnums::CO_PILOT, TeleopControlMappingEnums::Y_BUTTON, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton copilotLBumper = {TeleopControlMappingEnums::CO_PILOT, TeleopControlMappingEnums::LEFT_BUMPER, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton copilotRBumper = {TeleopControlMappingEnums::CO_PILOT, TeleopControlMappingEnums::RIGHT_BUMPER, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton copilotSelectButton = {TeleopControlMappingEnums::CO_PILOT, TeleopControlMappingEnums::SELECT_BUTTON, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton copilotStartButton = {TeleopControlMappingEnums::CO_PILOT, TeleopControlMappingEnums::START_BUTTON, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton copilotLStickPressed = {TeleopControlMappingEnums::CO_PILOT, TeleopControlMappingEnums::LEFT_STICK_PRESSED, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton copilotRStickPressed = {TeleopControlMappingEnums::CO_PILOT, TeleopControlMappingEnums::RIGHT_STICK_PRESSED, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton copilotLTriggerPressed = {TeleopControlMappingEnums::CO_PILOT, TeleopControlMappingEnums::LEFT_TRIGGER_PRESSED, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton copilotRTriggerPressed = {TeleopControlMappingEnums::CO_PILOT, TeleopControlMappingEnums::RIGHT_TRIGGER_PRESSED, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton copilotDPad0 = {TeleopControlMappingEnums::CO_PILOT, TeleopControlMappingEnums::POV_0, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton copilotDPad45 = {TeleopControlMappingEnums::CO_PILOT, TeleopControlMappingEnums::POV_45, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton copilotDPad90 = {TeleopControlMappingEnums::CO_PILOT, TeleopControlMappingEnums::POV_90, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton copilotDPad135 = {TeleopControlMappingEnums::CO_PILOT, TeleopControlMappingEnums::POV_135, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton copilotDPad180 = {TeleopControlMappingEnums::CO_PILOT, TeleopControlMappingEnums::POV_180, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton copilotDPad225 = {TeleopControlMappingEnums::CO_PILOT, TeleopControlMappingEnums::POV_225, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton copilotDPad270 = {TeleopControlMappingEnums::CO_PILOT, TeleopControlMappingEnums::POV_270, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton copilotDPad315 = {TeleopControlMappingEnums::CO_PILOT, TeleopControlMappingEnums::POV_315, TeleopControlMappingEnums::STANDARD};

    const TeleopControlButton extra1AButton = {TeleopControlMappingEnums::EXTRA1, TeleopControlMappingEnums::A_BUTTON, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton extra1BButton = {TeleopControlMappingEnums::EXTRA1, TeleopControlMappingEnums::B_BUTTON, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton extra1XButton = {TeleopControlMappingEnums::EXTRA1, TeleopControlMappingEnums::X_BUTTON, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton extra1YButton = {TeleopControlMappingEnums::EXTRA1, TeleopControlMappingEnums::Y_BUTTON, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton extra1LBumper = {TeleopControlMappingEnums::EXTRA1, TeleopControlMappingEnums::LEFT_BUMPER, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton extra1RBumper = {TeleopControlMappingEnums::EXTRA1, TeleopControlMappingEnums::RIGHT_BUMPER, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton extra1SelectButton = {TeleopControlMappingEnums::EXTRA1, TeleopControlMappingEnums::SELECT_BUTTON, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton extra1StartButton = {TeleopControlMappingEnums::EXTRA1, TeleopControlMappingEnums::START_BUTTON, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton extra1LStickPressed = {TeleopControlMappingEnums::EXTRA1, TeleopControlMappingEnums::LEFT_STICK_PRESSED, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton extra1RStickPressed = {TeleopControlMappingEnums::EXTRA1, TeleopControlMappingEnums::RIGHT_STICK_PRESSED, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton extra1LTriggerPressed = {TeleopControlMappingEnums::EXTRA1, TeleopControlMappingEnums::LEFT_TRIGGER_PRESSED, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton extra1RTriggerPressed = {TeleopControlMappingEnums::EXTRA1, TeleopControlMappingEnums::RIGHT_TRIGGER_PRESSED, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton extra1DPad0 = {TeleopControlMappingEnums::EXTRA1, TeleopControlMappingEnums::POV_0, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton extra1DPad45 = {TeleopControlMappingEnums::EXTRA1, TeleopControlMappingEnums::POV_45, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton extra1DPad90 = {TeleopControlMappingEnums::EXTRA1, TeleopControlMappingEnums::POV_90, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton extra1DPad135 = {TeleopControlMappingEnums::EXTRA1, TeleopControlMappingEnums::POV_135, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton extra1DPad180 = {TeleopControlMappingEnums::EXTRA1, TeleopControlMappingEnums::POV_180, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton extra1DPad225 = {TeleopControlMappingEnums::EXTRA1, TeleopControlMappingEnums::POV_225, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton extra1DPad270 = {TeleopControlMappingEnums::EXTRA1, TeleopControlMappingEnums::POV_270, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton extra1DPad315 = {TeleopControlMappingEnums::EXTRA1, TeleopControlMappingEnums::POV_315, TeleopControlMappingEnums::STANDARD};

    const TeleopControlButton extra2AButton = {TeleopControlMappingEnums::EXTRA2, TeleopControlMappingEnums::A_BUTTON, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton extra2BButton = {TeleopControlMappingEnums::EXTRA2, TeleopControlMappingEnums::B_BUTTON, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton extra2XButton = {TeleopControlMappingEnums::EXTRA2, TeleopControlMappingEnums::X_BUTTON, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton extra2YButton = {TeleopControlMappingEnums::EXTRA2, TeleopControlMappingEnums::Y_BUTTON, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton extra2LBumper = {TeleopControlMappingEnums::EXTRA2, TeleopControlMappingEnums::LEFT_BUMPER, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton extra2RBumper = {TeleopControlMappingEnums::EXTRA2, TeleopControlMappingEnums::RIGHT_BUMPER, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton extra2SelectButton = {TeleopControlMappingEnums::EXTRA2, TeleopControlMappingEnums::SELECT_BUTTON, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton extra2StartButton = {TeleopControlMappingEnums::EXTRA2, TeleopControlMappingEnums::START_BUTTON, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton extra2LStickPressed = {TeleopControlMappingEnums::EXTRA2, TeleopControlMappingEnums::LEFT_STICK_PRESSED, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton extra2RStickPressed = {TeleopControlMappingEnums::EXTRA2, TeleopControlMappingEnums::RIGHT_STICK_PRESSED, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton extra2LTriggerPressed = {TeleopControlMappingEnums::EXTRA2, TeleopControlMappingEnums::LEFT_TRIGGER_PRESSED, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton extra2RTriggerPressed = {TeleopControlMappingEnums::EXTRA2, TeleopControlMappingEnums::RIGHT_TRIGGER_PRESSED, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton extra2DPad0 = {TeleopControlMappingEnums::EXTRA2, TeleopControlMappingEnums::POV_0, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton extra2DPad45 = {TeleopControlMappingEnums::EXTRA2, TeleopControlMappingEnums::POV_45, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton extra2DPad90 = {TeleopControlMappingEnums::EXTRA2, TeleopControlMappingEnums::POV_90, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton extra2DPad135 = {TeleopControlMappingEnums::EXTRA2, TeleopControlMappingEnums::POV_135, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton extra2DPad180 = {TeleopControlMappingEnums::EXTRA2, TeleopControlMappingEnums::POV_180, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton extra2DPad225 = {TeleopControlMappingEnums::EXTRA2, TeleopControlMappingEnums::POV_225, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton extra2DPad270 = {TeleopControlMappingEnums::EXTRA2, TeleopControlMappingEnums::POV_270, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton extra2DPad315 = {TeleopControlMappingEnums::EXTRA2, TeleopControlMappingEnums::POV_315, TeleopControlMappingEnums::STANDARD};

    const TeleopControlButton extra3AButton = {TeleopControlMappingEnums::EXTRA3, TeleopControlMappingEnums::A_BUTTON, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton extra3BButton = {TeleopControlMappingEnums::EXTRA3, TeleopControlMappingEnums::B_BUTTON, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton extra3XButton = {TeleopControlMappingEnums::EXTRA3, TeleopControlMappingEnums::X_BUTTON, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton extra3YButton = {TeleopControlMappingEnums::EXTRA3, TeleopControlMappingEnums::Y_BUTTON, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton extra3LBumper = {TeleopControlMappingEnums::EXTRA3, TeleopControlMappingEnums::LEFT_BUMPER, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton extra3RBumper = {TeleopControlMappingEnums::EXTRA3, TeleopControlMappingEnums::RIGHT_BUMPER, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton extra3SelectButton = {TeleopControlMappingEnums::EXTRA3, TeleopControlMappingEnums::SELECT_BUTTON, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton extra3StartButton = {TeleopControlMappingEnums::EXTRA3, TeleopControlMappingEnums::START_BUTTON, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton extra3LStickPressed = {TeleopControlMappingEnums::EXTRA3, TeleopControlMappingEnums::LEFT_STICK_PRESSED, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton extra3RStickPressed = {TeleopControlMappingEnums::EXTRA3, TeleopControlMappingEnums::RIGHT_STICK_PRESSED, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton extra3LTriggerPressed = {TeleopControlMappingEnums::EXTRA3, TeleopControlMappingEnums::LEFT_TRIGGER_PRESSED, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton extra3RTriggerPressed = {TeleopControlMappingEnums::EXTRA3, TeleopControlMappingEnums::RIGHT_TRIGGER_PRESSED, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton extra3DPad0 = {TeleopControlMappingEnums::EXTRA3, TeleopControlMappingEnums::POV_0, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton extra3DPad45 = {TeleopControlMappingEnums::EXTRA3, TeleopControlMappingEnums::POV_45, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton extra3DPad90 = {TeleopControlMappingEnums::EXTRA3, TeleopControlMappingEnums::POV_90, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton extra3DPad135 = {TeleopControlMappingEnums::EXTRA3, TeleopControlMappingEnums::POV_135, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton extra3DPad180 = {TeleopControlMappingEnums::EXTRA3, TeleopControlMappingEnums::POV_180, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton extra3DPad225 = {TeleopControlMappingEnums::EXTRA3, TeleopControlMappingEnums::POV_225, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton extra3DPad270 = {TeleopControlMappingEnums::EXTRA3, TeleopControlMappingEnums::POV_270, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton extra3DPad315 = {TeleopControlMappingEnums::EXTRA3, TeleopControlMappingEnums::POV_315, TeleopControlMappingEnums::STANDARD};

    const TeleopControlButton extra4AButton = {TeleopControlMappingEnums::EXTRA4, TeleopControlMappingEnums::A_BUTTON, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton extra4BButton = {TeleopControlMappingEnums::EXTRA4, TeleopControlMappingEnums::B_BUTTON, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton extra4XButton = {TeleopControlMappingEnums::EXTRA4, TeleopControlMappingEnums::X_BUTTON, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton extra4YButton = {TeleopControlMappingEnums::EXTRA4, TeleopControlMappingEnums::Y_BUTTON, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton extra4LBumper = {TeleopControlMappingEnums::EXTRA4, TeleopControlMappingEnums::LEFT_BUMPER, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton extra4RBumper = {TeleopControlMappingEnums::EXTRA4, TeleopControlMappingEnums::RIGHT_BUMPER, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton extra4SelectButton = {TeleopControlMappingEnums::EXTRA4, TeleopControlMappingEnums::SELECT_BUTTON, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton extra4StartButton = {TeleopControlMappingEnums::EXTRA4, TeleopControlMappingEnums::START_BUTTON, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton extra4LStickPressed = {TeleopControlMappingEnums::EXTRA4, TeleopControlMappingEnums::LEFT_STICK_PRESSED, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton extra4RStickPressed = {TeleopControlMappingEnums::EXTRA4, TeleopControlMappingEnums::RIGHT_STICK_PRESSED, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton extra4LTriggerPressed = {TeleopControlMappingEnums::EXTRA4, TeleopControlMappingEnums::LEFT_TRIGGER_PRESSED, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton extra4RTriggerPressed = {TeleopControlMappingEnums::EXTRA4, TeleopControlMappingEnums::RIGHT_TRIGGER_PRESSED, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton extra4DPad0 = {TeleopControlMappingEnums::EXTRA4, TeleopControlMappingEnums::POV_0, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton extra4DPad45 = {TeleopControlMappingEnums::EXTRA4, TeleopControlMappingEnums::POV_45, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton extra4DPad90 = {TeleopControlMappingEnums::EXTRA4, TeleopControlMappingEnums::POV_90, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton extra4DPad135 = {TeleopControlMappingEnums::EXTRA4, TeleopControlMappingEnums::POV_135, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton extra4DPad180 = {TeleopControlMappingEnums::EXTRA4, TeleopControlMappingEnums::POV_180, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton extra4DPad225 = {TeleopControlMappingEnums::EXTRA4, TeleopControlMappingEnums::POV_225, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton extra4DPad270 = {TeleopControlMappingEnums::EXTRA4, TeleopControlMappingEnums::POV_270, TeleopControlMappingEnums::STANDARD};
    const TeleopControlButton extra4DPad315 = {TeleopControlMappingEnums::EXTRA4, TeleopControlMappingEnums::POV_315, TeleopControlMappingEnums::STANDARD};

    const TeleopControlAxis driverLJoystickX = {TeleopControlMappingEnums::DRIVER, TeleopControlMappingEnums::LEFT_JOYSTICK_X, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::REVERSED, 0.6};
    const TeleopControlAxis driverLJoystickY = {TeleopControlMappingEnums::DRIVER, TeleopControlMappingEnums::LEFT_JOYSTICK_Y, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::REVERSED, 0.6};
    const TeleopControlAxis driverRJoystickX = {TeleopControlMappingEnums::DRIVER, TeleopControlMappingEnums::RIGHT_JOYSTICK_X, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::SYNCED, 0.5};
    const TeleopControlAxis driverRJoystickY = {TeleopControlMappingEnums::DRIVER, TeleopControlMappingEnums::RIGHT_JOYSTICK_X, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::REVERSED, 1.0};
    const TeleopControlAxis driverLTrigger = {TeleopControlMappingEnums::DRIVER, TeleopControlMappingEnums::LEFT_TRIGGER, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::SYNCED, 1.0};
    const TeleopControlAxis driverRTrigger = {TeleopControlMappingEnums::DRIVER, TeleopControlMappingEnums::RIGHT_TRIGGER, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::SYNCED, 1.0};

    const TeleopControlAxis copilotLJoystickX = {TeleopControlMappingEnums::CO_PILOT, TeleopControlMappingEnums::LEFT_JOYSTICK_X, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::SYNCED, 1.0};
    const TeleopControlAxis copilotLJoystickY = {TeleopControlMappingEnums::CO_PILOT, TeleopControlMappingEnums::LEFT_JOYSTICK_Y, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::REVERSED, 1.0};
    const TeleopControlAxis copilotRJoystickX = {TeleopControlMappingEnums::CO_PILOT, TeleopControlMappingEnums::RIGHT_JOYSTICK_X, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::SYNCED, 1.0};
    const TeleopControlAxis copilotRJoystickY = {TeleopControlMappingEnums::CO_PILOT, TeleopControlMappingEnums::RIGHT_JOYSTICK_X, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::REVERSED, 1.0};
    const TeleopControlAxis copilotLTrigger = {TeleopControlMappingEnums::CO_PILOT, TeleopControlMappingEnums::LEFT_TRIGGER, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::SYNCED, 1.0};
    const TeleopControlAxis copilotRTrigger = {TeleopControlMappingEnums::CO_PILOT, TeleopControlMappingEnums::RIGHT_TRIGGER, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::SYNCED, 1.0};

    const TeleopControlAxis extra1LJoystickX = {TeleopControlMappingEnums::EXTRA1, TeleopControlMappingEnums::LEFT_JOYSTICK_X, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::SYNCED, 1.0};
    const TeleopControlAxis extra1LJoystickY = {TeleopControlMappingEnums::EXTRA1, TeleopControlMappingEnums::LEFT_JOYSTICK_Y, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::REVERSED, 1.0};
    const TeleopControlAxis extra1RJoystickX = {TeleopControlMappingEnums::EXTRA1, TeleopControlMappingEnums::RIGHT_JOYSTICK_X, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::SYNCED, 1.0};
    const TeleopControlAxis extra1RJoystickY = {TeleopControlMappingEnums::EXTRA1, TeleopControlMappingEnums::RIGHT_JOYSTICK_X, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::REVERSED, 1.0};
    const TeleopControlAxis extra1LTrigger = {TeleopControlMappingEnums::EXTRA1, TeleopControlMappingEnums::LEFT_TRIGGER, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::SYNCED, 1.0};
    const TeleopControlAxis extra1RTrigger = {TeleopControlMappingEnums::EXTRA1, TeleopControlMappingEnums::RIGHT_TRIGGER, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::SYNCED, 1.0};

    const TeleopControlAxis extra2LJoystickX = {TeleopControlMappingEnums::EXTRA2, TeleopControlMappingEnums::LEFT_JOYSTICK_X, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::SYNCED, 1.0};
    const TeleopControlAxis extra2LJoystickY = {TeleopControlMappingEnums::EXTRA2, TeleopControlMappingEnums::LEFT_JOYSTICK_Y, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::REVERSED, 1.0};
    const TeleopControlAxis extra2RJoystickX = {TeleopControlMappingEnums::EXTRA2, TeleopControlMappingEnums::RIGHT_JOYSTICK_X, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::SYNCED, 1.0};
    const TeleopControlAxis extra2RJoystickY = {TeleopControlMappingEnums::EXTRA2, TeleopControlMappingEnums::RIGHT_JOYSTICK_X, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::REVERSED, 1.0};
    const TeleopControlAxis extra2LTrigger = {TeleopControlMappingEnums::EXTRA2, TeleopControlMappingEnums::LEFT_TRIGGER, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::SYNCED, 1.0};
    const TeleopControlAxis extra2RTrigger = {TeleopControlMappingEnums::EXTRA2, TeleopControlMappingEnums::RIGHT_TRIGGER, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::SYNCED, 1.0};

    const TeleopControlAxis extra3LJoystickX = {TeleopControlMappingEnums::EXTRA3, TeleopControlMappingEnums::LEFT_JOYSTICK_X, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::SYNCED, 1.0};
    const TeleopControlAxis extra3LJoystickY = {TeleopControlMappingEnums::EXTRA3, TeleopControlMappingEnums::LEFT_JOYSTICK_Y, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::REVERSED, 1.0};
    const TeleopControlAxis extra3RJoystickX = {TeleopControlMappingEnums::EXTRA3, TeleopControlMappingEnums::RIGHT_JOYSTICK_X, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::SYNCED, 1.0};
    const TeleopControlAxis extra3RJoystickY = {TeleopControlMappingEnums::EXTRA3, TeleopControlMappingEnums::RIGHT_JOYSTICK_X, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::REVERSED, 1.0};
    const TeleopControlAxis extra3LTrigger = {TeleopControlMappingEnums::EXTRA3, TeleopControlMappingEnums::LEFT_TRIGGER, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::SYNCED, 1.0};
    const TeleopControlAxis extra3RTrigger = {TeleopControlMappingEnums::EXTRA3, TeleopControlMappingEnums::RIGHT_TRIGGER, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::SYNCED, 1.0};

    const TeleopControlAxis extra4LJoystickX = {TeleopControlMappingEnums::EXTRA4, TeleopControlMappingEnums::LEFT_JOYSTICK_X, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::SYNCED, 1.0};
    const TeleopControlAxis extra4LJoystickY = {TeleopControlMappingEnums::EXTRA4, TeleopControlMappingEnums::LEFT_JOYSTICK_Y, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::REVERSED, 1.0};
    const TeleopControlAxis extra4RJoystickX = {TeleopControlMappingEnums::EXTRA4, TeleopControlMappingEnums::RIGHT_JOYSTICK_X, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::SYNCED, 1.0};
    const TeleopControlAxis extra4RJoystickY = {TeleopControlMappingEnums::EXTRA4, TeleopControlMappingEnums::RIGHT_JOYSTICK_Y, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::REVERSED, 1.0};
    const TeleopControlAxis extra4LTrigger = {TeleopControlMappingEnums::EXTRA4, TeleopControlMappingEnums::LEFT_TRIGGER, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::SYNCED, 1.0};
    const TeleopControlAxis extra4RTrigger = {TeleopControlMappingEnums::EXTRA4, TeleopControlMappingEnums::RIGHT_TRIGGER, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::SYNCED, 1.0};

    // @ADDMECH add functions mapping to axes
    robin_hood::unordered_map<TeleopControlFunctions::FUNCTION, const TeleopControlAxis> m_axisMap{
        {TeleopControlFunctions::HOLONOMIC_DRIVE_FORWARD, driverLJoystickY},
        {TeleopControlFunctions::HOLONOMIC_DRIVE_STRAFE, driverLJoystickX},
        {TeleopControlFunctions::HOLONOMIC_DRIVE_ROTATE, driverRJoystickX},
        {TeleopControlFunctions::MANUAL_ROTATE, extra1LJoystickY},
        {TeleopControlFunctions::MANUAL_EXTEND_RETRACT, extra2LJoystickY},
        {TeleopControlFunctions::HOLD_POSITION_ROTATE, copilotLJoystickY},
        {TeleopControlFunctions::MANUAL_EXTEND_RETRACT, copilotRJoystickY}};

    // @ADDMECH add functions mapping to buttons
    robin_hood::unordered_map<TeleopControlFunctions::FUNCTION, const TeleopControlButton> m_buttonMap{
        {TeleopControlFunctions::FINDTARGET, driverLBumper},
        {TeleopControlFunctions::HOLONOMIC_ROTATE_FRONT, driverDPad0},
        {TeleopControlFunctions::HOLONOMIC_ROTATE_BACK, driverDPad180},
        {TeleopControlFunctions::HOLONOMIC_ROTATE_LEFT, driverDPad270},
        {TeleopControlFunctions::HOLONOMIC_ROTATE_RIGHT, driverDPad90},
        {TeleopControlFunctions::DRIVE_TO_WALL_GRID, driverXButton},
        {TeleopControlFunctions::DRIVE_TO_COOP_GRID, driverYButton},
        {TeleopControlFunctions::DRIVE_TO_HP_GRID, driverBButton},
        {TeleopControlFunctions::REZERO_PIGEON, driverAButton},
        {TeleopControlFunctions::HOLD_POSITION, driverRBumper},
        {TeleopControlFunctions::BALANCE_MODE, driverBButton},
        {TeleopControlFunctions::AUTO_BALANCE, driverLStickPressed},
        {TeleopControlFunctions::DRIVE_TO_HUMAN_PLAYER_LEFT, driverLStickPressed},
        {TeleopControlFunctions::DRIVE_TO_HUMAN_PLAYER_RIGHT, driverRStickPressed},
        {TeleopControlFunctions::FRONT_LEFT_BUMPER_TURNABOUT_POINT, driverRBumper},
        {TeleopControlFunctions::FRONT_RIGHT_BUMPER_TURNABOUT_POINT, driverLBumper},
        {TeleopControlFunctions::HOLD_POSITION, driverStartButton},

        // co-pilot controls

        {TeleopControlFunctions::STARTING_POSITION_EXTEND, copilotStartButton},
        {TeleopControlFunctions::CYCLE_GRABBER, copilotSelectButton},
        {TeleopControlFunctions::HUMAN_PLAYER_LEVEL, copilotBButton},
        {TeleopControlFunctions::LOW, copilotAButton},
        {TeleopControlFunctions::MED, copilotXButton},
        {TeleopControlFunctions::HIGH, copilotYButton},
        {TeleopControlFunctions::OPEN, copilotLBumper},
        {TeleopControlFunctions::GRAB, copilotRBumper},

        {TeleopControlFunctions::HOLD_POSITION_ROTATE, extra1AButton},
        {TeleopControlFunctions::CUBE_BACKROW_ROTATE, extra1XButton},
        {TeleopControlFunctions::CUBE_MIDROW_ROTATE, extra1BButton},
        {TeleopControlFunctions::CONE_BACKROW_ROTATE, extra1YButton},
        {TeleopControlFunctions::CONE_MIDROW_ROTATE, extra1DPad0},
        {TeleopControlFunctions::HUMAN_PLAYER_STATION_ROTATE, extra1DPad90},
        {TeleopControlFunctions::FLOOR_POSITION_ROTATE, extra1DPad270},
        {TeleopControlFunctions::STARTING_POSITION_ROTATE, extra1DPad180},

        {TeleopControlFunctions::HOLD_POSITION_EXTEND, extra2AButton},
        {TeleopControlFunctions::CUBE_BACKROW_EXTEND, extra2XButton},
        {TeleopControlFunctions::CUBE_MIDROW_EXTEND, extra2BButton},
        {TeleopControlFunctions::CONE_BACKROW_EXTEND, extra2YButton},
        {TeleopControlFunctions::CONE_MIDROW_EXTEND, extra2DPad0},
        {TeleopControlFunctions::HUMAN_PLAYER_STATION_EXTEND, extra2DPad90},
        {TeleopControlFunctions::FLOOR_EXTEND, extra2DPad270},

        {TeleopControlFunctions::OPEN, extra3YButton},
        {TeleopControlFunctions::GRAB, extra3DPad270}

    };
};
