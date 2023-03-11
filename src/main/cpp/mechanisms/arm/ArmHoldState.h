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
#include <string>

#include <mechanisms/base/Mech1IndMotorState.h>
#include <robotstate/IRobotStateChangeSubscriber.h>
#include <robotstate/RobotStateChanges.h>

class Arm;
class Extender;
class ControlData;
class TeleopControl;

class ArmHoldState : public Mech1IndMotorState, public IRobotStateChangeSubscriber
{
public:
    ArmHoldState() = delete;
    ArmHoldState(std::string stateName, int stateId, ControlData *control, double target);
    ~ArmHoldState() = default;

    void Init() override;
    void Run() override;
    bool AtTarget() const override;
    void Update(RobotStateChanges::StateChange change, int value) override;

private:
    Arm *m_arm;
    Extender *m_extender;
    TeleopControl *m_controller;
    ControlData *m_controlData;
    bool m_coneMode;

    static constexpr double m_offset[2] = {0.0412269,
                                           0.0446119};
    static constexpr double m_armComponent[2] = {0.000421601,
                                                 -0.00010589};
    static constexpr double m_extenderComponent[2] = {0.000703398,
                                                      0.000633812};
    static constexpr double m_armSquaredComponent[2] = {-0.00000267649,
                                                        0.00000489504};
    static constexpr double m_extenderSquaredComponent[2] = {-0.0000138281,
                                                             0.00000226623};
};
