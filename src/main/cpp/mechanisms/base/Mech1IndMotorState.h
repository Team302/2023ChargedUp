
//====================================================================================================================================================
/// Copyright 2023 Lake Orion Robotics FIRST Team 302
///
/// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
/// to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
/// and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
///
/// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
///
/// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
/// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
/// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE
/// OR OTHER DEALINGS IN THE SOFTWARE.
//====================================================================================================================================================

#pragma once
#include <string>

#include <mechanisms/base/Mech1IndMotor.h>
#include <State.h>
#include <mechanisms/controllers/ControlData.h>

class Mech1IndMotorState : public State
{
public:
    Mech1IndMotorState(
        Mech1IndMotor *mechanism,
        std::string stateName,
        int stateId,
        ControlData *control,
        double target);
    Mech1IndMotorState() = delete;
    ~Mech1IndMotorState() = default;

    void Init() override;
    void Run() override;
    void Exit() override;
    bool AtTarget() const override;

    void LogInformation() const override;

    double GetCurrentTarget() const { return m_target; }
    double GetOriginalTarget() const { return m_originalTarget; }
    double GetRPS() const { return m_mechanism->GetSpeed(); }

    /// May move to protected later
    void SetTarget(double newTarget) { m_target = newTarget; }

protected:
    ControlData *GetControlData() const { return m_control; }

private:
    Mech1IndMotor *m_mechanism;
    ControlData *m_control;
    double m_target;
    const double m_originalTarget;
    bool m_positionBased;
    bool m_speedBased;
};
