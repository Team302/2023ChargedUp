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

// Team 302 Includes
#include <mechanisms/arm/ArmManualState.h>
#include <mechanisms/arm/ArmState.h>
#include <mechanisms/arm/ArmHoldPosHelper.h>
#include <mechanisms/controllers/ControlData.h>
#include <teleopcontrol/TeleopControl.h>
#include <mechanisms/MechanismFactory.h>
#include <robotstate/RobotState.h>
#include <robotstate/RobotStateChanges.h>

/// DEBUGGING
#include <utils/logging/Logger.h>

ArmManualState::ArmManualState(std::string stateName,
                               int stateId,
                               ControlData *control0,
                               double target0) : ArmState(stateName, stateId, control0, target0),
                                                 IRobotStateChangeSubscriber(),
                                                 m_arm(MechanismFactory::GetMechanismFactory()->GetArm()),
                                                 m_controller(TeleopControl::GetInstance()),
                                                 m_controlData(control0),
                                                 m_gamepieceMode(RobotStateChanges::Cone),
                                                 m_intakeState(IntakeStateMgr::INTAKE_STATE::HOLD)
{
    RobotState::GetInstance()->RegisterForStateChanges(this, RobotStateChanges::StateChange::DesiredGamePiece);
    RobotState::GetInstance()->RegisterForStateChanges(this, RobotStateChanges::StateChange::IntakeState);
}

void ArmManualState::Init()
{
}

void ArmManualState::Run()
{
    if (m_controller != nullptr && m_arm != nullptr)
    {
        auto percent = m_controller->GetAxisValue(TeleopControlFunctions::MANUAL_ROTATE);
        if (percent < 0.0)
        {
            percent *= GetCurrentTarget(); // if we want to change downward speed change,
                                           // update target in xml
        }

        auto target = percent + ArmHoldPosHelper::CalculateHoldPositionTarget(m_arm->GetPositionDegrees().to<double>(),
                                                                              MechanismFactory::GetMechanismFactory()->GetExtender()->GetPositionInches().to<double>(),
                                                                              m_gamepieceMode,
                                                                              m_intakeState);
        m_arm->SetControlConstants(0, m_controlData);
        m_arm->UpdateTarget(target);
        m_arm->Update();
    }
}

bool ArmManualState::AtTarget() const
{
    return true;
}

void ArmManualState::Update(RobotStateChanges::StateChange change, int state)
{
    if (change == RobotStateChanges::DesiredGamePiece)
    {
        m_gamepieceMode = static_cast<RobotStateChanges::GamePiece>(state);
    }
    else if (change == RobotStateChanges::IntakeState)
    {
        m_intakeState = static_cast<IntakeStateMgr::INTAKE_STATE>(state);
    }
}