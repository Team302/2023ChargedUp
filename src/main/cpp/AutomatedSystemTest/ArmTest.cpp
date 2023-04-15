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
#include <AutomatedSystemTest/ArmTest.h>
#include <mechanisms/arm/ArmStateMgr.h>
#include <utils/logging/Logger.h>
#include <string>
#include <mechanisms/extender/ExtenderStateMgr.h>
#include <mechanisms\Intake\IntakeStateMgr.h>
#include <mechanisms/Intake/IntakeState.h>
#include <mechanisms/arm/ArmState.h>
#include <mechanisms/extender/ExtenderState.h>
using namespace std;

ArmTest::ArmTest()
{
    m_armTestComplete = false;
    m_extenderTestComplete = false;
    m_ArmTestDone = false;
    m_timer0 = 0;
    intakeTestComplete = false;
}
void ArmTest::Init()
{
    extenderPointer = ExtenderStateMgr::GetInstance();
    extenderPointer->SetCurrentState(ExtenderStateMgr::EXTENDER_STATE::STARTING_POSITION_EXTEND, (true));
    ArmStateMgr::GetInstance()->SetCurrentState(ArmStateMgr::ARM_STATE::STARTING_POSITION_ROTATE, true);
    IntakeStateMgr::GetInstance()->SetCurrentState(IntakeStateMgr::INTAKE_STATE::OFF, true);
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("Automatedsystemtest"), string("ArmTest (Init)"), "finished");
    m_ArmTestInitDone = true;
}
void ArmTest::Run()
{
    auto m_intakestate = IntakeStateMgr::GetInstance()->GetCurrentStatePtr();
    auto m_armstate = ArmStateMgr::GetInstance()->GetCurrentStatePtr();
    auto m_extenderstate = extenderPointer->GetCurrentStatePtr();
    if (m_armstate->GetStateId() == ArmStateMgr::ARM_STATE::STARTING_POSITION_ROTATE)
    {
        ArmStateMgr::GetInstance()->SetCurrentState(ArmStateMgr::ARM_STATE::CONE_BACKROW_ROTATE, true);
    }

    if (m_extenderstate->GetStateId() == ExtenderStateMgr::EXTENDER_STATE::STARTING_POSITION_EXTEND && m_armstate->GetStateId() == ArmStateMgr::ARM_STATE::CONE_BACKROW_ROTATE)
    {
        extenderPointer->SetCurrentState(ExtenderStateMgr::EXTENDER_STATE::CONE_BACKROW_EXTEND, (true));
        m_extenderTestComplete = true;
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("Automatedsystemtest"), string("ArmTest (Run)"), "finished");
        if (m_extenderstate->GetStateId() == ExtenderStateMgr::EXTENDER_STATE::CONE_BACKROW_EXTEND && m_armstate->GetStateId() == ArmStateMgr::ARM_STATE::CONE_BACKROW_ROTATE)
        {
            IntakeStateMgr::GetInstance()->SetCurrentState(IntakeStateMgr::INTAKE_STATE::INTAKE, true);
            m_timer0++;
            if (m_timer0 > 100)
            {
                IntakeStateMgr::GetInstance()->SetCurrentState(IntakeStateMgr::INTAKE_STATE::OFF, true);
                if (m_intakestate->GetStateId() == IntakeStateMgr::INTAKE_STATE::INTAKE && m_intakestate->AtTarget())
                {
                    ExtenderStateMgr::GetInstance()->SetCurrentState(ExtenderStateMgr::EXTENDER_STATE::STARTING_POSITION_EXTEND, (true));
                    if (m_extenderstate->GetStateId() == ExtenderStateMgr::EXTENDER_STATE::STARTING_POSITION_EXTEND && m_extenderstate->AtTarget())
                    {
                        ArmStateMgr::GetInstance()->SetCurrentState(ArmStateMgr::ARM_STATE::STARTING_POSITION_ROTATE, (true));
                        if (m_armstate->GetStateId() == ArmStateMgr::ARM_STATE::STARTING_POSITION_ROTATE && m_armstate->AtTarget())
                        {
                            m_armTestComplete = true;
                        }
                    }
                }
            }
        }
    }
    else
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("Automatedsystemtest"), string("ArmTest (Run)"), "Arm or Extender test problem");
    }

    if (m_armTestComplete && m_extenderTestComplete)
    {
        m_ArmTestDone = true;
    }
}