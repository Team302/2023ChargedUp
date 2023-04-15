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

using namespace std;

ArmTest::ArmTest()
{
    m_armTestComplete = false;
    m_extenderTestComplete = false;
    m_ArmTestDone = false;
    m_timer0 = 0;
}
void ArmTest::Init()
{
    ExtenderStateMgr::GetInstance()->SetCurrentState(ExtenderStateMgr::EXTENDER_STATE::STARTING_POSITION_EXTEND, (true));
    ArmStateMgr::GetInstance()->SetCurrentState(ArmStateMgr::ARM_STATE::STARTING_POSITION_ROTATE, true);
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("Automatedsystemtest"), string("ArmTest (Init)"), "finished");
    m_ArmTestInitDone = true;
}
void ArmTest::Run()
{
    auto m_armstate = ArmStateMgr::GetInstance()->GetCurrentState();
    auto m_extenderstate = ExtenderStateMgr::GetInstance()->GetCurrentState();
    if (m_armstate == ArmStateMgr::ARM_STATE::STARTING_POSITION_ROTATE)
    {
        ArmStateMgr::GetInstance()->SetCurrentState(ArmStateMgr::ARM_STATE::CONE_BACKROW_ROTATE, true);
    }

    if (m_extenderstate == ExtenderStateMgr::EXTENDER_STATE::STARTING_POSITION_EXTEND && m_armstate == ArmStateMgr::ARM_STATE::CONE_BACKROW_ROTATE)
    {
        ExtenderStateMgr::GetInstance()->SetCurrentState(ExtenderStateMgr::EXTENDER_STATE::CONE_BACKROW_EXTEND, (true));
        m_extenderTestComplete = true;
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("Automatedsystemtest"), string("ArmTest (Run)"), "finished");
        m_timer0++;
        if (m_timer0 > 50)
        {
            ExtenderStateMgr::GetInstance()->SetCurrentState(ExtenderStateMgr::EXTENDER_STATE::STARTING_POSITION_EXTEND, (true));
            if (m_extenderstate == ExtenderStateMgr::EXTENDER_STATE::STARTING_POSITION_EXTEND)
            {
                ArmStateMgr::GetInstance()->SetCurrentState(ArmStateMgr::ARM_STATE::STARTING_POSITION_ROTATE, (true));
                if (m_armstate == ArmStateMgr::ARM_STATE::STARTING_POSITION_ROTATE)
                {
                    m_armTestComplete = true;
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