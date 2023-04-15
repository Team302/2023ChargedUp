//====================================================================================================================================================
// Copyright 2023 Lake Orion Robotics FIRST Team 302
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PRINTOVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRINTESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE
// OR OTHER DEALINGS IN THE SOFTWARE.
//====================================================================================================================================================
// author: not charlie writer of dumb code, copy/paster of better code
// co-author: imapenguin: creator of literally all of this code.
#include <AutomatedSystemTest/AutomatedSystemTest.h>
#include <frc/PowerDistribution.h>
#include <hw/factories/PDPFactory.h>
#include <utils/logging/Logger.h>
#include <hal/PowerDistribution.h>
#include <units/velocity.h>
#include <units/angular_velocity.h>

using namespace std;
AutomatedSystemTest::AutomatedSystemTest()
{
    TEST_STEP m_testStep;
    m_finishedcurrenttest;
}
void AutomatedSystemTest::Init()
{
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("Automatedsystemtest"), string("test init"), "reached");
}
void AutomatedSystemTest::Run()
{
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("Automatedsystemtest"), string("run"), "reached");
    m_BaseTest->Init();
    if (m_BaseTest->m_InitDone)
    {
        m_BaseTest->Run();

        if (m_BaseTest->m_BaseTestDone)
        {
            m_ArmTest->Init();

            if (m_ArmTest->m_ArmTestInitDone)
            {
                m_ArmTest->Run();

                if (m_ArmTest->m_ArmTestDone)
                {
                    m_XBackwardTest->Init();

                    if (m_XBackwardTest->m_XBackWardTestInitDone)
                    {
                        m_XBackwardTest->Run();

                        if (m_XBackwardTest->m_XBackwardTestDone)
                        {
                            m_XForwardTest->Init();

                            if (m_XForwardTest->m_XForwardTestInitDone)
                            {
                                m_XForwardTest->Run();

                                if (m_XForwardTest->m_XForwardTestDone)
                                {
                                    m_YBackwardTest->Init();

                                    if (m_YBackwardTest->m_YBackwardTestInitDone)
                                    {
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}
void AutomatedSystemTest::Exit()
{
}