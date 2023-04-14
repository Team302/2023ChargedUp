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
// co-author: imapenguin: creator of the "angel function"
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
    m_PDP = PDPFactory::GetFactory()->GetPDP();
    TEST_STEP m_testStep;
    m_finishedcurrentstep;
    m_finishedcurrenttest;
    TestStepnum;
    m_timer0 = 0;
    m_timer1 = 0;
    m_timer2 = 0;
    m_timer3 = 0;
    m_swervechassisforwardusage;
    m_swervechassisstrafeusage;
    m_swervechassisturnusage;
    m_InitialPDPWatts;
    frc::PowerDistribution *m_PDP;
}
void AutomatedSystemTest::Init()
{
    m_PDP = PDPFactory::GetFactory()->GetPDP();
    m_timer1 = 0;
    m_timer2 = 0;
    m_timer3 = 0;
    TestStepnum = 0;
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("Automatedsystemtest"), string("test init"), "reached");
}
void AutomatedSystemTest::Run()
{
    m_finishedcurrentstep = false;

    if (TestStepnum == SWERVE_VX_FORWARD)
    {
        m_finishedcurrentstep = TestswervevxForward();
    }
    else if (TestStepnum == SWERVE_VX_BACKWARD)
    {
        m_finishedcurrentstep = TestswervevxBackward();
    }
    else if (TestStepnum == SWERVE_VY_FORWARD)
    {
        m_finishedcurrentstep = TestswervevyForward();
    }
    else if (TestStepnum == SWERVE_VY_BACKWARD)
    {
        m_finishedcurrentstep = TestswervevyBackward();
    }
    if (m_finishedcurrentstep == true && TestStepnum < 7)
    {
        TestStepnum++;
    }
    TestswervevxForward();
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("Automatedsystemtest"), string("periodic"), "reached");
}

// swerve forward test
bool AutomatedSystemTest::TestswervevxForward()
{
    m_finishedcurrenttest = false;
    m_startedtest = false;
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("Automatedsystemtest"), string("start swerve vx forward"), "reached");
    auto m_swervechassis = ChassisFactory::GetChassisFactory()->GetSwerveChassis();
    if (m_swervechassis != nullptr)
    {
        ChassisMovement moveinfo;
        auto maxspeed = m_swervechassis->GetMaxSpeed();

        if (!m_finishedcurrenttest)
        {
            m_timer0++;
            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("Automatedsystemtest"), string("timer0"), (m_timer0));
            // might be nice to have methods for each item

            if (m_timer0 > 20)
            {
                if (m_timer0 < 90 && m_timer0 > 40)
                {
                    moveinfo.chassisSpeeds.vx = 0.5 * maxspeed;
                    m_startedtest = true;
                }
                else if (m_timer0 > 100)
                {
                    m_swervechassisforwardusage = m_PDP->GetTotalCurrent();
                    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("Automatedsystemtest"), string("swerve vx forward usage"), (m_swervechassisforwardusage));
                    moveinfo.chassisSpeeds.vx = 0.0 * maxspeed;
                    m_finishedcurrenttest = true;
                }
                m_swervechassis->Drive(moveinfo);
                m_swervechassis->ZeroAlignSwerveModules();
            }
            return true;
        }
    }
}

bool AutomatedSystemTest::TestswervevxBackward()
{
    // chasis backwards test
    {
        m_finishedcurrenttest = false;
        m_startedtest = false;
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("Automatedsystemtest"), string("start swerve vx forward"), "reached");
        auto m_swervechassis = ChassisFactory::GetChassisFactory()->GetSwerveChassis();
        if (m_swervechassis != nullptr)
        {
            ChassisMovement moveinfo;
            auto maxspeed = m_swervechassis->GetMaxSpeed();

            if (!m_finishedcurrenttest)
            {
                m_timer1++;
                Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("Automatedsystemtest"), string("timer0"), (m_timer0));
                // might be nice to have methods for each item

                if (m_timer1 > 20)
                {
                    if (m_timer0 < 90 && m_timer0 > 40)
                    {
                        moveinfo.chassisSpeeds.vx = -0.5 * maxspeed;
                        m_startedtest = true;
                    }
                    else if (m_timer1 > 300)
                    {
                        m_swervechassisforwardusage = m_PDP->GetTotalCurrent();
                        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("Automatedsystemtest"), string("swerve vx forward usage"), (m_swervechassisforwardusage));
                        moveinfo.chassisSpeeds.vx = 0.0 * maxspeed;
                        m_finishedcurrenttest = true;
                    }
                    m_swervechassis->Drive(moveinfo);
                }
                return true;
            }
            else
            {
                m_swervechassis->ZeroAlignSwerveModules();
            }
        }
    }
}

// swerve strafe test
bool AutomatedSystemTest::TestswervevyForward()
{
    return true;
    auto m_swervechassis = ChassisFactory::GetChassisFactory()->GetSwerveChassis();
    auto maxspeed = m_swervechassis->GetMaxSpeed();
    ChassisMovement moveinfo;

    m_timer2++;
    m_swervechassis->ZeroAlignSwerveModules();

    if (m_timer2 > 20 && m_timer2 < 59)
    {

        moveinfo.chassisSpeeds.vy = 0.05 * maxspeed;
        m_swervechassis->Drive(moveinfo);
    }

    if (m_timer2 == 300)
    {
        if (m_PDP != nullptr)
        {
            m_swervechassisstrafeusage = m_PDP->GetTotalCurrent();
            moveinfo.chassisSpeeds.vy = 0.0 * maxspeed;
            m_swervechassis->Drive(moveinfo);
            return true;
        }
        else
        {
            moveinfo.chassisSpeeds.vy = 0.0 * maxspeed;
            m_swervechassis->Drive(moveinfo);
            return false;
        }
    }
}
// swerve left test
bool AutomatedSystemTest::TestswervevyBackward()
{
    return true;
    auto m_swervechassis = ChassisFactory::GetChassisFactory()->GetSwerveChassis();
    auto maxspeed = m_swervechassis->GetMaxSpeed();
    ChassisMovement moveinfo;
    m_timer3++;
    m_swervechassis->ZeroAlignSwerveModules();

    if (m_timer3 > 20 && m_timer3 < 59)
    {

        moveinfo.chassisSpeeds.vy = -0.05 * maxspeed;
        m_swervechassis->Drive(moveinfo);
    }

    if (m_timer3 == 300)
    {
        if (m_PDP != nullptr)
        {
            m_swervechassisstrafeusage = m_PDP->GetTotalCurrent();
            moveinfo.chassisSpeeds.vy = 0.0 * maxspeed;
            m_swervechassis->Drive(moveinfo);
            return true;
        }
        else
        {
            moveinfo.chassisSpeeds.vy = 0.0 * maxspeed;
            m_swervechassis->Drive(moveinfo);
            return false;
        }
    }
}