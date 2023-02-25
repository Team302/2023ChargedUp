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
// author: not charlie writer of dumb code, copy/paster of better code
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
}
void AutomatedSystemTest::Init()
{
    BasePDPValue();
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("Automatedsystemtest"), string("startingwattage"), (m_basepdpusage));
    m_timer1 = 0;
    m_timer2 = 0;
    m_timer3 = 0;
}
void AutomatedSystemTest::Run()
{
    bool finishedcurrentstep = true;
    int TestStepnum;
    if (TestStepnum == NO_TEST)
    {
    }
    else if (TestStepnum == ARM_TEST)
    {
        finishedcurrentstep = TestArm();
    }
    else if (TestStepnum == EXTENDER_TEST)
    {
        finishedcurrentstep = TestExtender();
    }
    else if (TestStepnum == SWERVE_VX_FORWARD)
    {
        finishedcurrentstep = TestswervevxForward();
    }
    else if (TestStepnum == SWERVE_VX_BACKWARD)
    {
        finishedcurrentstep = TestswervevxBackward();
    }
    else if (TestStepnum == SWERVE_VY_FORWARD)
    {
        finishedcurrentstep = TestswervevyForward();
    }
    else if (TestStepnum == SWERVE_VY_BACKWARD)
    {
        finishedcurrentstep = TestswervevyBackward();
    }
    if (finishedcurrentstep == true)
    {
        TestStepnum++;
    }
}
bool AutomatedSystemTest::BasePDPValue()
{
    if (m_PDP != nullptr)
    {
        m_basepdpusage = m_PDP->GetTotalCurrent();
        return true;
    }
    else
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, string("Automatedsystemtest"), string("pdp"), "cannot accses pdp");
        return false;
    }
}

bool AutomatedSystemTest::TestArm()
{
    ArmStateMgr::GetInstance()->SetCurrentState(ArmStateMgr::ARM_STATE::STARTING_POSITION_ROTATE, true);
    auto m_armstate = ArmStateMgr::GetInstance()->GetCurrentState();
    if (m_armstate == ArmStateMgr::ARM_STATE::STARTING_POSITION_ROTATE)
    {
        ArmStateMgr::GetInstance()->SetCurrentState(ArmStateMgr::ARM_STATE::CONE_BACKROW_ROTATE, true);
    }

    if (m_PDP != nullptr)
    {
        m_armusage = m_PDP->GetTotalCurrent();
        ArmStateMgr::GetInstance()->SetCurrentState(ArmStateMgr::ARM_STATE::STARTING_POSITION_ROTATE, true);
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("Automatedsystemtest"), string("arm use wattage"), (m_armusage));
        return true;
    }
    Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, string("Automatedsystemtest"), string("pdp"), "cannot accses pdp");
    ArmStateMgr::GetInstance()->SetCurrentState(ArmStateMgr::ARM_STATE::STARTING_POSITION_ROTATE, true);
    return false;
}
bool AutomatedSystemTest::TestExtender()
{
    ExtenderStateMgr::GetInstance()->SetCurrentState(ExtenderStateMgr::EXTENDER_STATE::STARTING_POSITION_EXTEND, (true));
    auto m_extenderstate = ExtenderStateMgr::GetInstance()->GetCurrentState();
    if (m_extenderstate == ExtenderStateMgr::EXTENDER_STATE::STARTING_POSITION_EXTEND)
    {
        ExtenderStateMgr::GetInstance()->SetCurrentState(ExtenderStateMgr::EXTENDER_STATE::CONE_BACKROW_EXTEND, (true));
    }

    if (m_PDP != nullptr)
    {
        m_extenderusage = m_PDP->GetTotalCurrent();
        ExtenderStateMgr::GetInstance()->SetCurrentState(ExtenderStateMgr::EXTENDER_STATE::STARTING_POSITION_EXTEND, (true));
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("Automatedsystemtest"), string("extender use wattage"), (m_extenderusage));
        return true;
    }
    Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, string("Automatedsystemtest"), string("pdp"), "cannot accses pdp");
    ExtenderStateMgr::GetInstance()->SetCurrentState(ExtenderStateMgr::EXTENDER_STATE::STARTING_POSITION_EXTEND, (true));
    return false;
}
// swerve forward test
bool AutomatedSystemTest::TestswervevxForward()
{
    auto m_swervechassis = ChassisFactory::GetChassisFactory()->GetSwerveChassis();
    auto maxspeed = m_swervechassis->GetMaxSpeed();
    ChassisMovement moveinfo;

    m_timer0++;
    m_swervechassis->ZeroAlignSwerveModules();
    if (m_timer0 == 20)
    {
        moveinfo.chassisSpeeds.vx = 0.25 * maxspeed;
        m_swervechassis->Drive(moveinfo);
    }
    if (m_timer0 = 100)
    {
        if (m_PDP != nullptr)
        {
            m_swervechassisforwardusage = m_PDP->GetTotalCurrent();
            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("Automatedsystemtest"), string("swerve forward usage"), (m_swervechassisforwardusage));
            moveinfo.chassisSpeeds.vx = 0.0 * maxspeed;
            m_swervechassis->Drive(moveinfo);
            return true;
        }
        else
        {
            Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, string("Automatedsystemtest"), string("pdp"), "cannot accses pdp");
            moveinfo.chassisSpeeds.vx = 0.0 * maxspeed;
            m_swervechassis->Drive(moveinfo);
            return false;
        }
    }
}
bool AutomatedSystemTest::TestswervevxBackward()
{
    // chasis backwards test
    auto m_swervechassis = ChassisFactory::GetChassisFactory()->GetSwerveChassis();
    auto maxspeed = m_swervechassis->GetMaxSpeed();
    ChassisMovement moveinfo;

    m_timer1++;
    m_swervechassis->ZeroAlignSwerveModules();
    if (m_timer1 > 20 && m_timer1 < 59)
    {
        moveinfo.chassisSpeeds.vx = -0.25 * maxspeed;
        m_swervechassis->Drive(moveinfo);
    }
    if (m_timer1 > 100)
    {
        if (m_PDP != nullptr)
        {
            m_swervechassisforwardusage = m_PDP->GetTotalCurrent();
            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("Automatedsystemtest"), string("swerve forward usage"), (m_swervechassisforwardusage));
            moveinfo.chassisSpeeds.vx = 0.0 * maxspeed;
            m_swervechassis->Drive(moveinfo);
            return true;
        }
        else
        {
            Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, string("Automatedsystemtest"), string("pdp"), "cannot accses pdp");
            moveinfo.chassisSpeeds.vx = 0.0 * maxspeed;
            m_swervechassis->Drive(moveinfo);
            return false;
        }
    }
}

// swerve strafe test
bool AutomatedSystemTest::TestswervevyForward()
{
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

// swerve turn test

/*if (m_finishedchassisstrafetest == true && m_finishedchassisturntest == false)
{
    m_timer2++;
    m_swervechassis->ZeroAlignSwerveModules();

    if (m_timer2 > 20 && m_timer2 < 59)
    {
        moveinfo.chassisSpeeds.omega = 0.0 * maxAngSpeed;
        m_swervechassis->Drive(moveinfo);
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("Automatedsystemtest"), string("timer 2 start"), (m_timer2));
        m_finishedstartswerveturntest = true;
    }
}
if (m_timer2 > 400 && m_finishedstartswerveturntest == true)
{

    if (m_PDP != nullptr)
    {
        m_swervechassisturnusage = m_PDP->GetTotalCurrent();
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("Automatedsystemtest"), string("timer 2 end"), (m_timer2));
        moveinfo.chassisSpeeds.omega = 0.0 * maxAngSpeed;
        m_swervechassis->Drive(moveinfo);
        m_swervechassis->ZeroAlignSwerveModules();
        m_finishedswerveturntest = true;
    }

    else
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, string("Automatedsystemtest"), string("pdp"), "cannot accses pdp");
    }
}*/