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
    m_swerveforwardtimer = 0;
    m_swervestrafetimer = 0;
    m_swervetruntimer = 0;
    m_armtimer = 0;
    m_extendertimer = 0;
    m_disableswerveforwardtest = false;
}
void AutomatedSystemTest::Init()
{
    BasePDPValue();
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("Automatedsystemtest"), string("startingwattage"), (m_basepdpusage));
    m_swerveforwardtimer = 0;
    m_swervestrafetimer = 0;
    m_swervetruntimer = 0;
    m_armtimer = 0;
    m_extendertimer = 0;
    m_disableswerveforwardtest = false;
}
void AutomatedSystemTest::Run()
{
    // TestArm();
    // TestExtender();
    TestSwerve();
}
void AutomatedSystemTest::BasePDPValue()
{
    if (m_PDP != nullptr)
    {
        m_basepdpusage = m_PDP->GetTotalCurrent();
        m_gotbasepdpuseage = true;
    }
    else
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, string("Automatedsystemtest"), string("pdp"), "cannot accses pdp");
    }
}

void AutomatedSystemTest::TestArm()
{
    if (m_gotbasepdpuseage == true)
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
            m_finishedarmtest = true;
        }
        Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, string("Automatedsystemtest"), string("pdp"), "cannot accses pdp");
        ArmStateMgr::GetInstance()->SetCurrentState(ArmStateMgr::ARM_STATE::STARTING_POSITION_ROTATE, true);
    }
}
void AutomatedSystemTest::TestExtender()
{
    if (m_finishedarmtest == true)
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
        }
        Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, string("Automatedsystemtest"), string("pdp"), "cannot accses pdp");
        ExtenderStateMgr::GetInstance()->SetCurrentState(ExtenderStateMgr::EXTENDER_STATE::STARTING_POSITION_EXTEND, (true));
    }
}
void AutomatedSystemTest::TestSwerve()
{
    m_finishedextendertest = true;
    if (m_finishedextendertest = true)
    {
        auto m_swervechassis = ChassisFactory::GetChassisFactory()->GetSwerveChassis();
        auto maxspeed = m_swervechassis->GetMaxSpeed();
        auto maxAngSpeed = m_swervechassis->GetMaxAngularSpeed();
        ChassisMovement moveinfo;
        if (m_swervechassis != nullptr)
        {
            m_swervechassis->ZeroAlignSwerveModules();
            m_finishedzeroswerve = true;
        }
        else
        {
            Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, string("Automatedsystemtest"), string("chassis"), "cannot accses swervedrive");
        }
        if (m_finishedzeroswerve == true, m_disableswerveforwardtest == false)
        {
            moveinfo.chassisSpeeds.vx = 0.5 * maxspeed;
            m_swervechassis->Drive(moveinfo);
            m_swerveforwardtimer++;
            if (m_swerveforwardtimer > 40)
            {
                if (m_PDP != nullptr)
                {
                    m_swervechassisforwardusage = m_PDP->GetTotalCurrent();
                    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("Automatedsystemtest"), string("swerve forward usage"), (m_swervechassisforwardusage));
                    moveinfo.chassisSpeeds.vx = 0.0 * maxspeed;
                    m_swervechassis->Drive(moveinfo);
                    // m_finishedchassisforwardtest = true;
                    m_disableswerveforwardtest = true;
                }
                else
                {
                    Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, string("Automatedsystemtest"), string("pdp"), "cannot accses pdp");
                }
            }
        }
        if (m_finishedchassisforwardtest == true)
        {
            moveinfo.chassisSpeeds.vy = 1.0 * maxspeed;
            m_swervechassis->Drive(moveinfo);
            m_swervestrafetimer++;
            if (m_swervestrafetimer > 1000)
            {
                if (m_PDP != nullptr)
                {
                    m_swervechassisstrafeusage = m_PDP->GetTotalCurrent();
                    moveinfo.chassisSpeeds.vy = 0.0 * maxspeed;
                    m_swervechassis->Drive(moveinfo);
                }
                else
                {
                    Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, string("Automatedsystemtest"), string("pdp"), "cannot accses pdp");
                }
            }
        }

        if (m_finishedchassisstrafetest == true)
        {
            moveinfo.chassisSpeeds.omega = 1.0 * maxAngSpeed;
            m_swervechassis->Drive(moveinfo);
            m_swervetruntimer++;
            if (m_swervetruntimer > 2000)
                if (m_PDP != nullptr)
                {
                    m_swervechassisturnusage = m_PDP->GetTotalCurrent();
                    moveinfo.chassisSpeeds.omega = 0.0 * maxAngSpeed;
                    m_swervechassis->Drive(moveinfo);
                }
                else
                {
                    Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, string("Automatedsystemtest"), string("pdp"), "cannot accses pdp");
                }
        }
    }
}