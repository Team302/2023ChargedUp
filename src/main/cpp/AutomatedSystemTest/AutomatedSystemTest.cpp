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
    double m_basepdpusage;
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("Automatedsystemtest"), string("startingwattage"), (m_basepdpusage));
}
void Autotest()
{
}
void AutomatedSystemTest::GetBasePDPValue()
{
    if (m_PDP != nullptr)
    {
        m_basepdpusage = m_PDP->GetTotalCurrent();
    }
    Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, string("Automatedsystemtest"), string("pdp"), "cannot accses pdp");
}

/*void AutomatedSystemTest::GetTestArm()
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
    }
    Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, string("Automatedsystemtest"), string("pdp"), "cannot accses pdp");
}*/
void AutomatedSystemTest::GetTestExtender()
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
    }
    Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, string("Automatedsystemtest"), string("pdp"), "cannot accses pdp");
}
void AutomatedSystemTest::GetTestSwerveInit()
{
    auto m_swervechassis = ChassisFactory::GetChassisFactory()->GetSwerveChassis();
    auto maxSpeed = m_swervechassis->GetMaxSpeed();
    auto maxAngSpeed = m_swervechassis->GetMaxAngularSpeed();
    ChassisMovement moveinfo;
    moveinfo.driveOption = ChassisOptionEnums::DriveStateType::FIELD_DRIVE;
    if (m_swervechassis != nullptr)
    {
        m_swervechassis->ZeroAlignSwerveModules();
    }
    else
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, string("Automatedsystemtest"), string("chassis"), "cannot accses swervedrive");
    }
    moveinfo.chassisSpeeds.vx = 1.0 * maxSpeed;

    if (m_PDP != nullptr)
    {
        m_swervechassisforwardusage = m_PDP->GetTotalCurrent();
    }
    else
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, string("Automatedsystemtest"), string("pdp"), "cannot accses pdp");
    }
}