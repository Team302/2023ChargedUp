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
#include <AutomatedSystemTest/XBackwardTest.h>
#include <utils/logging/Logger.h>
#include <string>
#include <chassis/swerve/swerveChassis.h>
#include <chassis/swerve/headingStates/ISwerveDriveOrientation.h>
#include <chassis/ChassisFactory.h>
#include <chassis/ChassisMovement.h>
#include <frc/kinematics/ChassisSpeeds.h>
using namespace std;

XBackwardTest::XBackwardTest()
{
}
void XBackwardTest::Init()
{
    auto m_swervechassis = ChassisFactory::GetChassisFactory()->GetSwerveChassis();
    m_swervechassis->ZeroAlignSwerveModules();
}
void XBackwardTest::Run()
{
    auto m_swervechassis = ChassisFactory::GetChassisFactory()->GetSwerveChassis();
    if (m_swervechassis != nullptr)
    {
        ChassisMovement moveinfo;
        auto maxspeed = m_swervechassis->GetMaxSpeed();
        m_timer0++;
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("Automatedsystemtest"), string("timer0"), (m_timer0));
        // might be nice to have methods for each item
        if (m_timer0 < 300 && m_timer0 > 20)
        {
            moveinfo.chassisSpeeds.vx = -0.5 * maxspeed;
        }
        else if (m_timer0 > 300)
        {
            moveinfo.chassisSpeeds.vx = 0.0 * maxspeed;
            m_xBackwardTestComplete = true;
        }
        m_swervechassis->Drive(moveinfo);
    }

    else
    {
        m_swervechassis->ZeroAlignSwerveModules();
    }
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("Automatedsystemtest"), string("X Backward Test"), "running");
}

bool XBackwardTest::IsDone()
{
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("Automatedsystemtest"), string("X Backward Test"), "complete");
    return m_xBackwardTestComplete;
}