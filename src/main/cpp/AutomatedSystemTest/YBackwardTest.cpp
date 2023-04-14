//====================================================================================================================================================
// Copyright 2023 Lake Orion Robotics FIRST Team 302
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EyPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE
// OR OTHER DEALINGS IN THE SOFTWARE.
//====================================================================================================================================================
#include <AutomatedSystemTest/yBackwardTest.h>
#include <utils/logging/Logger.h>
#include <string>
#include <chassis/swerve/swerveChassis.h>
#include <chassis/swerve/headingStates/ISwerveDriveOrientation.h>
#include <chassis/ChassisFactory.h>
#include <chassis/ChassisMovement.h>
#include <frc/kinematics/ChassisSpeeds.h>
using namespace std;

YBackwardTest::YBackwardTest()
{
}
void YBackwardTest::Init()
{
    auto m_swervechassis = ChassisFactory::GetChassisFactory()->GetSwerveChassis();
    m_swervechassis->ZeroAlignSwerveModules();
}
void YBackwardTest::Run()
{
    auto m_swervechassis = ChassisFactory::GetChassisFactory()->GetSwerveChassis();
    auto maxspeed = m_swervechassis->GetMaxSpeed();
    ChassisMovement moveinfo;
    m_timer0++;

    if (m_timer0 > 20 && m_timer0 < 300)
    {
        moveinfo.chassisSpeeds.vy = -0.05 * maxspeed;
        m_swervechassis->Drive(moveinfo);
    }

    else if (m_timer0 > 300)
    {

        moveinfo.chassisSpeeds.vy = 0.0 * maxspeed;
        m_swervechassis->Drive(moveinfo);
    }
    else
    {
        moveinfo.chassisSpeeds.vy = 0.0 * maxspeed;
        m_swervechassis->Drive(moveinfo);
    }
}

bool YBackwardTest::IsDone()
{
    return m_yBackwardTestComplete;
}