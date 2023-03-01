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
#pragma once
#include <frc/PowerDistribution.h>
#include <hw/factories/PDPFactory.h>
#include <mechanisms/arm/ArmStateMgr.h>
#include <mechanisms/extender/ExtenderStateMgr.h>
#include <chassis/swerve/swerveChassis.h>
#include <chassis/swerve/headingStates/ISwerveDriveOrientation.h>
#include <chassis/ChassisFactory.h>
#include <chassis/ChassisMovement.h>
#include <frc/kinematics/ChassisSpeeds.h>

using namespace std;
class PowerDistribution;
class AutomatedSystemTest
{
public:
    AutomatedSystemTest();
    ~AutomatedSystemTest() = default;
    void Init();
    void Run();

private:
    enum TEST_STEP
    {
        BASE_TEST = 0,
        ARM_TEST = 1,
        EXTENDER_TEST = 2,
        SWERVE_VX_FORWARD = 3,
        SWERVE_VX_BACKWARD = 4,
        SWERVE_VY_FORWARD = 5,
        SWERVE_VY_BACKWARD = 6,
    };
    TEST_STEP m_testStep;
    bool m_finishedcurrentstep;
    bool m_finishedcurrenttest;
    bool m_startedtest;
    int TestStepnum;
    int m_timer0 = 0;
    int m_timer1 = 0;
    int m_timer2 = 0;
    int m_timer3 = 0;
    double m_basepdpusage;
    double m_armusage;
    double m_extenderusage;
    double m_swervechassisforwardusage;
    double m_swervechassisstrafeusage;
    double m_swervechassisturnusage;
    double m_InitialPDPWatts;
    frc::PowerDistribution *m_PDP;
    bool BasePDPValue();
    bool TestExtender();
    bool TestArm();
    bool TestswervevxForward();
    bool TestswervevxBackward();
    bool TestswervevyForward();
    bool TestswervevyBackward();
    // double GetTestPnumatics();
    // double GetPDHTemp();
};