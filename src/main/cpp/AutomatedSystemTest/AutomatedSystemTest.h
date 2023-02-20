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
    int m_armtimer = 0;
    int m_extendertimer = 0;
    int m_swervetruntimer = 0;
    int m_swervestrafetimer = 0;
    int m_swerveforwardtimer = 0;
    bool m_finishedarmtest = false;
    bool m_gotbasepdpuseage = false;
    bool m_finishedextendertest = false;
    bool m_finishedchassistest = false;
    bool m_finishedchassisforwardtest = false;
    bool m_finishedchassisstrafetest = false;
    bool m_finishedchassisturntest = false;
    bool m_finishedalltest = false;
    bool m_finishedzeroswerve = false;
    bool m_disableswerveforwardtest = false;
    double m_basepdpusage;
    double m_armusage;
    double m_extenderusage;
    double m_swervechassisforwardusage;
    double m_swervechassisstrafeusage;
    double m_swervechassisturnusage;
    double test;
    double m_InitialPDPWatts;
    frc::PowerDistribution *m_PDP;
    void BasePDPValue();
    void TestSwerve();
    void TestExtender();
    void TestArm();
    // double GetTestPnumatics();
    // double GetPDHTemp();
};