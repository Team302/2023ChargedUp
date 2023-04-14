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
#include <AutomatedSystemTest/XBackwardTest.h>
#include <AutomatedSystemTest/ArmTest.h>
#include <AutomatedSystemTest/ExtenderTest.h>
#include <AutomatedSystemTest/XForwardTest.h>
#include <AutomatedSystemTest/YBackwardTest.h>
#include <AutomatedSystemTest/YForwardTest.h>

using namespace std;
class AutomatedSystemTest
{
public:
    AutomatedSystemTest();
    ~AutomatedSystemTest() = default;
    void Init();
    void Run();
    void Exit();

private:
    enum TEST_STEP
    {
        BASE_TEST = 0,
        ARM_TEST = 1,
        SWERVE_VX_FORWARD = 2,
        SWERVE_VX_BACKWARD = 3,
        SWERVE_VY_FORWARD = 4,
        SWERVE_VY_BACKWARD = 5,
    };
    TEST_STEP m_testStep;
    int m_stepnum = 1;
    bool m_currentTest;
    bool m_finishedcurrenttest;
    bool m_startedtest;
    // double GetTestPnumatics();
    // double GetPDHTemp();
};