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
// fix GetCurrent() and other PDP methods.
#include <AutomatedSystemTest/BasePDPTest.h>
#include <utils/logging/Logger.h>
#include <frc/PowerDistribution.h>
using namespace std;
BaseTest::BaseTest()
{
    m_BaseTestDone = false;
    m_InitDone = false;
}
void BaseTest::Init()
{
    m_InitDone = true;
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("Automatedsystemtest"), string("BaseTest (Init)"), "reached");
}
void BaseTest::Run()
{
    m_BaseTestDone = true;
}