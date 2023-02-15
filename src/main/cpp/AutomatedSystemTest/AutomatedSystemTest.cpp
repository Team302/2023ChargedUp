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
// author: not charlie writer of bad code
#include <AutomatedSystemTest/AutomatedSystemTest.h>
#include <frc/PowerDistribution.h>
#include <hw/factories/PDPFactory.h>
#include <utils/logging/Logger.h>
#include <hal/PowerDistribution.h>

using namespace std;

void AutomatedSystemTest::Init()
{
    double InitialPDPWatts = GetBasePDHValue();
}
double AutomatedSystemTest::GetBasePDHValue()
{
    auto PDP = PDPFactory::GetFactory()->GetPDP();
    if (PDP != nullptr)
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, string("automatedsystemtest"), string("test"), InitialPDPWatts);
        return PDP->GetTotalCurrent();
    }
    Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, string("automatedsystemtest"), string("pdh energy"), "cannot accses pdh");
    return 0;
}

double AutomatedSystemTest::GetCurrentPDHValue()
{
    ArmStateMgr::GetInstance()->SetCurrentState(ArmStateMgr::ARM_STATE::STARTING_POSITION_ROTATE, true);
    auto armstate = ArmStateMgr::GetInstance()->GetCurrentState();
    if (armstate == ArmStateMgr::ARM_STATE::STARTING_POSITION_ROTATE)
    {
        (reachedcurrentstate = true);
    }

    auto PDP = PDPFactory::GetFactory()->GetPDP();
    if (PDP != nullptr)
    {
        return PDP->GetTotalPower();
    }
    Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, string("automatedsystemtest"), string("test2"), "cannot accses pdh");
    return 0;
    ArmStateMgr::GetInstance()->SetCurrentState(ArmStateMgr::ARM_STATE::CONE_BACKROW_ROTATE, true);
    if (PDP != nullptr)
    {
        return PDP->GetTotalPower();
    }
    Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, string("automatedsystemtest"), string("test1"), "cannot accses pdh");
    return 0;
}