
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

// C++ Includes
#include <memory>
#include <string>

// FRC includes
#include <frc/Timer.h>

// Team 302 includes
#include <auton/drivePrimitives/DriveStop.h>
#include <auton/PrimitiveParams.h>
#include <auton/drivePrimitives/IPrimitive.h>
#include <mechanisms/MechanismFactory.h>
#include <mechanisms/controllers/ControlModes.h>

// Third Party Includes

using namespace std;
using namespace frc;

#include <auton/drivePrimitives/DriveDistance.h>
#include <auton/drivePrimitives/DriveToTarget.h>
#include <auton/PrimitiveParams.h>
#include <hw/interfaces/IDragonDistanceSensor.h>
#include <utils/logging/Logger.h>

DriveToTarget::DriveToTarget() : m_sensor(nullptr),
                                 m_underDistanceCounts(0),
                                 m_minTimeToRun(0)
{
}

void DriveToTarget::Init(
    PrimitiveParams *params)
{
    if (m_sensor != nullptr)
    {
        params->SetDistance(m_sensor->GetDistance());
        m_minTimeToRun = 0.3;
        m_underDistanceCounts = 0;

        DriveDistance::Init(params);
    }
    else
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, string("DriveToTarget"), string("DriveToTarget"), string("No Lidar"));
    }
}

void DriveToTarget::Run()
{
    if (m_sensor != nullptr)
    {
        DriveDistance::Run();
        if (m_minTimeToRun <= 0)
        {
            if (m_sensor->GetDistance() <= MIN_CUBE_DISTANCE)
            {
                m_underDistanceCounts++;
            }
        }

        m_minTimeToRun -= IPrimitive::LOOP_LENGTH;
    }
}

bool DriveToTarget::IsDone()
{
    bool done = m_underDistanceCounts >= UNDER_DISTANCE_COUNT_THRESHOLD;

    return (DriveDistance::IsDone() || done);
}
