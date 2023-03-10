
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

// Includes
// Team302 includes
#include <auton/drivePrimitives/DriveTime.h>
#include <auton/PrimitiveFactory.h>
#include <auton/PrimitiveParams.h>
#include <mechanisms/MechanismFactory.h>

DriveTime::DriveTime() : SuperDrive(),
						 m_timeRemaining(0.0) // Value will changed in init

{
}

void DriveTime::Init(PrimitiveParams *params)
{
	SuperDrive::Init(params);
	// Get timeRemaining from m_params
	m_timeRemaining = params->GetTime();
}

void DriveTime::Run()
{
	SuperDrive::Run();
}

bool DriveTime::IsDone()
{
	m_timeRemaining -= LOOP_LENGTH;					   // Decrement time remaining
	return ((m_timeRemaining <= (LOOP_LENGTH / 2.0))); // Return true when time runs out
}
