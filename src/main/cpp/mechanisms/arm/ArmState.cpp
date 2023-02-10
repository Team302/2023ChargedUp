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
//==============================================================
// This file is auto generated by FRCrobotCodeGen302.exe Version $CODE_GENERATOR_VERSION$
// Changes to this file may cause incorrect behavior and will be lost when
// the code is regenerated, unless the changes are delimited by:
//  //========= Hand modified code start section x ========
//                    Your hand written code goes here
//	//========= Hand modified code end section x ========
//==============================================================

// C++ Includes
#include <string>

// FRC includes

// Team 302 includes
#include <mechanisms/base/Mech1IndMotorState.h>
#include <mechanisms/controllers/ControlData.h>
#include <mechanisms/arm/ArmState.h>
#include <mechanisms/MechanismFactory.h>

// Third Party Includes

using namespace std;

ArmState::ArmState(
	string stateName,
	int stateId,
	ControlData *control0,
	double target0) : Mech1IndMotorState(MechanismFactory::GetMechanismFactory()->GetArm(), stateName, stateId, control0, target0),
					  m_arm(MechanismFactory::GetMechanismFactory()->GetArm())
{
}

bool ArmState::AtTarget() const
{
	/*
	if (m_arm != nullptr)
	{
		if (abs(m_arm->GetPositionDegrees().to<double>() - m_arm->GetTarget()) < 0.5 && m_arm->GetPositionDegrees().to<double>() > 0.5)
		{
			return true;
		}
	}*/

	return true;
}
