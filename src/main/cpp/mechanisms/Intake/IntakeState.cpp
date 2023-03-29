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
#include <mechanisms/base/Mech2Motors1Solenoid.h>
#include <mechanisms/base/Mech2Motors1SolenoidState.h>
#include <mechanisms/controllers/ControlData.h>
#include <mechanisms/intake/IntakeState.h>
#include <mechanisms/MechanismFactory.h>
#include <utils/logging/Logger.h>

// Third Party Includes

using namespace std;

IntakeState::IntakeState(string stateName, int stateId, ControlData *control, ControlData *control2, double primaryTarget, double secondaryTarget, MechanismTargetData::SOLENOID solState) : Mech2Motors1SolenoidState(MechanismFactory::GetMechanismFactory()->GetIntake(), stateName, stateId, control, control2, primaryTarget, secondaryTarget, solState)
{
    string identifier(stateName);
    identifier += to_string(stateId);

    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string(string("IntakeDebugging")), identifier, "creating");
    identifier += string(" target");
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("IntakeDebugging"), identifier, primaryTarget);
    identifier += string(" 2");
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("IntakeDebugging"), identifier, secondaryTarget);
}

bool IntakeState::AtTarget() const
{
    //========= Hand modified code start section 0 ========
    //========= Hand modified code end section 0 ========
    return true;
}
