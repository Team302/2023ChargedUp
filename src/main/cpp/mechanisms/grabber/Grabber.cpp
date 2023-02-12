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
#include <memory>
#include <string>

// team 302 includes
#include <hw/interfaces/IDragonMotorController.h>
#include <mechanisms/base/Mech2Solenoids.h>
#include <mechanisms/grabber/grabber.h>

// Third Party Includes
//========= Hand modified code start section 1 ========

//========= Hand modified code end section 1 ========

using namespace std;

/// @brief Create an Grabber mechanism wiht 1 independent motor
/// @param [in] std::string the name of the file that will set control parameters for this mechanism
/// @param [in] std::string the name of the network table for logging information
/// @param [in] std::shared_ptr<IDragonMotorController>

Grabber::Grabber(
	std::string controlFileName,
	std::string networkTableName,
	std::shared_ptr<DragonSolenoid> solenoid0,
	std::shared_ptr<DragonSolenoid> solenoid1) : Mech2Solenoids(MechanismTypes::MECHANISM_TYPE::GRABBER, controlFileName, networkTableName, solenoid0, solenoid1)
{
}

//========= Hand modified code start section 0 ========

//========= Hand modified code end section 0 ========
