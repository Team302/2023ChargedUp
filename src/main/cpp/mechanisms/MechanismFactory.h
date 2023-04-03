
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

//========================================================================================================
/// MechanismFactory.h
//========================================================================================================
///
/// File Description:
///     This controls the creation of mechanisms/subsystems
///
//========================================================================================================

#pragma once

// C++ Includes
#include <map>
#include <memory>
#include <string>
#include <vector>

// FRC includes

// Team 302 includes
#include <hw/usages/AnalogInputMap.h>
#include <hw/usages/DigitalInputMap.h>
#include <hw/usages/DragonSolenoidMap.h>
#include <hw/usages/IDragonMotorControllerMap.h>
#include <hw/usages/ServoMap.h>
#include <mechanisms/MechanismTypes.h>
#include <mechanisms/base/Mech.h>

// @ADDMECH include for your mechanism
#include <mechanisms/arm/Arm.h>
#include <mechanisms/extender/Extender.h>
#include <mechanisms/intake/Intake.h>
#include <utils/logging/Logger.h>
// Third Party Includes

// forward declares
class DragonAnalogInput;
class DragonCanCoder;
class DragonDigitalInput;
class DragonServo;
class DragonSolenoid;
class IDragonMotorController;
class Mech;

class MechanismFactory
{
public:
	static MechanismFactory *GetMechanismFactory();

	/// @brief    Find or create the requested mechanism
	void CreateMechanism(
		MechanismTypes::MECHANISM_TYPE type,
		std::string networkTableName,
		std::string controlFileName,
		const IDragonMotorControllerMap &motorControllers,
		const DragonSolenoidMap &solenoids,
		const ServoMap &servos,
		const DigitalInputMap &digitalInputs,
		const AnalogInputMap &analogInputs,
		DragonCanCoder *canCoder);

	// @ADDMECH  Add inline Get method for your mechanism
	inline Arm *GetArm() const { return m_arm; }
	inline Extender *GetExtender() const { return m_extender; }
	inline Intake *GetIntake() const
	{
		Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, std::string("IntakeDebugging"), std::string("Getting intake2"), m_intake != nullptr ? "true" : "false");
		return m_intake;
	}

	Mech *GetMechanism(
		MechanismTypes::MECHANISM_TYPE type) const;

private:
	std::shared_ptr<IDragonMotorController> GetMotorController(const IDragonMotorControllerMap &motorcontrollers, MotorControllerUsage::MOTOR_CONTROLLER_USAGE usage);
	std::shared_ptr<DragonSolenoid> GetSolenoid(const DragonSolenoidMap &solenoids, SolenoidUsage::SOLENOID_USAGE usage);
	DragonServo *GetServo(const ServoMap &servos, ServoUsage::SERVO_USAGE usage);
	std::shared_ptr<DragonDigitalInput> GetDigitalInput(const DigitalInputMap &digitaInputs, DigitalInputUsage::DIGITAL_INPUT_USAGE usage);

	DragonAnalogInput *GetAnalogInput(const AnalogInputMap &analogInputs, DragonAnalogInput::ANALOG_SENSOR_TYPE usage);

	MechanismFactory();
	virtual ~MechanismFactory() = default;

	static MechanismFactory *m_mechanismFactory;

	// @ADDMECH  Add your mechanism here
	Arm *m_arm;
	Extender *m_extender;
	Intake *m_intake;
};