
//====================================================================================================================================================
/// Copyright 2023 Lake Orion Robotics FIRST Team 302
///
/// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
/// to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
/// and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
///
/// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
///
/// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
/// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
/// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE
/// OR OTHER DEALINGS IN THE SOFTWARE.
//====================================================================================================================================================

// C++ Includes
#include <map>
#include <memory>

// FRC includes

// Team 302 includes
#include <hw/DragonAnalogInput.h>
#include <hw/DragonCanCoder.h>
#include <hw/DragonDigitalInput.h>
#include <hw/DragonServo.h>
#include <hw/DragonSolenoid.h>
#include <hw/usages/MotorControllerUsage.h>
#include <hw/interfaces/IDragonMotorController.h>
#include <hw/usages/AnalogInputMap.h>
#include <hw/usages/DigitalInputMap.h>
#include <hw/usages/DragonSolenoidMap.h>
#include <hw/usages/IDragonMotorControllerMap.h>
#include <hw/usages/ServoMap.h>
#include <mechanisms/base/Mech.h>
#include <mechanisms/MechanismFactory.h>
#include <mechanisms/MechanismTypes.h>
#include <utils/logging/Logger.h>
// @ADDMECH include for your mechanism
#include <mechanisms/arm/Arm.h>
#include <mechanisms/extender/Extender.h>
#include <mechanisms/intake/Intake.h>
// Third Party Includes
#include <ctre/phoenix/sensors/CANCoder.h>

using namespace std;
using namespace ctre::phoenix::sensors;

//=====================================================================================
/// Method:         GetMechanismFactory
/// Description:    Find or create the mechanism factory
/// Returns:        MechanismFactory* pointer to the mechanism factory
//=====================================================================================
MechanismFactory *MechanismFactory::m_mechanismFactory = nullptr;
MechanismFactory *MechanismFactory::GetMechanismFactory()
{
	if (MechanismFactory::m_mechanismFactory == nullptr)
	{
		MechanismFactory::m_mechanismFactory = new MechanismFactory();
	}
	return MechanismFactory::m_mechanismFactory;
}

MechanismFactory::MechanismFactory() : m_arm(nullptr),
									   m_extender(nullptr),
									   m_intake(nullptr)
// @ADDMECH Initialize mechanism to NULLPTR
{
	Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("IntakeDebugging"), string("intake created"), false);
}

/// @brief      create the requested mechanism
/// @param [in] MechanismTypes::MECHANISM_TYPE  type - the type of mechanism to create
/// @param [in] const IDragonMotorControllerMap& map of the motor usage to the motor controller
/// @param [in]
/// @param [in]
/// @param [in]
/// @param [in]
void MechanismFactory::CreateMechanism(
	MechanismTypes::MECHANISM_TYPE type,
	string networkTableName,
	string controlFileName,
	const IDragonMotorControllerMap &motorControllers,
	const DragonSolenoidMap &solenoids,
	const ServoMap &servos,
	const DigitalInputMap &digitalInputs,
	const AnalogInputMap &analogInputs,
	DragonCanCoder *canCoder)
{

	// Create the mechanism
	switch (type)
	{

		// @ADDMECH  Add case for Mechanism

	case MechanismTypes::ARM:
	{
		auto motor = GetMotorController(motorControllers, MotorControllerUsage::ARM);
		if (motor.get() != nullptr)
		{
			m_arm = new Arm(controlFileName, networkTableName, motor, canCoder);
		}
	}
	break;

	case MechanismTypes::EXTENDER:
	{
		auto motor = GetMotorController(motorControllers, MotorControllerUsage::Extender);
		if (motor.get() != nullptr)
		{
			m_extender = new Extender(controlFileName, networkTableName, motor);
		}
	}
	break;

	case MechanismTypes::INTAKE:
	{
		auto intakeSol = GetSolenoid(solenoids, SolenoidUsage::IntakeSolenoid);
		auto gamePiecePresent = GetDigitalInput(digitalInputs, DigitalInputUsage::GAME_PIECE_PRESENT_SENSOR);
		auto motor = GetMotorController(motorControllers, MotorControllerUsage::INTAKE1);
		auto motor2 = GetMotorController(motorControllers, MotorControllerUsage::INTAKE2);
		if (intakeSol.get() != nullptr && gamePiecePresent.get() != nullptr && motor.get() != nullptr && motor2.get() != nullptr)
		{
			Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("IntakeDebugging"), string("components"), "not nullptr");
			m_intake = new Intake(controlFileName, networkTableName, motor, motor2, intakeSol, gamePiecePresent);
			Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("IntakeDebugging"), string("intake created"), true);
		}
		else
		{
			Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("IntakeDebugging"), string("components"), "nullptr");
		}
	}
	break;

	default:
	{
		string msg = "unknown Mechanism type ";
		msg += to_string(type);
		Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, string("MechanismFactory"), string("CreateMechanism"), msg);
	}
	break;
	}
}

Mech *MechanismFactory::GetMechanism(MechanismTypes::MECHANISM_TYPE type) const
{
	switch (type)
	{

		// @ADDMECH  Add case for Mechanism

	case MechanismTypes::ARM:
		return m_arm;
		break;

	case MechanismTypes::EXTENDER:
		return m_extender;
		break;

	case MechanismTypes::INTAKE:
		Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("IntakeDebugging"), string("Getting intake"), m_intake != nullptr ? "true" : "false");
		return m_intake;
		break;
	default:
		return nullptr;
		break;
	}
	return nullptr;
}

shared_ptr<IDragonMotorController> MechanismFactory::GetMotorController(const IDragonMotorControllerMap &motorControllers, MotorControllerUsage::MOTOR_CONTROLLER_USAGE usage)
{
	shared_ptr<IDragonMotorController> motor;
	auto it = motorControllers.find(usage);
	if (it != motorControllers.end()) // found it
	{
		motor = it->second;
	}
	else
	{
		string msg = "motor not found; usage = ";
		msg += to_string(usage);
		Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, string("MechanismFactory"), string("GetMotorController"), msg);
	}

	if (motor.get() == nullptr)
	{
		string msg = "motor is nullptr; usage = ";
		msg += to_string(usage);
		Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, string("MechanismFactory"), string("GetMotorController"), msg);
	}
	return motor;
}

shared_ptr<DragonSolenoid> MechanismFactory::GetSolenoid(const DragonSolenoidMap &solenoids, SolenoidUsage::SOLENOID_USAGE usage)
{
	shared_ptr<DragonSolenoid> solenoid;
	auto it = solenoids.find(usage);
	if (it != solenoids.end()) // found it
	{
		solenoid = it->second;
	}
	else
	{
		string msg = "solenoid not found; usage = ";
		msg += to_string(usage);
		Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, string("MechanismFactory"), string("GetSolenoid"), msg);
	}

	if (solenoid.get() == nullptr)
	{
		string msg = "solenoid is nullptr; usage = ";
		msg += to_string(usage);
		Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, string("MechanismFactory"), string("GetSolenoid"), msg);
	}
	return solenoid;
}
DragonServo *MechanismFactory::GetServo(const ServoMap &servos, ServoUsage::SERVO_USAGE usage)
{
	DragonServo *servo = nullptr;
	auto it = servos.find(usage);
	if (it != servos.end()) // found it
	{
		servo = it->second;
	}
	else
	{
		string msg = "servo not found; usage = ";
		msg += to_string(usage);
		Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, string("MechanismFactory"), string("GetServo"), msg);
	}

	if (servo == nullptr)
	{
		string msg = "servo is nullptr; usage = ";
		msg += to_string(usage);
		Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, string("MechanismFactory"), string("GetServo"), msg);
	}
	return servo;
}
shared_ptr<DragonDigitalInput> MechanismFactory::GetDigitalInput(const DigitalInputMap &digitaInputs, DigitalInputUsage::DIGITAL_INPUT_USAGE usage)
{
	shared_ptr<DragonDigitalInput> dio;
	auto it = digitaInputs.find(usage);
	if (it != digitaInputs.end()) // found it
	{
		dio = it->second;
	}
	else
	{
		string msg = "digital input not found; usage = ";
		msg += to_string(usage);
		Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, string("MechanismFactory"), string("GetDigitalInput"), msg);
	}

	if (dio.get() == nullptr)
	{
		string msg = "digital input is nullptr; usage = ";
		msg += to_string(usage);
		Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, string("MechanismFactory"), string("GetDigitalInput"), msg);
	}
	return dio;
}

DragonAnalogInput *MechanismFactory::GetAnalogInput(const AnalogInputMap &analogInputs, DragonAnalogInput::ANALOG_SENSOR_TYPE usage)
{
	DragonAnalogInput *anIn = nullptr;
	auto it = analogInputs.find(usage);
	if (it != analogInputs.end()) // found it
	{
		anIn = it->second;
	}
	else
	{
		string msg = "analog input not found; usage = ";
		msg += to_string(usage);
		Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, string("MechanismFactory"), string("GetAnalogInput"), msg);
	}

	if (anIn == nullptr)
	{
		string msg = "analog input is nullptr; usage = ";
		msg += to_string(usage);
		Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, string("MechanismFactory"), string("GetAnalogInput"), msg);
	}
	return anIn;
}
