
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
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>

// Team 302 includes
#include <mechanisms/base/Mech.h>
#include <mechanisms/base/Mech1IndMotor.h>
#include <mechanisms/base/Mech2Motors1Solenoid.h>
#include <mechanisms/controllers/ControlData.h>
#include <hw/interfaces/IDragonMotorController.h>
#include <utils/logging/Logger.h>

// Third Party Includes
#include <units/time.h>

using namespace std;

/// @brief Create a generic mechanism wiht 2 independent motors
/// @param [in] MechanismTypes::MECHANISM_TYPE the type of mechansim
/// @param [in] std::string the name of the file that will set control parameters for this mechanism
/// @param [in] std::string the name of the network table for logging information
/// @param [in] std::shared_ptr<IDragonMotorController> primary motor used by this mechanism
/// @param [in] std::shared_ptr<IDragonMotorController> secondary motor used by this mechanism
Mech2Motors1Solenoid::Mech2Motors1Solenoid(
    MechanismTypes::MECHANISM_TYPE type,
    std::string controlFileName,
    std::string networkTableName,
    shared_ptr<IDragonMotorController> primaryMotor,
    shared_ptr<IDragonMotorController> secondaryMotor,
    std::shared_ptr<DragonSolenoid> solenoid) : Mech(type, controlFileName, networkTableName),
                                                m_motorMech(new Mech2IndMotors(type, controlFileName, networkTableName, primaryMotor, secondaryMotor)),
                                                m_solenoidMech(new Mech1Solenoid(type, controlFileName, networkTableName, solenoid))
{
}

/// @brief update the output to the mechanism using the current controller and target value(s)
/// @return void
void Mech2Motors1Solenoid::Update()
{
    if (m_motorMech != nullptr)
    {
        m_motorMech->Update();
    }
}

void Mech2Motors1Solenoid::UpdateTargets(double primary, double secondary)
{
    if (m_motorMech != nullptr)
    {
        m_motorMech->UpdateTargets(primary, secondary);
    }
}

/// @brief  Return the current position of the primary motor in the mechanism.  The value is in inches or degrees.
/// @return double	position in inches (translating mechanisms) or degrees (rotating mechanisms)
double Mech2Motors1Solenoid::GetPrimaryPosition() const
{
    return m_motorMech != nullptr ? m_motorMech->GetPrimaryPosition() : 0.0;
}

/// @brief  Return the current position of the secondary motor in the mechanism.  The value is in inches or degrees.
/// @return double	position in inches (translating mechanisms) or degrees (rotating mechanisms)
double Mech2Motors1Solenoid::GetSecondaryPosition() const
{
    return m_motorMech != nullptr ? m_motorMech->GetSecondaryPosition() : 0.0;
}

/// @brief  Get the current speed of the primary motor in the mechanism.  The value is in inches per second or degrees per second.
/// @return double	speed in inches/second (translating mechanisms) or degrees/second (rotating mechanisms)
double Mech2Motors1Solenoid::GetPrimarySpeed() const
{
    return m_motorMech != nullptr ? m_motorMech->GetPrimarySpeed() : 0.0;
}

/// @brief  Get the current speed of the secondary motor in the mechanism.  The value is in inches per second or degrees per second.
/// @return double	speed in inches/second (translating mechanisms) or degrees/second (rotating mechanisms)
double Mech2Motors1Solenoid::GetSecondarySpeed() const
{
    return m_motorMech != nullptr ? m_motorMech->GetSecondarySpeed() : 0.0;
}

/// @brief  Set the control constants (e.g. PIDF values).
/// @param [in] ControlData*                                   pid:  the control constants
/// @return void
void Mech2Motors1Solenoid::SetControlConstants(int slot, ControlData *pid)
{
    if (m_motorMech != nullptr)
    {
        m_motorMech->SetControlConstants(slot, pid);
    }
}
void Mech2Motors1Solenoid::SetSecondaryControlConstants(int slot, ControlData *pid)
{
    if (m_motorMech != nullptr)
    {
        m_motorMech->SetSecondaryControlConstants(slot, pid);
    }
}

/// @brief      Activate/deactivate pneumatic solenoid
/// @param [in] bool - true == extend, false == retract
/// @return     void
void Mech2Motors1Solenoid::ActivateSolenoid(bool activate)
{
    if (m_solenoidMech != nullptr)
    {
        m_solenoidMech->ActivateSolenoid(activate);
    }
}

/// @brief      Check if the pneumatic solenoid is activated
/// @return     bool - true == extended, false == retracted
bool Mech2Motors1Solenoid::IsSolenoidActivated() const
{
    if (m_solenoidMech != nullptr)
    {
        return m_solenoidMech->IsSolenoidActivated();
    }
    return false;
}

/// @brief log data to the network table if it is activated and time period has past
void Mech2Motors1Solenoid::LogInformation() const
{
    if (m_motorMech != nullptr)
    {
        m_motorMech->LogInformation();
    }
}
