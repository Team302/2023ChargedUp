
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
#include <numbers>
#include <string>

// FRC includes
#include <frc/geometry/Rotation2d.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc/controller/PIDController.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <units/angle.h>
#include <units/length.h>
#include <units/velocity.h>

// Team 302 includes
#include <SwerveModule.h>
#include <hw/DragonCanCoder.h>
#include <utils/AngleUtils.h>

// Third Party Includes
#include <ctre/phoenix/motorcontrol/can/WPI_TalonFX.h>
#include <ctre/phoenix/sensors/CANCoder.h>

using namespace std;
using namespace frc;
using namespace ctre::phoenix;
using namespace ctre::phoenix::motorcontrol::can;
using namespace ctre::phoenix::sensors;

/// @brief Constructs a Swerve Module.  This is assuming 2 TalonFX (Falcons) with a CanCoder for the turn angle
/// @param [in] ModuleID                                                type:           Which Swerve Module is it
/// @param [in] shared_ptr<IDragonMotorController>                      driveMotor:     Motor that makes the robot move
/// @param [in] shared_ptr<IDragonMotorController>                      turnMotor:      Motor that turns the swerve module
/// @param [in] DragonCanCoder*                                 		canCoder:       Sensor for detecting the angle of the wheel
/// @param [in] units::length::inch_t                                   wheelDiameter   Diameter of the wheel
SwerveModule::SwerveModule(
    WPI_TalonFX *driveMotor,
    WPI_TalonFX *turnMotor,
    DragonCanCoder *canCoder,
    double countsOnTurnEncoderPerDegreesOnAngleSensor) : m_driveMotor(driveMotor),
                                                         m_turnMotor(turnMotor),
                                                         m_turnSensor(canCoder),
                                                         m_countsOnTurnEncoderPerDegreesOnAngleSensor(countsOnTurnEncoderPerDegreesOnAngleSensor)
{

    // Set up the Drive Motor

    auto fx = m_driveMotor;

    fx->ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::IntegratedSensor, 0, 10);
    fx->ConfigIntegratedSensorInitializationStrategy(BootToZero);
    auto driveMotorSensors = fx->GetSensorCollection();
    driveMotorSensors.SetIntegratedSensorPosition(0, 0);

    // Set up the Absolute Turn Sensor
    m_turnSensor->ConfigAbsoluteSensorRange(AbsoluteSensorRange::Signed_PlusMinus180, 0);
    m_turnSensor->GetAbsolutePosition();

    // Set up the Turn Motor
    fx = m_turnMotor;
    fx->ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::IntegratedSensor, 0, 10);
    fx->ConfigIntegratedSensorInitializationStrategy(BootToZero);
    auto turnMotorSensors = fx->GetSensorCollection();
    turnMotorSensors.SetIntegratedSensorPosition(0, 0);
}

/// @brief Get the current state of the module (speed of the wheel and angle of the wheel)
/// @returns SwerveModuleState
SwerveModuleState SwerveModule::GetState() const
{
    // Get the Module Drive Motor Speed
    auto mpr = units::length::meter_t(GetWheelDiameter() * numbers::pi);
    auto mps = units::velocity::meters_per_second_t(mpr.to<double>() * (m_driveMotor->GetSelectedSensorVelocity() / COUNTS_PER_REV)); // divide by counts per rev

    // Get the Module Current Rotation Angle
    Rotation2d angle{units::angle::degree_t(m_turnSensor->GetAbsolutePosition())};

    // Create the state and return it
    SwerveModuleState state{mps, angle};
    return state;
}

/// @brief Get the current position of the swerve module (distance and rotation)
/// @return frc::SwerveModulePosition - current position
frc::SwerveModulePosition SwerveModule::GetPosition() const
{
    return {(m_driveMotor->GetSelectedSensorPosition() / COUNTS_PER_REV) * m_wheelDiameter * numbers::pi, // distance travled by drive motor
            Rotation2d(units::angle::degree_t(m_turnSensor->GetAbsolutePosition()))};                     // angle of the swerve module from sensor
}

/// @brief Set the current state of the module (speed of the wheel and angle of the wheel)
/// @param [in] const SwerveModuleState& targetState:   state to set the module to
/// @returns void
void SwerveModule::SetDesiredState(
    const SwerveModuleState &targetState)
{
    // Update targets so the angle turned is less than 90 degrees
    // If the desired angle is less than 90 degrees from the target angle (e.g., -90 to 90 is the amount of turn), just use the angle and speed values
    // if it is more than 90 degrees (90 to 270), the can turn the opposite direction -- increase the angle by 180 degrees -- and negate the wheel speed
    // finally, get the value between -90 and 90
    Rotation2d currAngle = Rotation2d(units::angle::degree_t(m_turnSensor->GetAbsolutePosition()));
    auto optimizedState = frc::SwerveModuleState::Optimize(targetState, currAngle.Degrees());
    // Set Turn Target
    SetTurnAngle(optimizedState.angle.Degrees());

    // Set Drive Target
    SetDriveSpeed(optimizedState.speed);
}

/// @brief Run the swerve module at the same speed and angle
/// @returns void

/// @brief Turn the swerve module to a specified angle
/// @param [in] units::angle::degree_t the target angle to turn the wheel to
/// @returns void
void SwerveModule::SetTurnAngle(units::angle::degree_t targetAngle)
{
    auto currAngle = units::angle::degree_t(m_turnSensor->GetAbsolutePosition());
    auto deltaAngle = AngleUtils::GetDeltaAngle(currAngle, targetAngle);

    if (abs(deltaAngle.to<double>()) > 1.0)
    {
        auto fx = m_turnMotor;
        auto sensors = fx->GetSensorCollection();
        auto deltaTicks = m_countsOnTurnEncoderPerDegreesOnAngleSensor * deltaAngle.to<double>();

        double currentTicks = sensors.GetIntegratedSensorPosition();
        double desiredTicks = currentTicks + deltaTicks;

        m_turnMotor->Set(desiredTicks);
    }
}