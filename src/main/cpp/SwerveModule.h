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

#pragma once

// C++ Includes
#include <memory>
#include <string>

// FRC Includes
#include <frc/Encoder.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveModulePosition.h>

#include <frc/kinematics/SwerveModuleState.h>

#include <units/acceleration.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>

// Team 302 Includes
#include <hw/DragonCanCoder.h>

#include <ctre/phoenix/motorcontrol/can/WPI_TalonFX.h>

using namespace ctre::phoenix::motorcontrol::can;

class SwerveModule
{
public:
    /// @brief Constructs a Swerve Module.  This is assuming 2 TalonFX (Falcons) with a CanCoder for the turn angle
    /// @param [in] ModuleID                                                type:           Which Swerve Module is it
    /// @param [in] shared_ptr<IDragonMotorController>                      driveMotor:     Motor that makes the robot move
    /// @param [in] shared_ptr<IDragonMotorController>                      turnMotor:      Motor that turns the swerve module
    /// @param [in] DragonCanCoder*       		                            canCoder:       Sensor for detecting the angle of the wheel
    SwerveModule(WPI_TalonFX *driveMotor,
                 WPI_TalonFX *turningMotor,
                 DragonCanCoder *canCoder,
                 double countsOnTurnEncoderPerDegreesOnAngleSensor);

    /// @brief Get the current state of the module (speed of the wheel and angle of the wheel)
    /// @returns SwerveModuleState
    frc::SwerveModuleState GetState() const;
    frc::SwerveModulePosition GetPosition() const;

    /// @brief Set the current state of the module (speed of the wheel and angle of the wheel)
    /// @param [in] const SwerveModuleState& referenceState:   state to set the module to
    /// @returns void
    void SetDesiredState(const frc::SwerveModuleState &state);

    units::length::inch_t GetWheelDiameter() const { return m_wheelDiameter; }

private:
    const int COUNTS_PER_REV = 2048;

    void SetDriveSpeed(units::velocity::meters_per_second_t speed);
    void SetTurnAngle(units::angle::degree_t angle);

    WPI_TalonFX *m_driveMotor;
    WPI_TalonFX *m_turnMotor;
    DragonCanCoder *m_turnSensor;

    const units::length::inch_t m_wheelDiameter = units::length::inch_t(4.0);

    double m_countsOnTurnEncoderPerDegreesOnAngleSensor;
};