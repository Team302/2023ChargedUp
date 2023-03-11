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

#include <frc/kinematics/SwerveModuleState.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/pressure.h>
#include <frc/Compressor.h>

// Team302 Includes
#include <chassis/swerve/SwerveChassis.h>
#include <chassis/swerve/driveStates/ISwerveDriveState.h>
#include <chassis/ChassisMovement.h>

class RobotDrive : public ISwerveDriveState
{
public:
    RobotDrive();
    ~RobotDrive() = default;

    std::array<frc::SwerveModuleState, 4> UpdateSwerveModuleStates(
        ChassisMovement &chassisMovement) override;

    void Init(ChassisMovement &chassisMovement) override;

    // Compressor Limit
    bool CompressorSpeedLimit(ChassisMovement, double);

protected:
    frc::SwerveModuleState m_flState;
    frc::SwerveModuleState m_frState;
    frc::SwerveModuleState m_blState;
    frc::SwerveModuleState m_brState;

    units::length::inch_t m_wheelbase;
    units::length::inch_t m_wheeltrack;
    units::velocity::feet_per_second_t m_maxspeed;

private:
    void CorrectForTipping(ChassisMovement &chassisMovement);
    frc::Compressor *m_compressor;
    units::pressure::pounds_per_square_inch_t m_minPressure;
    units::pressure::pounds_per_square_inch_t m_maxPressure;
};