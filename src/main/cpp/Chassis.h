// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <numbers>

#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>

#include <ctre/phoenix/sensors/WPI_Pigeon2.h>

#include <SwerveModule.h>

/**
 * Represents a swerve drive style drivetrain.
 */
class Chassis
{
public:
    Chassis();

    void Drive(units::meters_per_second_t xSpeed,
               units::meters_per_second_t ySpeed, units::radians_per_second_t rot,
               bool fieldRelative);
    void UpdateOdometry();

    static constexpr units::meters_per_second_t kMaxSpeed =
        3.0_mps; // 3 meters per second
    static constexpr units::radians_per_second_t kMaxAngularSpeed{
        std::numbers::pi}; // 1/2 rotation per second

private:
    const int PIGEON_CANID = 0;
    const std::string CANBUS = "canivore";

    const units::length::meter_t CHASSIS_TRACK = units::length::meter_t(20.25);     // x direction
    const units::length::meter_t CHASSIS_WHEELBASE = units::length::meter_t(20.25); // y direction

    frc::Translation2d m_frontLeftLocation{CHASSIS_WHEELBASE, CHASSIS_TRACK};
    frc::Translation2d m_frontRightLocation{CHASSIS_WHEELBASE, -CHASSIS_TRACK};
    frc::Translation2d m_backLeftLocation{-CHASSIS_WHEELBASE, CHASSIS_TRACK};
    frc::Translation2d m_backRightLocation{-CHASSIS_WHEELBASE, -CHASSIS_TRACK};

    const int frCancoderID = 0;
    const int flCancoderID = 0;
    const int brCancoderID = 0;
    const int blCancoderID = 0;

    const double frCancoderOffset = 0.0;
    const double flCancoderOffset = 0.0;
    const double brCancoderOffset = 0.0;
    const double blCancoderOffset = 0.0;

    double countsOnTurnEncoderPerDegree = -120.9325;

    const int frDriveCANID = 0;
    const int frTurnCANID = 0;
    const int flDriveCANID = 0;
    const int flTurnCANID = 0;
    const int brDriveCANID = 0;
    const int brTurnCANID = 0;
    const int blDriveCANID = 0;
    const int blTurnCANID = 0;

    SwerveModule *m_frontLeft;
    SwerveModule *m_frontRight;
    SwerveModule *m_backLeft;
    SwerveModule *m_backRight;

    ctre::phoenix::sensors::WPI_Pigeon2 m_pigeon{PIGEON_CANID, CANBUS};

    frc::SwerveDriveKinematics<4>
        m_kinematics{
            m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation,
            m_backRightLocation};

    frc::SwerveDriveOdometry<4> *m_odometry = nullptr;
};