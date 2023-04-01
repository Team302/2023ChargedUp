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
        4.0_mps; // 4 meters per second
    static constexpr units::radians_per_second_t kMaxAngularSpeed{
        std::numbers::pi}; // 1/2 rotation per second

private:
    const int PIGEON_CANID = 0;
    const std::string CANBUS = "canivore";

    const units::length::meter_t CHASSIS_TRACK = units::length::inch_t(20.25);     // x direction
    const units::length::meter_t CHASSIS_WHEELBASE = units::length::inch_t(20.25); // y direction

    frc::Translation2d m_frontLeftLocation{CHASSIS_WHEELBASE, CHASSIS_TRACK};
    frc::Translation2d m_frontRightLocation{CHASSIS_WHEELBASE, -CHASSIS_TRACK};
    frc::Translation2d m_backLeftLocation{-CHASSIS_WHEELBASE, CHASSIS_TRACK};
    frc::Translation2d m_backRightLocation{-CHASSIS_WHEELBASE, -CHASSIS_TRACK};

    const int kFRCancoderID = 0;
    const int kFLCancoderID = 0;
    const int kBRCancoderID = 0;
    const int kBLCancoderID = 0;

    const double kFRCancoderOffset = 0.0;
    const double kFLCancoderOffset = 0.0;
    const double kBRCancoderOffset = 0.0;
    const double kBLCancoderOffset = 0.0;

    const double kCountsOnTurnEncoderPerDegree = -120.9325;

    const int kFRDriveCANID = 0;
    const int kFRTurnCANID = 0;
    const int kFLDriveCANID = 0;
    const int kFLTurnCANID = 0;
    const int kBRDriveCANID = 0;
    const int kBRTurnCANID = 0;
    const int kBLDriveCANID = 0;
    const int kBLTurnCANID = 0;

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