// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <Chassis.h>

Chassis::Chassis()
{
    DragonCanCoder *flCancoder = new DragonCanCoder(kFLCancoderID,
                                                    CANBUS,
                                                    kFLCancoderOffset, false);
    DragonCanCoder *frCancoder = new DragonCanCoder(kFRCancoderID,
                                                    CANBUS,
                                                    kFRCancoderOffset, false);
    DragonCanCoder *blCancoder = new DragonCanCoder(kBLCancoderID,
                                                    CANBUS,
                                                    kBLCancoderOffset, false);
    DragonCanCoder *brCancoder = new DragonCanCoder(kBRCancoderID,
                                                    CANBUS,
                                                    kBRCancoderOffset, false);

    m_frontLeft = new SwerveModule(new WPI_TalonFX(kFLDriveCANID, CANBUS),
                                   new WPI_TalonFX(kFLTurnCANID, CANBUS),
                                   flCancoder,
                                   kCountsOnTurnEncoderPerDegree);

    m_frontRight = new SwerveModule(new WPI_TalonFX(kFRDriveCANID, CANBUS),
                                    new WPI_TalonFX(kFRTurnCANID, CANBUS),
                                    frCancoder,
                                    kCountsOnTurnEncoderPerDegree);

    m_backLeft = new SwerveModule(new WPI_TalonFX(kBLDriveCANID, CANBUS),
                                  new WPI_TalonFX(kBLTurnCANID, CANBUS),
                                  blCancoder,
                                  kCountsOnTurnEncoderPerDegree);

    m_backRight = new SwerveModule(new WPI_TalonFX(kBRDriveCANID, CANBUS),
                                   new WPI_TalonFX(kBRTurnCANID, CANBUS),
                                   brCancoder,
                                   kCountsOnTurnEncoderPerDegree);

    m_pigeon.ConfigFactoryDefault();

    m_odometry = new frc::SwerveDriveOdometry<4>(m_kinematics,
                                                 frc::Rotation2d(units::angle::degree_t(m_pigeon.GetYaw())),
                                                 {m_frontLeft->GetPosition(), m_frontRight->GetPosition(),
                                                  m_backLeft->GetPosition(), m_backRight->GetPosition()});
}

void Chassis::Drive(units::meters_per_second_t xSpeed,
                    units::meters_per_second_t ySpeed,
                    units::radians_per_second_t rot, bool fieldRelative)
{
    auto states = m_kinematics.ToSwerveModuleStates(
        fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                            xSpeed, ySpeed, rot, frc::Rotation2d(units::angle::degree_t(m_pigeon.GetYaw())))
                      : frc::ChassisSpeeds{xSpeed, ySpeed, rot});

    m_kinematics.DesaturateWheelSpeeds(&states, kMaxSpeed);

    auto [fl, fr, bl, br] = states;

    m_frontLeft->SetDesiredState(fl);
    m_frontRight->SetDesiredState(fr);
    m_backLeft->SetDesiredState(bl);
    m_backRight->SetDesiredState(br);
}

void Chassis::UpdateOdometry()
{
    m_odometry->Update(frc::Rotation2d(units::angle::degree_t(m_pigeon.GetYaw())),
                       {m_frontLeft->GetPosition(), m_frontRight->GetPosition(),
                        m_backLeft->GetPosition(), m_backRight->GetPosition()});
}