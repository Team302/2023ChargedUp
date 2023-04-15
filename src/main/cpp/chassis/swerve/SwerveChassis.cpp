
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
#include <iostream>
#include <map>
#include <memory>
#include <cmath>

// FRC includes
#include <frc/DriverStation.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Transform2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/velocity.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveDriveKinematics.h>

// Team 302 includes
#include <chassis/PoseEstimatorEnum.h>
#include <chassis/swerve/SwerveChassis.h>

#include <chassis/swerve/driveStates/AutoBalanceDrive.h>
#include <chassis/swerve/driveStates/FieldDrive.h>
#include <chassis/swerve/driveStates/VisionDrive.h>
#include <chassis/swerve/driveStates/HoldDrive.h>
#include <chassis/swerve/driveStates/RobotDrive.h>
#include <chassis/swerve/driveStates/StopDrive.h>
#include <chassis/swerve/driveStates/TrajectoryDrive.h>
#include <chassis/swerve/driveStates/TrajectoryDrivePathPlanner.h>
#include <chassis/swerve/driveStates/VisionDrive.h>

#include <chassis/swerve/headingStates/FaceGoalHeading.h>
#include <chassis/swerve/headingStates/FaceGamePiece.h>
#include <chassis/swerve/headingStates/FaceAprilTag.h>
#include <chassis/swerve/headingStates/ISwerveDriveOrientation.h>
#include <chassis/swerve/headingStates/MaintainHeading.h>
#include <chassis/swerve/headingStates/SpecifiedHeading.h>
#include <chassis/swerve/headingStates/IgnoreHeading.h>

#include <DragonVision/DragonLimelight.h>
#include <DragonVision/LimelightFactory.h>
#include <utils/FMSData.h>
#include <utils/AngleUtils.h>
#include <utils/ConversionUtils.h>
#include <utils/logging/Logger.h>

// Third Party Includes
#include <ctre/phoenix/sensors/CANCoder.h>

constexpr int LEFT_FRONT = 0;
constexpr int RIGHT_FRONT = 1;
constexpr int LEFT_BACK = 2;
constexpr int RIGHT_BACK = 3;

using std::map;
using std::shared_ptr;
using std::string;

using frc::BuiltInAccelerometer;
using frc::ChassisSpeeds;
using frc::Pose2d;
using frc::Rotation2d;
using frc::Transform2d;

/// @brief Construct a swerve chassis
/// @param [in] std::shared_ptr<SwerveModule>           frontleft:          front left swerve module
/// @param [in] std::shared_ptr<SwerveModule>           frontright:         front right swerve module
/// @param [in] std::shared_ptr<SwerveModule>           backleft:           back left swerve module
/// @param [in] std::shared_ptr<SwerveModule>           backright:          back right swerve module
/// @param [in] units::length::inch_t                   wheelBase:          distance between the front and rear wheels
/// @param [in] units::length::inch_t                   track:              distance between the left and right wheels
/// @param [in] units::velocity::meters_per_second_t    maxSpeed:           maximum linear speed of the chassis
/// @param [in] units::radians_per_second_t             maxAngularSpeed:    maximum rotation speed of the chassis
/// @param [in] double                                  maxAcceleration:    maximum acceleration in meters_per_second_squared
SwerveChassis::SwerveChassis(
    shared_ptr<SwerveModule> frontLeft,
    shared_ptr<SwerveModule> frontRight,
    shared_ptr<SwerveModule> backLeft,
    shared_ptr<SwerveModule> backRight,
    units::length::inch_t wheelDiameter,
    units::length::inch_t wheelBase,
    units::length::inch_t track,
    units::velocity::meters_per_second_t maxSpeed,
    units::radians_per_second_t maxAngularSpeed,
    units::acceleration::meters_per_second_squared_t maxAcceleration,
    units::angular_acceleration::radians_per_second_squared_t maxAngularAcceleration,
    string networkTableName,
    string controlFileName) : m_frontLeft(frontLeft),
                              m_frontRight(frontRight),
                              m_backLeft(backLeft),
                              m_backRight(backRight),
                              m_robotDrive(nullptr),
                              m_flState(),
                              m_frState(),
                              m_blState(),
                              m_brState(),
                              m_wheelDiameter(wheelDiameter),
                              m_wheelBase(wheelBase),
                              m_track(track),
                              m_maxSpeed(maxSpeed),
                              m_maxAngularSpeed(maxAngularSpeed),
                              m_maxAcceleration(maxAcceleration),               // Not used at the moment
                              m_maxAngularAcceleration(maxAngularAcceleration), // Not used at the moment
                              m_pigeon(PigeonFactory::GetFactory()->GetPigeon(DragonPigeon::PIGEON_USAGE::CENTER_OF_ROBOT)),
                              m_accel(BuiltInAccelerometer()),
                              m_runWPI(false),
                              m_poseOpt(PoseEstimatorEnum::WPI),
                              m_pose(),
                              m_offsetPoseAngle(0_deg), // not used at the moment
                              m_drive(units::velocity::meters_per_second_t(0.0)),
                              m_steer(units::velocity::meters_per_second_t(0.0)),
                              m_rotate(units::angular_velocity::radians_per_second_t(0.0)),
                              m_frontLeftLocation(wheelBase / 2.0, track / 2.0),
                              m_frontRightLocation(wheelBase / 2.0, -1.0 * track / 2.0),
                              m_backLeftLocation(-1.0 * wheelBase / 2.0, track / 2.0),
                              m_backRightLocation(-1.0 * wheelBase / 2.0, -1.0 * track / 2.0),
                              m_kinematics(m_frontLeftLocation,
                                           m_frontRightLocation,
                                           m_backLeftLocation,
                                           m_backRightLocation),
                              m_poseEstimator(m_kinematics,
                                              frc::Rotation2d{},
                                              {m_frontLeft.get()->GetPosition(), m_frontRight.get()->GetPosition(), m_backLeft.get()->GetPosition(), m_backRight.get()->GetPosition()},
                                              frc::Pose2d(),
                                              {0.1, 0.1, 0.1},
                                              {0.1, 0.1, 0.1}),
                              m_storedYaw(m_pigeon->GetYaw()),
                              m_yawCorrection(units::angular_velocity::degrees_per_second_t(0.0)),
                              m_targetHeading(units::angle::degree_t(0)),
                              m_vision(DragonVision::GetDragonVision()),
                              m_networkTableName(networkTableName),
                              m_controlFileName(controlFileName)
{
    frontLeft.get()->Init(wheelDiameter, maxSpeed, maxAngularSpeed, maxAcceleration, maxAngularAcceleration, m_frontLeftLocation);
    frontRight.get()->Init(wheelDiameter, maxSpeed, maxAngularSpeed, maxAcceleration, maxAngularAcceleration, m_frontRightLocation);
    backLeft.get()->Init(wheelDiameter, maxSpeed, maxAngularSpeed, maxAcceleration, maxAngularAcceleration, m_backLeftLocation);
    backRight.get()->Init(wheelDiameter, maxSpeed, maxAngularSpeed, maxAcceleration, maxAngularAcceleration, m_backRightLocation);
    ZeroAlignSwerveModules();
}

void SwerveChassis::InitStates()
{
    m_robotDrive = new RobotDrive();

    m_driveStateMap[ChassisOptionEnums::FIELD_DRIVE] = new FieldDrive(m_robotDrive);
    m_driveStateMap[ChassisOptionEnums::HOLD_DRIVE] = new HoldDrive();
    m_driveStateMap[ChassisOptionEnums::ROBOT_DRIVE] = m_robotDrive;
    m_driveStateMap[ChassisOptionEnums::STOP_DRIVE] = new StopDrive(m_robotDrive);
    m_driveStateMap[ChassisOptionEnums::TRAJECTORY_DRIVE] = new TrajectoryDrive(m_robotDrive);
    m_driveStateMap[ChassisOptionEnums::TRAJECTORY_DRIVE_PLANNER] = new TrajectoryDrivePathPlanner(m_robotDrive);
    m_driveStateMap[ChassisOptionEnums::VISION_DRIVE] = new VisionDrive(m_robotDrive);
    m_driveStateMap[ChassisOptionEnums::AUTO_BALANCE] = new AutoBalanceDrive(m_robotDrive);

    m_headingStateMap[ChassisOptionEnums::HeadingOption::MAINTAIN] = new MaintainHeading();
    m_headingStateMap[ChassisOptionEnums::HeadingOption::SPECIFIED_ANGLE] = new SpecifiedHeading();
    m_headingStateMap[ChassisOptionEnums::HeadingOption::FACE_GAME_PIECE] = new FaceGamePiece();
    m_headingStateMap[ChassisOptionEnums::HeadingOption::FACE_APRIL_TAG] = new FaceAprilTag();
    m_headingStateMap[ChassisOptionEnums::HeadingOption::TOWARD_GOAL] = new FaceGoalHeading();
    m_headingStateMap[ChassisOptionEnums::HeadingOption::IGNORE] = new IgnoreHeading();
}
/// @brief Align all of the swerve modules to point forward
void SwerveChassis::ZeroAlignSwerveModules()
{
    m_frontLeft.get()->ZeroAlignModule();
    m_frontRight.get()->ZeroAlignModule();
    m_backLeft.get()->ZeroAlignModule();
    m_backRight.get()->ZeroAlignModule();
}

/// @brief Drive the chassis
void SwerveChassis::Drive(ChassisMovement moveInfo)
{
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("swerve"), string("Vx"), moveInfo.chassisSpeeds.vx.to<double>());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("swerve"), string("Vy"), moveInfo.chassisSpeeds.vy.to<double>());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("swerve"), string("Omega"), moveInfo.chassisSpeeds.omega.to<double>());

    m_currentOrientationState = GetHeadingState(moveInfo);
    if (m_currentOrientationState != nullptr)
    {
        m_currentOrientationState->UpdateChassisSpeeds(moveInfo);
    }

    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("swerve"), string("VxAfterHeading"), moveInfo.chassisSpeeds.vx.to<double>());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("swerve"), string("VyAfterHeading"), moveInfo.chassisSpeeds.vy.to<double>());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("swerve"), string("OmegaAfterHeading"), moveInfo.chassisSpeeds.omega.to<double>());

    m_currentDriveState = GetDriveState(moveInfo);

    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("swerve"), string("m_currentDriveState "), m_currentDriveState != nullptr ? string("not nullptr ") : string("nullptr"));

    if (m_currentDriveState != nullptr)
    {
        auto states = m_currentDriveState->UpdateSwerveModuleStates(moveInfo);
        m_frontLeft.get()->SetDesiredState(states[LEFT_FRONT]);
        m_frontRight.get()->SetDesiredState(states[RIGHT_FRONT]);
        m_backLeft.get()->SetDesiredState(states[LEFT_BACK]);
        m_backRight.get()->SetDesiredState(states[RIGHT_BACK]);
    }
    auto table = nt::NetworkTableInstance::GetDefault().GetTable("Anti-Tip");
    table.get()->PutBoolean(std::string(" "), (moveInfo.checkTipping));
}

void SwerveChassis::Drive()
{
    // No-op for now
}

ISwerveDriveState *SwerveChassis::GetSpecifiedDriveState(ChassisOptionEnums::DriveStateType driveOption)
{
    auto itr = m_driveStateMap.find(driveOption);
    if (itr == m_driveStateMap.end())
    {
        return m_robotDrive;
    }
    return itr->second;
}

ISwerveDriveOrientation *SwerveChassis::GetHeadingState(ChassisMovement moveInfo)
{
    auto itr = m_headingStateMap.find(moveInfo.headingOption);
    if (itr == m_headingStateMap.end())
    {
        itr = m_headingStateMap.find(ChassisOptionEnums::HeadingOption::MAINTAIN);
    }
    return itr->second;
}
ISwerveDriveState *SwerveChassis::GetDriveState(ChassisMovement moveInfo)
{
    ISwerveDriveState *state = nullptr;

    auto isVisionDrive = moveInfo.driveOption == ChassisOptionEnums::VISION_DRIVE;
    auto isAutoBlance = moveInfo.driveOption == ChassisOptionEnums::AUTO_BALANCE;
    auto isHoldDrive = moveInfo.driveOption == ChassisOptionEnums::HOLD_DRIVE;
    auto hasTrajectory = moveInfo.driveOption == ChassisOptionEnums::TRAJECTORY_DRIVE || moveInfo.driveOption == ChassisOptionEnums::TRAJECTORY_DRIVE_PLANNER;

    if (!hasTrajectory && !isVisionDrive && !isAutoBlance && !isHoldDrive &&
        (units::math::abs(moveInfo.chassisSpeeds.vx) < m_velocityDeadband) &&
        (units::math::abs(moveInfo.chassisSpeeds.vy) < m_velocityDeadband) &&
        (units::math::abs(moveInfo.chassisSpeeds.omega) < m_angularDeadband))
    {
        if (moveInfo.noMovementOption == ChassisOptionEnums::NoMovementOption::HOLD_POSITION)
        {
            state = m_driveStateMap[ChassisOptionEnums::HOLD_DRIVE];
        }
        else
        {
            state = m_driveStateMap[ChassisOptionEnums::STOP_DRIVE];
        }
    }
    else
    {
        auto itr = m_driveStateMap.find(moveInfo.driveOption);
        if (itr == m_driveStateMap.end())
        {
            return m_robotDrive;
        }
        state = itr->second;

        if (m_currentDriveState == nullptr)
        {
            m_currentDriveState = m_robotDrive;
        }
    }

    if (state != m_currentDriveState)
    {
        m_initialized = false;
    }

    if (!m_initialized)
    {
        state->Init(moveInfo);
        m_initialized = true;
    }

    return state;
}

Pose2d SwerveChassis::GetPose() const
{
    return m_poseEstimator.GetEstimatedPosition();
}

units::angle::degree_t SwerveChassis::GetYaw() const
{
    units::degree_t yaw{m_pigeon->GetYaw()};
    return yaw;
}

units::angle::degree_t SwerveChassis::GetPitch() const
{
    units::degree_t pitch{m_pigeon->GetPitch()};
    return pitch;
}

units::angle::degree_t SwerveChassis::GetRoll() const
{
    units::degree_t roll{m_pigeon->GetRoll()};
    return roll;
}

/// @brief update the chassis odometry based on current states of the swerve modules and the pigeon
void SwerveChassis::UpdateOdometry()
{
    units::degree_t yaw{m_pigeon->GetYaw()};
    Rotation2d rot2d{yaw};

    /*if (m_vision == nullptr)
    {
        auto targetInfo = m_vision->getTargetInfo();
        if (targetInfo != nullptr)
        {
            auto distToTarget = targetInfo->getDistanceToTarget().to<double>();
            frc::Pose2d pose = m_vision->GetRobotPosition();

            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("UpdateOdometry"), string("DistToTarget"), distToTarget);
            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("UpdateOdometry"), string("HasReset"), m_hasResetToVisionTarget);

            if (distToTarget > 29.5 && !m_hasResetToVisionTarget && distToTarget < 80 && pose.X().to<double>() > 0 && pose.Y().to<double>() > 0) // Need to add low pass filter for all 3 conditions
            {
                m_poseEstimator.ResetPosition(rot2d, wpi::array<frc::SwerveModulePosition, 4>{m_frontLeft.get()->GetPosition(), m_frontRight.get()->GetPosition(), m_backLeft.get()->GetPosition(), m_backRight.get()->GetPosition()}, pose);
                m_hasResetToVisionTarget = true;
            }
            else if (distToTarget < 80 && distToTarget < 200)
            {
                m_poseEstimator.Update(rot2d, wpi::array<frc::SwerveModulePosition, 4>{m_frontLeft.get()->GetPosition(),
                                                                                       m_frontRight.get()->GetPosition(),
                                                                                       m_backLeft.get()->GetPosition(),
                                                                                       m_backRight.get()->GetPosition()});
            }
            else if (distToTarget < 200)
            {
                m_poseEstimator.Update(rot2d, wpi::array<frc::SwerveModulePosition, 4>{m_frontLeft.get()->GetPosition(),
                                                                                       m_frontRight.get()->GetPosition(),
                                                                                       m_backLeft.get()->GetPosition(),
                                                                                       m_backRight.get()->GetPosition()});
                m_hasResetToVisionTarget = false;
            }
        }
    }
    else
    {*/
    m_poseEstimator.Update(rot2d, wpi::array<frc::SwerveModulePosition, 4>{m_frontLeft.get()->GetPosition(),
                                                                           m_frontRight.get()->GetPosition(),
                                                                           m_backLeft.get()->GetPosition(),
                                                                           m_backRight.get()->GetPosition()});
    m_hasResetToVisionTarget = false;
    //}

    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, std::string("SwerveOdometry"), std::string("X Position: "), m_poseEstimator.GetEstimatedPosition().X().to<double>());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, std::string("SwerveOdometry"), std::string("Y Position: "), m_poseEstimator.GetEstimatedPosition().Y().to<double>());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, std::string("SwerveOdometry"), std::string("Rotation: "), m_poseEstimator.GetEstimatedPosition().Rotation().Degrees().to<double>());
}
/// @brief set all of the encoders to zero
void SwerveChassis::SetEncodersToZero()
{
    m_frontLeft.get()->SetEncodersToZero();
    m_frontRight.get()->SetEncodersToZero();
    m_backLeft.get()->SetEncodersToZero();
    m_backRight.get()->SetEncodersToZero();
}

double SwerveChassis::GetEncoderValues(std::shared_ptr<SwerveModule> motor)
{
    return motor.get()->GetEncoderValues();
}

/// @brief Provide the current chassis speed information
ChassisSpeeds SwerveChassis::GetChassisSpeeds() const
{
    return m_kinematics.ToChassisSpeeds({m_frontLeft.get()->GetState(),
                                         m_frontRight.get()->GetState(),
                                         m_backLeft.get()->GetState(),
                                         m_backRight.get()->GetState()});
}

void SwerveChassis::ResetPose(const Pose2d &pose)
{
    units::degree_t yaw{m_pigeon->GetYaw()};
    Rotation2d rot2d{yaw};

    SetEncodersToZero();

    ZeroAlignSwerveModules();

    m_poseEstimator.ResetPosition(rot2d, wpi::array<frc::SwerveModulePosition, 4>{m_frontLeft.get()->GetPosition(), m_frontRight.get()->GetPosition(), m_backLeft.get()->GetPosition(), m_backRight.get()->GetPosition()}, pose);
}

void SwerveChassis::ResetYaw()
{
    units::degree_t yaw{m_pigeon->GetYaw()};
    Rotation2d rot2d{yaw};

    frc::DriverStation::Alliance alliance = FMSData::GetInstance()->GetAllianceColor();

    if (alliance == frc::DriverStation::Alliance::kBlue)
    {
        m_pigeon->ReZeroPigeon(0.0, 0.0);
    }
    else
    {
        m_pigeon->ReZeroPigeon(180.0, 0.0);
    }

    ZeroAlignSwerveModules();
}

ChassisSpeeds SwerveChassis::GetFieldRelativeSpeeds(
    units::meters_per_second_t xSpeed,
    units::meters_per_second_t ySpeed,
    units::radians_per_second_t rot)
{
    units::angle::radian_t yaw(ConversionUtils::DegreesToRadians(m_pigeon->GetYaw()));
    auto temp = xSpeed * cos(yaw.to<double>()) + ySpeed * sin(yaw.to<double>());
    auto strafe = -1.0 * xSpeed * sin(yaw.to<double>()) + ySpeed * cos(yaw.to<double>());
    auto forward = temp;

    ChassisSpeeds output{forward, strafe, rot};
    return output;
}

void SwerveChassis::SetStoredHeading(units::angle::degree_t heading)
{
    m_storedYaw = heading;
}

void SwerveChassis::SetTargetHeading(units::angle::degree_t targetYaw)
{
    m_targetHeading = targetYaw;
}