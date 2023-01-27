
//====================================================================================================================================================
// Copyright 2022 Lake Orion Robotics FIRST Team 302
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

#include <chassis/swerve/driveStates/FieldDrive.h>
#include <chassis/swerve/driveStates/HoldDrive.h>
#include <chassis/swerve/driveStates/RobotDrive.h>
#include <chassis/swerve/driveStates/StopDrive.h>
#include <chassis/swerve/driveStates/TrajectoryDrive.h>

#include <chassis/swerve/headingStates/FaceGoalHeading.h>
#include <chassis/swerve/headingStates/ISwerveDriveOrientation.h>
#include <chassis/swerve/headingStates/MaintainHeading.h>
#include <chassis/swerve/headingStates/SpecifiedHeading.h>

#include <hw/DragonLimelight.h>
#include <hw/factories/LimelightFactory.h>
#include <utils/AngleUtils.h>
#include <utils/ConversionUtils.h>
#include <utils/Logger.h>

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
SwerveChassis::SwerveChassis
(
    shared_ptr<SwerveModule>                                    frontLeft, 
    shared_ptr<SwerveModule>                                    frontRight,
    shared_ptr<SwerveModule>                                    backLeft, 
    shared_ptr<SwerveModule>                                    backRight, 
    units::length::inch_t                                       wheelDiameter,
    units::length::inch_t                                       wheelBase,
    units::length::inch_t                                       track,
    double                                                      odometryComplianceCoefficient,
    units::velocity::meters_per_second_t                        maxSpeed,
    units::radians_per_second_t                                 maxAngularSpeed,
    units::acceleration::meters_per_second_squared_t            maxAcceleration,
    units::angular_acceleration::radians_per_second_squared_t   maxAngularAcceleration,
    string                                                      networkTableName,
    string                                                      controlFileName
) : m_frontLeft(frontLeft), 
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
    m_odometryComplianceCoefficient(odometryComplianceCoefficient),
    m_maxSpeed(maxSpeed),
    m_maxAngularSpeed(maxAngularSpeed), 
    m_maxAcceleration(maxAcceleration), //Not used at the moment
    m_maxAngularAcceleration(maxAngularAcceleration), //Not used at the moment
    m_pigeon(PigeonFactory::GetFactory()->GetPigeon(DragonPigeon::PIGEON_USAGE::CENTER_OF_ROBOT)),
    m_accel(BuiltInAccelerometer()),
    m_runWPI(false),
    m_poseOpt(PoseEstimatorEnum::WPI),
    m_pose(),
    m_offsetPoseAngle(0_deg),  //not used at the moment
    m_drive(units::velocity::meters_per_second_t(0.0)),
    m_steer(units::velocity::meters_per_second_t(0.0)),
    m_rotate(units::angular_velocity::radians_per_second_t(0.0)),
    m_frontLeftLocation(wheelBase/2.0, track/2.0),
    m_frontRightLocation(wheelBase/2.0, -1.0*track/2.0),
    m_backLeftLocation(-1.0*wheelBase/2.0, track/2.0),
    m_backRightLocation(-1.0*wheelBase/2.0, -1.0*track/2.0),
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
    m_limelight(LimelightFactory::GetLimelightFactory()->GetLimelight()),
    m_networkTableName(networkTableName),
    m_controlFileName(controlFileName)
{
    frontLeft.get()->Init( wheelDiameter, maxSpeed, maxAngularSpeed, maxAcceleration, maxAngularAcceleration, m_frontLeftLocation );
    frontRight.get()->Init( wheelDiameter, maxSpeed, maxAngularSpeed, maxAcceleration, maxAngularAcceleration, m_frontRightLocation );
    backLeft.get()->Init( wheelDiameter, maxSpeed, maxAngularSpeed, maxAcceleration, maxAngularAcceleration, m_backLeftLocation );
    backRight.get()->Init( wheelDiameter, maxSpeed, maxAngularSpeed, maxAcceleration, maxAngularAcceleration, m_backRightLocation );
    ZeroAlignSwerveModules();
}

void SwerveChassis::InitStates()
{
    m_robotDrive = new RobotDrive();

    m_driveStateMap[ChassisOptionEnums::DriveStateType::FIELD_DRIVE] =  new FieldDrive(m_robotDrive);
    m_driveStateMap[ChassisOptionEnums::DriveStateType::HOLD_DRIVE]  = new HoldDrive();
    m_driveStateMap[ChassisOptionEnums::DriveStateType::ROBOT_DRIVE] = m_robotDrive;
    m_driveStateMap[ChassisOptionEnums::DriveStateType::STOP_DRIVE] = new StopDrive();
    m_driveStateMap[ChassisOptionEnums::DriveStateType::TRAJECTORY_DRIVE] = new TrajectoryDrive(m_robotDrive);
    //m_driveStateMap[ChassisOptionEnums::DriveStateType::POLAR_DRIVE] = new 

    m_headingStateMap[ChassisOptionEnums::HeadingOption::MAINTAIN] = new MaintainHeading();
    m_headingStateMap[ChassisOptionEnums::HeadingOption::SPECIFIED_ANGLE] = new SpecifiedHeading();
    m_headingStateMap[ChassisOptionEnums::HeadingOption::TOWARD_GOAL] = new FaceGoalHeading();

}
/// @brief Align all of the swerve modules to point forward
void SwerveChassis::ZeroAlignSwerveModules()
{
    m_frontLeft.get()->ZeroAlignModule();
    m_frontRight.get()->ZeroAlignModule();
    m_backLeft.get()->ZeroAlignModule();
    m_backRight.get()->ZeroAlignModule();
}

units::angular_velocity::degrees_per_second_t SwerveChassis::CalcHeadingCorrection
(
    units::angle::degree_t  targetAngle,
    double                  kP
) 
{
    auto currentAngle = GetPose().Rotation().Degrees();
    auto errorAngle = AngleUtils::GetEquivAngle(AngleUtils::GetDeltaAngle(currentAngle, targetAngle));
    auto correction = units::angular_velocity::degrees_per_second_t(errorAngle.to<double>()*kP);
    return correction;
}

/// @brief Drive the chassis
void SwerveChassis::Drive
( 
    ChassisMovement             moveInfo 
)
{
    auto heading = GetHeadingState(moveInfo);
    if (heading != nullptr)
    {
        heading->UpdateChassisSpeeds(moveInfo);
    }

    auto drive = GetDriveState(moveInfo);
    if (drive != nullptr)
    {
        drive->Init(moveInfo);

        auto states = drive->UpdateSwerveModuleStates(moveInfo);       
        m_frontLeft.get()->SetDesiredState(states[LEFT_FRONT]);
        m_frontRight.get()->SetDesiredState(states[RIGHT_FRONT]);
        m_backLeft.get()->SetDesiredState(states[LEFT_BACK]);
        m_backRight.get()->SetDesiredState(states[RIGHT_BACK]); 
    }
}
ISwerveDriveOrientation* SwerveChassis::GetHeadingState
(
    ChassisMovement         moveInfo
)
{
    auto itr = m_headingStateMap.find(moveInfo.headingOption);
    if (itr == m_headingStateMap.end())
    {
        itr = m_headingStateMap.find(ChassisOptionEnums::HeadingOption::MAINTAIN);
    }
    return itr->second;
}
ISwerveDriveState* SwerveChassis::GetDriveState
(
    ChassisMovement         moveInfo
)
{
    auto itr = m_driveStateMap.find(moveInfo.driveOption);
    if (itr == m_driveStateMap.end())
    {
        return m_robotDrive;
    }
    auto state = itr->second;
    state->Init(moveInfo);
    return state;
}

void SwerveChassis::Drive()
{
    // No-op for now
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

/// @brief update the chassis odometry based on current states of the swerve modules and the pigeon
void SwerveChassis::UpdateOdometry() 
{
    units::degree_t yaw{m_pigeon->GetYaw()};
    Rotation2d rot2d {yaw}; 

    m_poseEstimator.Update(rot2d, wpi::array<frc::SwerveModulePosition, 4>{m_frontLeft.get()->GetPosition(),
                                  m_frontRight.get()->GetPosition(), 
                                  m_backLeft.get()->GetPosition(),
                                  m_backRight.get()->GetPosition()});

    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, std::string("SwerveOdometry"), std::string("X Position: "), m_poseEstimator.GetEstimatedPosition().X().to<double>());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, std::string("SwerveOdometry"), std::string("Y Position: "), m_poseEstimator.GetEstimatedPosition().Y().to<double>());
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
    return m_kinematics.ToChassisSpeeds({ m_frontLeft.get()->GetState(), 
                                          m_frontRight.get()->GetState(),
                                          m_backLeft.get()->GetState(),
                                          m_backRight.get()->GetState() });
}

/// @brief Reset the current chassis pose based on the provided pose and rotation
/// @param [in] const Pose2d&       pose        Current XY position
/// @param [in] const Rotation2d&   angle       Current rotation angle
void SwerveChassis::ResetPose
( 
    const Pose2d&       pose,
    const Rotation2d&   angle
)
{
    //m_poseEstimator.ResetPosition(pose, angle);

    SetEncodersToZero();
}


void SwerveChassis::ResetPose
( 
    const Pose2d&       pose
)
{
    Rotation2d angle = pose.Rotation();

    ResetPose(pose, angle);
}

ChassisSpeeds SwerveChassis::GetFieldRelativeSpeeds
(
    units::meters_per_second_t xSpeed,
    units::meters_per_second_t ySpeed,
    units::radians_per_second_t rot        
)
{
    units::angle::radian_t yaw(ConversionUtils::DegreesToRadians(m_pigeon->GetYaw()));
    auto temp = xSpeed*cos(yaw.to<double>()) + ySpeed*sin(yaw.to<double>());
    auto strafe = -1.0*xSpeed*sin(yaw.to<double>()) + ySpeed*cos(yaw.to<double>());
    auto forward = temp;

    ChassisSpeeds output{forward, strafe, rot};

    return output;
}


void SwerveChassis::SetTargetHeading(units::angle::degree_t targetYaw) 
{
    m_targetHeading = targetYaw;
}

void SwerveChassis::ReZero()
{
    m_storedYaw = units::angle::degree_t(0.0);
}