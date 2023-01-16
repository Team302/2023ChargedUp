
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
    m_storedYaw(m_pigeon->GetYaw()),
    m_yawCorrection(units::angular_velocity::degrees_per_second_t(0.0)),
    m_targetHeading(units::angle::degree_t(0)),
    m_limelight(LimelightFactory::GetLimelightFactory()->GetLimelight()),
    m_networkTableName(networkTableName),
    m_controlFileName(controlFileName)
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

    frontLeft.get()->Init( wheelDiameter, maxSpeed, maxAngularSpeed, maxAcceleration, maxAngularAcceleration, m_frontLeftLocation );
    frontRight.get()->Init( wheelDiameter, maxSpeed, maxAngularSpeed, maxAcceleration, maxAngularAcceleration, m_frontRightLocation );
    backLeft.get()->Init( wheelDiameter, maxSpeed, maxAngularSpeed, maxAcceleration, maxAngularAcceleration, m_backLeftLocation );
    backRight.get()->Init( wheelDiameter, maxSpeed, maxAngularSpeed, maxAcceleration, maxAngularAcceleration, m_backRightLocation );

    ZeroAlignSwerveModules();
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
    /**
    auto speeds = moveInfo.chassisSpeeds;
    auto xSpeed = (abs(speeds.vx.to<double>()) < m_deadband) ? units::meters_per_second_t(0.0) : speeds.vx; 
    auto ySpeed = (abs(speeds.vy.to<double>()) < m_deadband) ? units::meters_per_second_t(0.0) : speeds.vy; 
    auto rot = speeds.omega;
    **/

    auto heading = GetHeadingState(moveInfo);
    if (heading != nullptr)
    {
        heading->UpdateChassisSpeeds(moveInfo);
    }

    auto drive = GetDriveState(moveInfo);
    if (drive != nullptr)
    {
        auto states = drive->UpdateSwerveModuleStates(moveInfo);
//        auto [fl, fr, bl, br] = states;
        
        m_frontLeft.get()->SetDesiredState(states[0]);
        m_frontRight.get()->SetDesiredState(states[1]);
        m_backLeft.get()->SetDesiredState(states[2]);
        m_backRight.get()->SetDesiredState(states[3]); 
    }

    /**
    if ( (abs(xSpeed.to<double>()) < m_deadband) && 
         (abs(ySpeed.to<double>()) < m_deadband) && 
         (abs(rot.to<double>())    < m_angularDeadband.to<double>()))  //our angular deadband, only used once, equates to 10 degrees per second
    {
        m_frontLeft.get()->StopMotors();
        m_frontRight.get()->StopMotors();
        m_backLeft.get()->StopMotors();
        m_backRight.get()->StopMotors();
        m_drive = units::velocity::meters_per_second_t(0.0);
        m_steer = units::velocity::meters_per_second_t(0.0);
        m_rotate = units::angular_velocity::radians_per_second_t(0.0);
    }
    else
    {   
        m_drive = units::velocity::meters_per_second_t(xSpeed);
        m_steer = units::velocity::meters_per_second_t(ySpeed);
        m_rotate = units::angular_velocity::radians_per_second_t(rot);

        if ( m_runWPI )
        {
            units::degree_t yaw{m_pigeon->GetYaw()};
            Rotation2d currentOrientation {yaw};
            ChassisSpeeds chassisSpeeds = moveInfo.driveOption==ChassisOptionEnums::DriveStateType::FIELD_DRIVE ? 
                                            ChassisSpeeds::FromFieldRelativeSpeeds(xSpeed, ySpeed, rot, currentOrientation) : 
                                            ChassisSpeeds{xSpeed, ySpeed, rot};

            auto states = m_kinematics.ToSwerveModuleStates(chassisSpeeds);

            m_kinematics.DesaturateWheelSpeeds(&states, m_maxSpeed);

            auto [fl, fr, bl, br] = states;
       
            m_frontLeft.get()->SetDesiredState(fl);
            m_frontRight.get()->SetDesiredState(fr);
            m_backLeft.get()->SetDesiredState(bl);
            m_backRight.get()->SetDesiredState(br); 
        }
        else
        {
            ChassisSpeeds chassisSpeeds = moveInfo.driveOption==ChassisOptionEnums::DriveStateType::FIELD_DRIVE ?
                                                    GetFieldRelativeSpeeds(xSpeed,ySpeed, rot) : 
                                                    ChassisSpeeds{xSpeed, ySpeed, rot};
            auto states = m_kinematics.ToSwerveModuleStates(chassisSpeeds);
            m_kinematics.DesaturateWheelSpeeds(&states, m_maxSpeed);

            UpdateSwerveModuleStates(chassisSpeeds);

            //Hold position / lock wheels in 'X' configuration
            if(m_hold)
            {
                m_flState.angle = {units::angle::degree_t(45)};
                m_frState.angle = {units::angle::degree_t(-45)};
                m_blState.angle = {units::angle::degree_t(135)};
                m_brState.angle = {units::angle::degree_t(-135)};
            }
            //May need to add m_hold = false here if it gets stuck in hold position
            
            m_frontLeft.get()->SetDesiredState(m_flState);
            m_frontRight.get()->SetDesiredState(m_frState);
            m_backLeft.get()->SetDesiredState(m_blState);
            m_backRight.get()->SetDesiredState(m_brState);
        }
    }    
    **/
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

void SwerveChassis::DriveHoldPosition()
{
    m_hold = true;
}


Pose2d SwerveChassis::GetPose() const
{
    //if (m_poseOpt==PoseEstimatorEnum::WPI)
    //{
    //    return m_poseEstimator.GetEstimatedPosition();
    //}
    return m_pose;
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

    if (m_poseOpt == PoseEstimatorEnum::WPI)
    {
        //auto currentPose = m_poseEstimator.GetEstimatedPosition();

        //m_poseEstimator.Update(rot2d, m_frontLeft.get()->GetState(),
        //                              m_frontRight.get()->GetState(), 
        //                              m_backLeft.get()->GetState(),
        //                              m_backRight.get()->GetState());

        //auto updatedPose = m_poseEstimator.GetEstimatedPosition();
    }
    else if (m_poseOpt==PoseEstimatorEnum::EULER_AT_CHASSIS)
    {
        // get change in time
        units::time::second_t deltaT(0.02);
        //auto deltaT = m_timer.Get();
        //m_timer.Reset();

        // get the information from the last pose 
        auto startX = m_pose.X();
        auto startY = m_pose.Y();

        // xk+1 = xk + vk cos θk T
        // yk+1 = yk + vk sin θk T
        // Thetak+1 = Thetagyro,k+1
        units::angle::radian_t rads = yaw;          // convert angle to radians
        double cosAng = cos(rads.to<double>());
        double sinAng = sin(rads.to<double>());
        auto vx = m_drive * cosAng + m_steer * sinAng;
        auto vy = m_drive * sinAng + m_steer * cosAng;

        units::length::meter_t currentX = startX + m_odometryComplianceCoefficient*(vx * deltaT);
        units::length::meter_t currentY = startY + m_odometryComplianceCoefficient*(vy * deltaT);

        Pose2d currPose{currentX, currentY, rot2d};
        auto trans = currPose - m_pose;
        m_pose = m_pose + trans;
    }
    else if (m_poseOpt==PoseEstimatorEnum::EULER_USING_MODULES ||
             m_poseOpt==PoseEstimatorEnum::POSE_EST_USING_MODULES)
    {
        auto flPose = m_frontLeft.get()->GetCurrentPose(m_poseOpt);
        auto frPose = m_frontRight.get()->GetCurrentPose(m_poseOpt);
        auto blPose = m_backLeft.get()->GetCurrentPose(m_poseOpt);
        auto brPose = m_backRight.get()->GetCurrentPose(m_poseOpt);

        auto chassisX = (flPose.X() + frPose.X() + blPose.X() + brPose.X()) / 4.0;
        auto chassisY = (flPose.Y() + frPose.Y() + blPose.Y() + brPose.Y()) / 4.0;
        Pose2d currPose{chassisX, chassisY, rot2d};
        auto trans = currPose - m_pose;
        m_pose = m_pose + trans;
    }
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
    m_pose = pose;

    auto pigeon = PigeonFactory::GetFactory()->GetPigeon(DragonPigeon::PIGEON_USAGE::CENTER_OF_ROBOT);

    pigeon->ReZeroPigeon(angle.Degrees().to<double>(), 0);

    m_storedYaw = angle.Degrees();

    //m_offsetPoseAngle = units::angle::degree_t(m_pigeon->GetYaw()) - angle.Degrees();

    Transform2d t_fl {m_frontLeftLocation,angle};
    auto flPose = pose + t_fl;
    m_frontLeft.get()->UpdateCurrPose(flPose.X(), flPose.Y());

    Transform2d t_fr {m_frontRightLocation,angle};
    auto frPose = m_pose + t_fr;
    m_frontRight.get()->UpdateCurrPose(frPose.X(), frPose.Y());

    Transform2d t_bl {m_backLeftLocation,angle};
    auto blPose = m_pose + t_bl;
    m_backLeft.get()->UpdateCurrPose(blPose.X(), blPose.Y());

    Transform2d t_br {m_backRightLocation,angle};
    auto brPose = m_pose + t_br;
    m_backRight.get()->UpdateCurrPose(brPose.X(), brPose.Y());
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

void SwerveChassis::UpdateSwerveModuleStates
(
    frc::ChassisSpeeds speeds
)
{
    // These calculations are based on Ether's Chief Delphi derivation
    // The only changes are that that derivation is based on positive angles being clockwise
    // and our codes/sensors are based on positive angles being counter clockwise.

    // A = Vx - omega * L/2
    // B = Vx + omega * L/2
    // C = Vy - omega * W/2
    // D = Vy + omega * W/2
    //
    // Where:
    // Vx is the sideways (strafe) vector
    // Vy is the forward vector
    // omega is the rotation about Z vector
    // L is the wheelbase (front to back)
    // W is the wheeltrack (side to side)
    //
    // Since our Vx is forward and Vy is strafe we need to rotate the vectors
    // We will use these variable names in the code to help tie back to the document.
    // Variable names, though, will follow C++ standards and start with a lower case letter.

    auto l = GetWheelBase();
    auto w = GetTrack();

    auto vy = 1.0 * speeds.vx;
    auto vx = -1.0 * speeds.vy;
    auto omega = speeds.omega;

    units::velocity::meters_per_second_t omegaL = omega.to<double>() * l / 2.0 / 1_s;
    units::velocity::meters_per_second_t omegaW = omega.to<double>() * w / 2.0 / 1_s;
    
    auto a = vx - omegaL;
    auto b = vx + omegaL;
    auto c = vy - omegaW;
    auto d = vy + omegaW;

    // here we'll negate the angle to conform to the positive CCW convention
    m_flState.angle = units::angle::radian_t(atan2(b.to<double>(), d.to<double>()));
    m_flState.angle = -1.0 * m_flState.angle.Degrees();
    m_flState.speed = units::velocity::meters_per_second_t(sqrt( pow(b.to<double>(),2) + pow(d.to<double>(),2) ));
    auto maxCalcSpeed = abs(m_flState.speed.to<double>());

    m_frState.angle = units::angle::radian_t(atan2(b.to<double>(), c.to<double>()));
    m_frState.angle = -1.0 * m_frState.angle.Degrees();
    m_frState.speed = units::velocity::meters_per_second_t(sqrt( pow(b.to<double>(),2) + pow(c.to<double>(),2) ));
    if (abs(m_frState.speed.to<double>())>maxCalcSpeed)
    {
        maxCalcSpeed = abs(m_frState.speed.to<double>());
    }

    m_blState.angle = units::angle::radian_t(atan2(a.to<double>(), d.to<double>()));
    m_blState.angle = -1.0 * m_blState.angle.Degrees();
    m_blState.speed = units::velocity::meters_per_second_t(sqrt( pow(a.to<double>(),2) + pow(d.to<double>(),2) ));
    if (abs(m_blState.speed.to<double>())>maxCalcSpeed)
    {
        maxCalcSpeed = abs(m_blState.speed.to<double>());
    }

    m_brState.angle = units::angle::radian_t(atan2(a.to<double>(), c.to<double>()));
    m_brState.angle = -1.0 * m_brState.angle.Degrees();
    m_brState.speed = units::velocity::meters_per_second_t(sqrt( pow(a.to<double>(),2) + pow(c.to<double>(),2) ));
    if (abs(m_brState.speed.to<double>())>maxCalcSpeed)
    {
        maxCalcSpeed = abs(m_brState.speed.to<double>());
    }

    // normalize speeds if necessary (maxCalcSpeed > max attainable speed)
    if ( maxCalcSpeed > m_maxSpeed.to<double>() )
    {
        auto ratio = m_maxSpeed.to<double>() / maxCalcSpeed;
        m_flState.speed *= ratio;
        m_frState.speed *= ratio;
        m_blState.speed *= ratio;
        m_brState.speed *= ratio;
    }
}

void SwerveChassis::SetTargetHeading(units::angle::degree_t targetYaw) 
{
    m_targetHeading = targetYaw;
}

void SwerveChassis::ReZero()
{
    m_storedYaw = units::angle::degree_t(0.0);
}