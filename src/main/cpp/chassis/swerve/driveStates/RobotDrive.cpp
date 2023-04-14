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

#include <string>

// FRC Includes
#include <frc/filter/SlewRateLimiter.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Twist2d.h>
#include <units/velocity.h>
#include <units/angle.h>

// Team302 Includes

#include <chassis/swerve/driveStates/RobotDrive.h>
#include <chassis/ChassisFactory.h>
#include <chassis/ChassisMovement.h>
#include <utils/logging/Logger.h>
#include <utils/FMSData.h>

using frc::Pose2d;
using frc::Rotation2d;
using std::string;

RobotDrive::RobotDrive() : ISwerveDriveState::ISwerveDriveState(),
                           m_flState(),
                           m_frState(),
                           m_blState(),
                           m_brState(),
                           m_wheelbase(units::length::inch_t(20.0)),
                           m_wheeltrack(units::length::inch_t(20.0)),
                           m_maxspeed(units::velocity::feet_per_second_t(1.0)),
                           m_chassis(ChassisFactory::GetChassisFactory()->GetSwerveChassis()),
                           m_hasTargetPose(false),
                           m_targetPose(),
                           m_timer()

{
    if (m_chassis != nullptr)
    {
        m_wheelbase = m_chassis->GetWheelBase();
        m_wheeltrack = m_chassis->GetTrack();
        m_maxspeed = m_chassis->GetMaxSpeed();
    }
    else
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, string("RobotDrive"), string("Chassis"), string("nullptr"));
    }
}

std::array<frc::SwerveModuleState, 4> RobotDrive::UpdateSwerveModuleStates(ChassisMovement &chassisMovement)
{
    if (chassisMovement.checkTipping)
    {
        DecideTipCorrection(chassisMovement);
    }
    else
    {
    }

    OverSteerCorrection(chassisMovement);

    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "RobotDrive", "Vx", chassisMovement.chassisSpeeds.vx.to<double>());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "RobotDrive", "Vy", chassisMovement.chassisSpeeds.vy.to<double>());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "RobotDrive", "Omega", chassisMovement.chassisSpeeds.omega.to<double>());

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
    auto l = m_wheelbase;
    auto w = m_wheeltrack;

    auto vy = 1.0 * chassisMovement.chassisSpeeds.vx;
    auto vx = -1.0 * chassisMovement.chassisSpeeds.vy;
    auto omega = chassisMovement.chassisSpeeds.omega;

    units::length::meter_t centerOfRotationW = (w / 2.0) - chassisMovement.centerOfRotationOffset.Y();
    units::length::meter_t centerOfRotationL = (l / 2.0) - chassisMovement.centerOfRotationOffset.X();

    auto radius = sqrt(pow(centerOfRotationL.to<double>(), 2) + pow(centerOfRotationW.to<double>(), 2));

    units::velocity::meters_per_second_t omegaW = omega.to<double>() * centerOfRotationW / 1_s;
    units::velocity::meters_per_second_t omegaL = omega.to<double>() * centerOfRotationL / 1_s;

    auto a = vx + omegaL / radius;
    auto b = vx - omegaL / radius;
    auto c = vy + omegaW / radius;
    auto d = vy - omegaW / radius;

    // here we'll negate the angle to conform to the positive CCW convention
    m_flState.angle = units::angle::radian_t(atan2(b.to<double>(), d.to<double>()));
    m_flState.angle = -1.0 * m_flState.angle.Degrees();
    m_flState.speed = units::velocity::meters_per_second_t(sqrt(pow(b.to<double>(), 2) + pow(d.to<double>(), 2)));
    double maxCalcSpeed = abs(m_flState.speed.to<double>());

    m_frState.angle = units::angle::radian_t(atan2(b.to<double>(), c.to<double>()));
    m_frState.angle = -1.0 * m_frState.angle.Degrees();
    m_frState.speed = units::velocity::meters_per_second_t(sqrt(pow(b.to<double>(), 2) + pow(c.to<double>(), 2)));
    if (abs(m_frState.speed.to<double>()) > maxCalcSpeed)
    {
        maxCalcSpeed = abs(m_frState.speed.to<double>());
    }

    m_blState.angle = units::angle::radian_t(atan2(a.to<double>(), d.to<double>()));
    m_blState.angle = -1.0 * m_blState.angle.Degrees();
    m_blState.speed = units::velocity::meters_per_second_t(sqrt(pow(a.to<double>(), 2) + pow(d.to<double>(), 2)));
    if (abs(m_blState.speed.to<double>()) > maxCalcSpeed)
    {
        maxCalcSpeed = abs(m_blState.speed.to<double>());
    }

    m_brState.angle = units::angle::radian_t(atan2(a.to<double>(), c.to<double>()));
    m_brState.angle = -1.0 * m_brState.angle.Degrees();
    m_brState.speed = units::velocity::meters_per_second_t(sqrt(pow(a.to<double>(), 2) + pow(c.to<double>(), 2)));
    if (abs(m_brState.speed.to<double>()) > maxCalcSpeed)
    {
        maxCalcSpeed = abs(m_brState.speed.to<double>());
    }

    // normalize speeds if necessary (maxCalcSpeed > max attainable speed)
    if (maxCalcSpeed > m_maxspeed.to<double>())
    {
        auto ratio = m_maxspeed.to<double>() / maxCalcSpeed;
        m_flState.speed *= ratio;
        m_frState.speed *= ratio;
        m_blState.speed *= ratio;
        m_brState.speed *= ratio;
    }

    /*
    SwerveChassis *chassis = ChassisFactory::GetChassisFactory()->GetSwerveChassis();
    frc::SwerveDriveKinematics<4> kinematics = chassis->GetKinematics();

    wpi::array<frc::SwerveModuleState, 4> states = kinematics.ToSwerveModuleStates(chassisMovement.chassisSpeeds, chassisMovement.centerOfRotationOffset);

    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("Robot Drive"), string("bl_Before"), m_blState.speed.to<double>());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("Robot Drive"), string("br_Before"), m_brState.speed.to<double>());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("Robot Drive"), string("MaxSpeed"), chassis->GetMaxSpeed().to<double>());

    chassis->GetKinematics().DesaturateWheelSpeeds(&states, chassis->GetMaxSpeed());

    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("Robot Drive"), string("bl_After"), m_blState.speed.to<double>());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("Robot Drive"), string("br_After"), m_brState.speed.to<double>());

    auto [fl, fr, bl, br] = states;

    m_flState = fl;
    m_frState = fr;
    m_blState = bl;
    m_brState = br;
*/
    return {m_flState, m_frState, m_blState, m_brState};
}

void RobotDrive::DecideTipCorrection(ChassisMovement &chassisMovement)

{
    if (frc::DriverStation::IsFMSAttached())
    {
        if (frc::DriverStation::GetMatchTime() > 20)
        {
            CorrectForTipping(chassisMovement);
        }
    }
    else
    {
        CorrectForTipping(chassisMovement);
    }
}
void RobotDrive::CorrectForTipping(ChassisMovement &chassisMovement)
{
    // TODO: add checktipping variable to network table
    auto chassis = ChassisFactory::GetChassisFactory()->GetSwerveChassis();
    if (chassis != nullptr)
    {
        // pitch is positive when back of robot is lifted and negative when front of robot is lifted
        // vx is positive driving forward, so if pitch is +, need to slow down
        auto pitch = m_chassis->GetPitch();
        if (std::abs(pitch.to<double>()) > chassisMovement.tippingTolerance.to<double>())
        {
            auto adjust = m_maxspeed * chassisMovement.tippingCorrection * pitch.to<double>();
            chassisMovement.chassisSpeeds.vx -= adjust;
        }

        // roll is positive when the left side of the robot is lifted and negative when the right side of the robot is lifted
        // vy is positive strafing left, so if roll is +, need to strafe slower
        auto roll = m_chassis->GetRoll();
        if (std::abs(roll.to<double>()) > chassisMovement.tippingTolerance.to<double>())
        {
            auto adjust = m_maxspeed * chassisMovement.tippingCorrection * roll.to<double>();
            chassisMovement.chassisSpeeds.vy -= adjust;
        }
    }
}
void RobotDrive::OverSteerCorrection(ChassisMovement &chassisMovement)
{
    /*
    units::time::second_t kLooperDt = units::time::second_t(20.0 / 1000.0);
    frc::Pose2d robot_pose_vel = frc::Pose2d(vx * kLooperDt, vy * kLooperDt, frc::Rotation2d(omega * kLooperDt));
    // frc::Twist2d twist_vel = chassis->GetPose().Log(robot_pose_vel);
    frc::Twist2d twist_vel = frc::Pose2d().Log(robot_pose_vel);
    // chassisMovement.chassisSpeeds = frc::ChassisSpeeds(twist_vel.dx / kLooperDt, twist_vel.dy / kLooperDt, twist_vel.dtheta / kLooperDt);

    // vx = twist_vel.dx / kLooperDt;
    // vy = twist_vel.dx / kLooperDt;
    // omega = twist_vel.dtheta / kLooperDt;
    */

    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("RobotDrive debugging"), string("input chassisMovement vx"), chassisMovement.chassisSpeeds.vx.to<double>());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("RobotDrive debugging"), string("input chassisMovement vy"), chassisMovement.chassisSpeeds.vy.to<double>());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("RobotDrive debugging"), string("input chassisMovement omega"), chassisMovement.chassisSpeeds.omega.to<double>());

    if (m_chassis != nullptr)
    {
        auto currentPose = m_chassis->GetPose();

        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("RobotDrive debugging"), string("input pose X"), currentPose.X().to<double>());
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("RobotDrive debugging"), string("input pose Y"), currentPose.Y().to<double>());
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("RobotDrive debugging"), string("input pose angle"), currentPose.Rotation().Degrees().to<double>());

        if (m_hasTargetPose)
        {
            auto time = m_timer.Get();
            auto errorX = (currentPose.X() - m_targetPose.X());
            chassisMovement.chassisSpeeds.vx += (errorX * PCONST_X) / time;
            auto errorY = (currentPose.Y() - m_targetPose.Y());
            chassisMovement.chassisSpeeds.vy += (errorY * PCONST_Y) / time;

            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("RobotDrive debugging"), string("x error"), errorX.to<double>());
            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("RobotDrive debugging"), string("y error"), errorY.to<double>());
        }

        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("RobotDrive debugging"), string("positive input chassisMovement vx"), chassisMovement.chassisSpeeds.vx.to<double>() > 0.01);
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("RobotDrive debugging"), string("positive input chassisMovement vy"), chassisMovement.chassisSpeeds.vy.to<double>() > 0.01);
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("RobotDrive debugging"), string("positive input chassisMovement omega"), chassisMovement.chassisSpeeds.omega.to<double>() > 0.01);

        auto targetX = currentPose.X() + chassisMovement.chassisSpeeds.vx * LOOP_TIME;
        auto targetY = currentPose.Y() + chassisMovement.chassisSpeeds.vy * LOOP_TIME;
        auto targetRot = currentPose.Rotation() + Rotation2d(units::angle::radian_t(chassisMovement.chassisSpeeds.omega.to<double>() * LOOP_TIME.to<double>()));
        // auto targetPose = Pose2d(targetX, targetY, targetRot);
        m_targetPose = Pose2d(targetX, targetY, targetRot);
        m_hasTargetPose = true;
        m_timer.Reset();
        m_timer.Start();

        // auto twist = currentPose.Log(targetPose);

        // Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("RobotDrive debugging"), string("twist dx"), twist.dx.to<double>());
        // Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("RobotDrive debugging"), string("twist dy"), twist.dy.to<double>());
        // Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("RobotDrive debugging"), string("twist dtheta"), twist.dtheta.to<double>());
        // Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("RobotDrive debugging"), string("positive twist dx"), twist.dx.to<double>() > 0.0);
        // Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("RobotDrive debugging"), string("positive twist dy"), twist.dy.to<double>() > 0.0);
        // Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("RobotDrive debugging"), string("positive twist dtheta"), twist.dtheta.to<double>() > 0.0);

        // chassisMovement.chassisSpeeds.vx = twist.dx / LOOP_TIME;
        // chassisMovement.chassisSpeeds.vy = twist.dy / LOOP_TIME;
        // chassisMovement.chassisSpeeds.omega = twist.dtheta / LOOP_TIME;
        // frc::ChassisSpeeds dummy;
        // dummy.vx = twist.dx / LOOP_TIME;
        // dummy.vy = twist.dy / LOOP_TIME;
        // dummy.omega = twist.dtheta / LOOP_TIME;
        /*
        auto ccwRot = chassisMovement.chassisSpeeds.omega > ROTATE_TOLERANCE;
        auto cwRot = chassisMovement.chassisSpeeds.omega < -1.0 * ROTATE_TOLERANCE;
        auto forwardDrive = chassisMovement.chassisSpeeds.vx > MOVE_TOLERANCE;
        auto backwardDrive = chassisMovement.chassisSpeeds.vx < -1.0 * MOVE_TOLERANCE;
        auto strafeLeft = chassisMovement.chassisSpeeds.vy > MOVE_TOLERANCE;
        auto strafeRight = chassisMovement.chassisSpeeds.vy < -1.0 * MOVE_TOLERANCE;
        auto twistVxAdjust = twist.dx.to<double>();
        auto twistVyAdjust = twist.dy.to<double>();

        auto vxFactor = 1.0;
        auto vyFactor = -1.0;
        if (ccwRot)
        {
            if (forwardDrive && !strafeLeft)
            {
                vxFactor = 1.0;
                vyFactor = -1.0;
            }
            else if (backwardDrive)
            {
                vxFactor = -1.0;
                vyFactor = 1.0;
            }
        }
        else if (cwRot)
        {
            if (forwardDrive && !strafeLeft)
            {
                vxFactor = 1.0;
                vyFactor = -1.0;
            }
            else if (backwardDrive)
            {
                vxFactor = 1.0;
                vyFactor = -1.0;
            }
        }
        chassisMovement.chassisSpeeds.vx += units::velocity::meters_per_second_t(vxFactor * twistVxAdjust);
        chassisMovement.chassisSpeeds.vy += units::velocity::meters_per_second_t(vyFactor * twistVyAdjust);
        // chassisMovement.chassisSpeeds.omega += units::angular_velocity::radians_per_second_t(twist.dtheta.to<double>());
        */
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("RobotDrive debugging"), string("corrected chassisMovement vx"), chassisMovement.chassisSpeeds.vx.to<double>());
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("RobotDrive debugging"), string("corrected chassisMovement vy"), chassisMovement.chassisSpeeds.vy.to<double>());
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("RobotDrive debugging"), string("corrected chassisMovement omega"), chassisMovement.chassisSpeeds.omega.to<double>());
    }
}
void RobotDrive::Init(ChassisMovement &chassisMovement)
{
}