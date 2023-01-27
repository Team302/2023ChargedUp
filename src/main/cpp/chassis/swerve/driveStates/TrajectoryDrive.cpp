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

#include <frc/geometry/Pose2d.h>

//Team302 Includes
#include <chassis/swerve/driveStates/TrajectoryDrive.h>
#include <chassis/ChassisMovement.h>
#include <chassis/ChassisFactory.h>
#include <utils/Logger.h>

using frc::Pose2d;

TrajectoryDrive::TrajectoryDrive(RobotDrive* robotDrive) : RobotDrive(),
    m_trajectory(),
    m_robotDrive(robotDrive),
    m_holonomicController(frc2::PIDController{1.5, 0, 0},
                          frc2::PIDController{1.5, 0, 0},
                          frc::ProfiledPIDController<units::radian>{0.1, 0, 0,
                          frc::TrapezoidProfile<units::radian>::Constraints{0_rad_per_s, 0_rad_per_s / 1_s}}),
    m_desiredState(),
    m_trajectoryStates(),
    m_timer(std::make_unique<frc::Timer>()),
    m_chassis(ChassisFactory::GetChassisFactory()->GetSwerveChassis())
{

}

void TrajectoryDrive::Init
(
    ChassisMovement& chassisMovement
)
{
    //Clear m_trajectoryStates in case it holds onto a previous trajectory
    m_trajectoryStates.clear();

    m_trajectory = chassisMovement.trajectory;
    m_trajectoryStates = m_trajectory.States();

    /// DEBUGGING
    m_initTimesRan++;
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "Trajectory Drive", "Init", m_initTimesRan);

    if (!m_trajectoryStates.empty()) // only go if path name found
    {
        //Desired state is first state in trajectory
        m_desiredState = m_trajectoryStates.front(); //m_desiredState is the first state, or starting position

        /// DEBUGGING
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "Trajectory Drive", "Init", "Not empty");

        m_timer.get()->Reset(); //Restarts and starts timer
        m_timer.get()->Start();
    }
}

std::array<frc::SwerveModuleState, 4> TrajectoryDrive::UpdateSwerveModuleStates
(
    ChassisMovement& chassisMovement
)
{
    if (!m_trajectoryStates.empty()) //If we have a path parsed / have states to run
    {
        // calculate where we are and where we want to be
        CalcCurrentAndDesiredStates();

        // Use the controller to calculate the chassis speeds for getting there
        frc::ChassisSpeeds refChassisSpeeds;
        refChassisSpeeds = m_holonomicController.Calculate( m_chassis->GetPose(),
                                                          m_desiredState, 
                                                          m_desiredState.pose.Rotation());
        //Set chassisMovement speeds that will be used by RobotDrive
        chassisMovement.chassisSpeeds = refChassisSpeeds;

        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "Trajectory Drive", "Desired X", m_desiredState.pose.X().to<double>());
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "Trajectory Drive", "Desired Y", m_desiredState.pose.Y().to<double>());
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "Trajectory Drive", "Current Time", m_timer.get()->Get().to<double>());

        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "Trajectory Drive", "ChassisMovement VX", chassisMovement.chassisSpeeds.vx.to<double>());
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "Trajectory Drive", "ChassisMovement VY", chassisMovement.chassisSpeeds.vy.to<double>());
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "Trajectory Drive", "ChassisMovement Omega", chassisMovement.chassisSpeeds.omega.to<double>());

        return m_robotDrive->UpdateSwerveModuleStates(chassisMovement);

    }
    else //If we don't have states to run, don't move the robot
    {
        //Create 0 speed frc::ChassisSpeeds
        frc::ChassisSpeeds speeds;
        speeds.vx = 0_mps;
        speeds.vy = 0_mps;
        speeds.omega = units::angular_velocity::radians_per_second_t(0);
        
        //Set chassisMovement speeds that will be used by RobotDrive
        chassisMovement.chassisSpeeds = speeds;
        return m_robotDrive->UpdateSwerveModuleStates(chassisMovement);
    }
}

void TrajectoryDrive::CalcCurrentAndDesiredStates()
{
    //Get current time
    auto sampleTime = units::time::second_t(m_timer.get()->Get());
    //Set desired state to the state at current time
    m_desiredState = m_trajectory.Sample(sampleTime);
}