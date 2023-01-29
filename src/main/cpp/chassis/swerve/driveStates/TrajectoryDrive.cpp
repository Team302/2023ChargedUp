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
    m_holonomicController(frc2::PIDController{1.0, 0, 0},
                          frc2::PIDController{1.0, 0, 0},
                          frc::ProfiledPIDController<units::radian>{0.1, 0, 0,
                          frc::TrapezoidProfile<units::radian>::Constraints{0_rad_per_s, 0_rad_per_s / 1_s}}),
    m_desiredState(),
    m_trajectoryStates(),
    m_prevPose(ChassisFactory::GetChassisFactory()->GetSwerveChassis()->GetPose()),
    m_wasMoving(false),
    m_timer(std::make_unique<frc::Timer>()),
    m_chassis(ChassisFactory::GetChassisFactory()->GetSwerveChassis()),
    m_whyDone("Trajectory isn't finished/Error")

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


    if (!m_trajectoryStates.empty()) // only go if path name found
    {
        //Desired state is first state in trajectory
        m_desiredState = m_trajectoryStates.front(); //m_desiredState is the first state, or starting position

        m_finalState = m_trajectoryStates.back();

        m_timer.get()->Reset(); //Restarts and starts timer
        m_timer.get()->Start();
    }

    m_delta = m_finalState.pose - ChassisFactory::GetChassisFactory()->GetSwerveChassis()->GetPose();
}

std::array<frc::SwerveModuleState, 4> TrajectoryDrive::UpdateSwerveModuleStates
(
    ChassisMovement& chassisMovement
)
{
    if (!m_trajectoryStates.empty()) //If we have a path parsed / have states to run
    {
        if(m_trajectory.InitialPose() != chassisMovement.trajectory.InitialPose())  
        {
            Init(chassisMovement);
        }

        // calculate where we are and where we want to be
        CalcCurrentAndDesiredStates();

        // Use the controller to calculate the chassis speeds for getting there
        frc::ChassisSpeeds refChassisSpeeds;
        refChassisSpeeds = m_holonomicController.Calculate( m_chassis->GetPose(),
                                                          m_desiredState, 
                                                          m_desiredState.pose.Rotation());
        //Set chassisMovement speeds that will be used by RobotDrive
        chassisMovement.chassisSpeeds = refChassisSpeeds;
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

bool TrajectoryDrive::IsDone()
{
    
    bool isDone = false;
    
    if (!m_trajectoryStates.empty()) //If we have states... 
    {
        auto curPos = ChassisFactory::GetChassisFactory()->GetSwerveChassis()->GetPose();
        
        // Check if the current pose and the trajectory's final pose are the same
        if (IsSamePose(curPos, m_desiredState.pose, 100.0))
        {
            isDone = true;
            m_whyDone = "Current Pose = Trajectory final pose";
        }
        
        if ( !isDone )
        {
            // Now check if the current pose is getting closer or farther from the target pose 
            auto trans =  m_finalState.pose - curPos;
            auto thisDeltaX = trans.X().to<double>();
            auto thisDeltaY = trans.Y().to<double>();
            if (abs(trans.X().to<double>()) < m_delta.X().to<double>() && abs(trans.Y().to<double>()) < m_delta.Y().to<double>())
            {   // Getting closer so just update the deltas
                m_delta.X() = units::length::meter_t(thisDeltaX);
                m_delta.Y() = units::length::meter_t(thisDeltaY);
            }
            else
            {   // Getting farther away:  determine if it is because of the path curvature (not straight line between start and the end)
                // or because we went past the target (in this case, we are done)
                // Assume that once we get within a third of a meter (just under 12 inches), if we get
                // farther away we are passing the target, so we should stop.  Otherwise, keep trying.
                if ((abs(m_delta.X().to<double>()) < 0.3 && abs(m_delta.Y().to<double>()) < 0.3))
                {
                    isDone = true;
                    m_whyDone = "Within 12 inches of target or getting farther away from target";
                }
            }
        }       
 
        if (m_timer.get()->Get() > 1_s)//This if statement makes sure that we aren't checking for position change right at the start
        {
            auto moving = !IsSamePose(curPos, m_prevPose, 7.5);
            if (!moving && m_wasMoving)  //If we aren't moving and last state we were moving, then...
            {
                    isDone = true;
                    m_whyDone = "Stopped moving";                    
            }
            m_prevPose = curPos;
            m_wasMoving = moving;
        }
    }
    else
    {
        m_whyDone = "No states in trajectory";
        isDone = true;
    }
    return isDone;
}

bool TrajectoryDrive::IsSamePose(frc::Pose2d currentPose, frc::Pose2d previousPose, double tolerance)
{
    // Detect if the two poses are the same within a tolerance
    double dCurPosX = currentPose.X().to<double>() * 1000; //cm
    double dCurPosY = currentPose.Y().to<double>() * 1000;
    double dPrevPosX = previousPose.X().to<double>() * 1000;
    double dPrevPosY = previousPose.Y().to<double>() * 1000;

    double dDeltaX = abs(dPrevPosX - dCurPosX);
    double dDeltaY = abs(dPrevPosY - dCurPosY);

    //  If Position of X or Y has moved since last scan..  Using Delta X/Y
    return (dDeltaX <= tolerance && dDeltaY <= tolerance);
}