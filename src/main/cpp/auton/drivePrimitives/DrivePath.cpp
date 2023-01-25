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

//C++
#include <string>

//FRC Includes
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/Filesystem.h>
#include <frc/trajectory/TrajectoryUtil.h>
#include <units/angular_velocity.h>
#include <wpi/fs.h>

// 302 Includes
#include <auton/drivePrimitives/DrivePath.h>
#include <chassis/ChassisMovement.h>
#include <chassis/ChassisOptionEnums.h>
#include <chassis/ChassisFactory.h>
#include <chassis/IChassis.h>
#include <utils/Logger.h>


using namespace std;
using namespace frc;

using namespace wpi::math;

DrivePath::DrivePath() : m_chassis(ChassisFactory::GetChassisFactory()->GetIChassis()),
                         m_timer(make_unique<Timer>()),
                         m_trajectory(),
                         m_runHoloController(true),
                         m_ramseteController(),
                         m_holoController(frc2::PIDController{1.65, 0, 0},
                                          frc2::PIDController{1.65, 0, 0},
                                          frc::ProfiledPIDController<units::radian>{0.25, 0, 0,
                                                                                    frc::TrapezoidProfile<units::radian>::Constraints{0_rad_per_s, 0_rad_per_s / 1_s}}),
                         //max velocity of 1 rotation per second and a max acceleration of 180 degrees per second squared.
                         m_headingOption(ChassisOptionEnums::HeadingOption::MAINTAIN),
                         m_heading(0.0),
                         m_maxTime(-1.0), 
                         m_ntName("DrivePath")

{
}
void DrivePath::Init(PrimitiveParams *params)
{
    m_pathname = params->GetPathName(); //Grabs path name from auton xml
    m_ntName = string("DrivePath: ") + m_pathname;
    m_headingOption = params->GetHeadingOption();
    m_heading = params->GetHeading();
    m_maxTime = params->GetTime();

    //Start timeout timer for path
    m_timer.get()->Reset();
    m_timer.get()->Start();

    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, m_ntName, "WhyDone", "Not done");

    GetTrajectory(params->GetPathName());  //Parses path from json file based on path name given in xml
}
void DrivePath::Run()
{
    ChassisMovement moveInfo;
    moveInfo.driveOption = ChassisOptionEnums::DriveStateType::TRAJECTORY_DRIVE;
    moveInfo.controllerType = ChassisOptionEnums::AutonControllerType::HOLONOMIC;
    moveInfo.headingOption = ChassisOptionEnums::HeadingOption::MAINTAIN;

    // Use the controller to calculate the chassis speeds for getting there
    if (m_runHoloController)
    {
        switch (m_headingOption)
        {
            case ChassisOptionEnums::HeadingOption::MAINTAIN:
               break;

            case ChassisOptionEnums::HeadingOption::TOWARD_GOAL:
                moveInfo.headingOption = ChassisOptionEnums::HeadingOption::TOWARD_GOAL;
                break;

            case ChassisOptionEnums::HeadingOption::SPECIFIED_ANGLE:
                moveInfo.headingOption = ChassisOptionEnums::HeadingOption::SPECIFIED_ANGLE;
                moveInfo.yawAngle = units::angle::degree_t(m_heading);
                m_chassis.get()->SetTargetHeading(units::angle::degree_t(m_heading));
                break;
            
            default:
                break;
        }
    }
    else
    {
        moveInfo.controllerType = ChassisOptionEnums::AutonControllerType::RAMSETE;
    }
    m_chassis.get()->Drive(moveInfo);
}

bool DrivePath::IsDone()
{
    
    if(m_timer.get()->Get() > m_trajectory.TotalTime())
    {
        return true;
    }
    else
    {
        /// @TODO: Add accessor for current drive state to return why/isDone from TrajectoryDrive
        /*if(TrajectoryDrive->IsDone()) //TrajectoryDrive is done -> log the reason why and end drive path primitive
        {
            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, m_ntName, "WhyDone", TrajectoryDrive->WhyDone());
            return true;
        }
        else //TrajectoryDrive isn't done
        {
            return false;
        }
        */


       /// PLACEHOLDER UNTIL ACCESSOR CREATED
       return false;
    }
}

void DrivePath::GetTrajectory //Parses pathweaver json to create a series of points that we can drive the robot to
(
    string  path
)
{
    if (!path.empty()) // only go if path name found
    {
        // Read path into trajectory for deploy directory.  JSON File ex. Bounce1.wpilib.json
    	auto deployDir = frc::filesystem::GetDeployDirectory();
        deployDir += "/paths/output/" + path;

        m_trajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDir);

        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, m_ntName, string("Deploy path is "), deployDir.c_str()); //Debugging
        
        m_trajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDir);  //Creates a trajectory or path that can be used in the code, parsed from pathweaver json
    }

}