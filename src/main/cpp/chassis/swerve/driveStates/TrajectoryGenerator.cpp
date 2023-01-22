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

//C++ Includes

//FRC Includes

//Team 302 Includes
#include <chassis/swerve/driveStates/TrajectoryGenerator.h>
#include <chassis/ChassisFactory.h>
#include <utils/ObstacleXmlParser.h>

TrajectoryGenerator::TrajectoryGenerator(units::meters_per_second_t maxVelocity,
                                        units::meters_per_second_squared_t maxAcceleration) : 
                                        m_config(maxVelocity, maxAcceleration),
                                        m_obstacles(ObstacleXmlParser::GetInstance()->GetObstacles())
{

}                                        

frc::Trajectory TrajectoryGenerator::GenerateTrajectory(frc::Pose2d startPoint, frc::Pose2d endPoint)
{
    //do math to determine if field obstacles are in between start point and end point
    //"bounding box" would be area of retangle created by start and end point with half or all of the robot width added
    
    //get area of startPoint and endPoint
    units::length::meter_t lowerX, lowerY, upperX, upperY;

    auto robotWidth = ChassisFactory::GetChassisFactory()->GetSwerveChassis()->GetWheelBase();
    auto robotLength = ChassisFactory::GetChassisFactory()->GetSwerveChassis()->GetTrack();

    if(startPoint.X() < endPoint.X())
    {
        lowerX = startPoint.X() - robotWidth;
        upperX = endPoint.X() + robotWidth;
    }
    else
    {
        lowerX = endPoint.X() - robotWidth;
        upperX = startPoint.X() + robotWidth;
    }

    if(startPoint.Y() < endPoint.Y())
    {
        lowerY = startPoint.Y() - robotLength;
        upperY = endPoint.Y() + robotLength;
    }
    else
    {
        lowerY = endPoint.Y() - robotLength;
        upperY = startPoint.Y() + robotLength;
    }

    frc::Translation2d lowerBounds = {lowerX, lowerY};
    frc::Translation2d upperBounds = {upperX, upperY};

    std::vector<frc::Translation2d> avoidancePoints = AvoidObstacles(lowerBounds, upperBounds);
}

std::vector<frc::Translation2d> TrajectoryGenerator::AvoidObstacles(frc::Translation2d lower, frc::Translation2d upper)
{
    auto robotWidth = ChassisFactory::GetChassisFactory()->GetSwerveChassis()->GetWheelBase();
    auto robotLength = ChassisFactory::GetChassisFactory()->GetSwerveChassis()->GetTrack();
    
    for(frc::Translation2d obstacle : m_obstacles)
    {
        if(obstacle.X() > lower.X() && obstacle.X() < upper.X())
        {
            if(obstacle.X() + robotWidth < lower.X() && obstacle.X() < upper.X())
            {}
        }
    }
}