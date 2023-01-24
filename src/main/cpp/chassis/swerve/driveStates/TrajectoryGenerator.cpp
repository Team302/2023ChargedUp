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
                                        m_config(maxVelocity, maxAcceleration)
{
    //create map with enum and points parsed from xml
    ObstacleXmlParser::GetInstance()->ParseObstacles();
    m_waypoints = ObstacleXmlParser::GetInstance()->GetWaypoints();

    //for(frc::Translation2d waypoint )
}                                        

frc::Trajectory TrajectoryGenerator::GenerateTrajectory(frc::Pose2d currentPose, WAYPOINTS endPoint)
{
    // check to see if robot is in front of or behind charging pad
    // if so, add intermediate points


    if(currentPose.X() > )

    // get waypoint based on endPoint enum, then get waypoint x and y values from xml


}