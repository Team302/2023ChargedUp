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
#include <utils/WaypointXmlParser.h>
#include <utils/Logger.h>

TrajectoryGenerator::TrajectoryGenerator(units::meters_per_second_t maxVelocity,
                                        units::meters_per_second_squared_t maxAcceleration) : 
                                        m_config(maxVelocity, maxAcceleration)
{
    //create map with enum and points parsed from xml
    WaypointXmlParser::GetInstance()->ParseWaypoints();
    std::vector<Waypoint2d> parsedWaypoints = WaypointXmlParser::GetInstance()->GetWaypoints();
    
    for(Waypoint2d waypoint: parsedWaypoints)
    {
        m_waypoints.emplace(waypoint.waypointIdentifier, waypoint.coordinates);
    }
}                                        

frc::Trajectory TrajectoryGenerator::GenerateTrajectory(frc::Pose2d currentPose, TARGET_POSITION endPoint)
{
    // check to see if robot is in front of or behind charging pad
    // if so, add intermediate points

    std::vector<frc::Translation2d> intermediatePoints;

    WAYPOINTS endWaypoint;

    //check if we are going to grids
    if(endPoint != TARGET_POSITION::HUMAN_PLAYER_SUBSTATION)
    {
        if(currentPose.Y() > m_waypoints[WAYPOINTS::GRID_ONE_COL_THREE].Y())
        {
            switch (endPoint)
            {
                case TARGET_POSITION::COLUMN_ONE:
                    endWaypoint = WAYPOINTS::GRID_ONE_COL_ONE;
                    break;
                case TARGET_POSITION::COLUMN_TWO:
                    endWaypoint = WAYPOINTS::GRID_ONE_COL_TWO;
                    break;
                case TARGET_POSITION::COLUMN_THREE:
                    endWaypoint = WAYPOINTS::GRID_ONE_COL_THREE;
                    break;
                default:
                    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, std::string("TrajectoryGenerator"), std::string("Grid One"), std::string("Could not find target position"));
                    break;
            }

            //check if we are behind charging pad, if so, add intermediate point
            if(currentPose.X() > m_waypoints[WAYPOINTS::GRID_ONE_INTERMEDIATE].X())
            {
                intermediatePoints.emplace_back(m_waypoints[WAYPOINTS::GRID_ONE_INTERMEDIATE]);
            }
        }
        else if(currentPose.Y() > m_waypoints[WAYPOINTS::GRID_TWO_COL_THREE].Y() && 
                currentPose.Y() < m_waypoints[WAYPOINTS::GRID_ONE_COL_THREE].Y())
        {
            switch (endPoint)
            {
                case TARGET_POSITION::COLUMN_ONE:
                    endWaypoint = WAYPOINTS::GRID_TWO_COL_ONE;
                    break;
                case TARGET_POSITION::COLUMN_TWO:
                    endWaypoint = WAYPOINTS::GRID_TWO_COL_TWO;
                    break;
                case TARGET_POSITION::COLUMN_THREE:
                    endWaypoint = WAYPOINTS::GRID_TWO_COL_THREE;
                    break;
                default:
                    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, std::string("TrajectoryGenerator"), std::string("Grid Two"), std::string("Could not find target position"));
                    break;
            }

            //check if we are behind charging pad, if so, add intermediate point
            if(currentPose.X() > m_waypoints[WAYPOINTS::GRID_TWO_INTERMEDIATE].X())
            {
                intermediatePoints.emplace_back(m_waypoints[WAYPOINTS::GRID_TWO_INTERMEDIATE]);
            }
        }
        else
        {
            switch (endPoint)
            {
                case TARGET_POSITION::COLUMN_ONE:
                    endWaypoint = WAYPOINTS::GRID_THREE_COL_ONE;
                    break;
                case TARGET_POSITION::COLUMN_TWO:
                    endWaypoint = WAYPOINTS::GRID_THREE_COL_TWO;
                    break;
                case TARGET_POSITION::COLUMN_THREE:
                    endWaypoint = WAYPOINTS::GRID_THREE_COL_THREE;
                    break;
                default:
                    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, std::string("TrajectoryGenerator"), std::string("Grid Two"), std::string("Could not find target position"));
                    break;
            }

            //check if we are behind charging pad, if so, add intermediate point
            if(currentPose.X() > m_waypoints[WAYPOINTS::GRID_THREE_INTERMEDIATE].X())
            {
                intermediatePoints.emplace_back(m_waypoints[WAYPOINTS::GRID_THREE_INTERMEDIATE]);
            }
        }
    }
    else //we are going to human player substation
    {

    }

    frc::TrajectoryGenerator::GenerateTrajectory(currentPose, intermediatePoints, frc::Pose2d{m_waypoints[endWaypoint], frc::Rotation2d(0, 0)}, m_config);
}