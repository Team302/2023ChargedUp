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
#include <frc/trajectory/TrajectoryGenerator.h>

//Team 302 Includes
#include <chassis/swerve/driveStates/TrajectoryGenerator.h>
#include <chassis/ChassisFactory.h>
#include <utils/WaypointXmlParser.h>
#include <utils/Logger.h>
#include <utils/Waypoint2d.h>

using namespace Dragons;

Dragons::TrajectoryGenerator::TrajectoryGenerator(units::meters_per_second_t maxVelocity,
                                        units::meters_per_second_squared_t maxAcceleration) : 
                                        m_config(maxVelocity, maxAcceleration)
{
    //create map with enum and points parsed from xml
    WaypointXmlParser::GetInstance()->ParseWaypoints();
    std::vector<Waypoint2d> parsedWaypoints = WaypointXmlParser::GetInstance()->GetWaypoints();
    
    for(Waypoint2d waypoint: parsedWaypoints)
    {
        m_blueWaypoints.emplace(waypoint.waypointIdentifier, waypoint.blueCoordinates);
        m_redWaypoints.emplace(waypoint.waypointIdentifier, waypoint.redCoordinates);
    }
}                                        

frc::Trajectory Dragons::TrajectoryGenerator::GenerateTrajectory(frc::Pose2d currentPose, TARGET_POSITION endPoint)
{
    std::vector<frc::Translation2d> intermediatePoints;

    WAYPOINTS endWaypoint;

    //check if we are going to grids
    if(endPoint != TARGET_POSITION::HUMAN_PLAYER_SUBSTATION)
    {
        if(currentPose.Y() > m_blueWaypoints[WAYPOINTS::GRID_WALL_COL_THREE].Y()) //Are we below or in line with the wall grid?  the waypoint color doesnt matter since they are same y value
        {
            switch (endPoint)
            {
                case TARGET_POSITION::COLUMN_ONE:
                    endWaypoint = WAYPOINTS::GRID_WALL_COL_ONE;
                    break;
                case TARGET_POSITION::COLUMN_TWO:
                    endWaypoint = WAYPOINTS::GRID_WALL_COL_TWO;
                    break;
                case TARGET_POSITION::COLUMN_THREE:
                    endWaypoint = WAYPOINTS::GRID_WALL_COL_THREE;
                    break;
                default:
                    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, std::string("TrajectoryGenerator"), std::string("Grid Wall"), std::string("Could not find target position"));
                    break;
            }

            //check if we are behind charging pad, if so, add intermediate point

            //FIRST DIFFERENTIATE BETWEEN RED AND BLUE ALLIANCE
            if(currentPose.X() > m_blueWaypoints[WAYPOINTS::GRID_WALL_INTERMEDIATE].X())
            {
                intermediatePoints.emplace_back(m_blueWaypoints[WAYPOINTS::GRID_WALL_INTERMEDIATE]);
            }
        }
        else if(currentPose.Y() > m_blueWaypoints[WAYPOINTS::GRID_COOP_COL_THREE].Y() && 
                currentPose.Y() < m_blueWaypoints[WAYPOINTS::GRID_WALL_COL_THREE].Y()) //are we in between the HP grid and the wall grid? the waypoint color doesnt matter since they are same y value
        {
            switch (endPoint)
            {
                case TARGET_POSITION::COLUMN_ONE:
                    endWaypoint = WAYPOINTS::GRID_COOP_COL_ONE;
                    break;
                case TARGET_POSITION::COLUMN_TWO:
                    endWaypoint = WAYPOINTS::GRID_COOP_COL_TWO;
                    break;
                case TARGET_POSITION::COLUMN_THREE:
                    endWaypoint = WAYPOINTS::GRID_COOP_COL_THREE;
                    break;
                default:
                    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, std::string("TrajectoryGenerator"), std::string("Grid Coop"), std::string("Could not find target position"));
                    break;
            }

            //check if we are behind charging pad, if so, add intermediate point

            //FIRST DIFFERENTIATE BETWEEN RED AND BLUE ALLIANCE
            if(currentPose.X() > m_blueWaypoints[WAYPOINTS::GRID_COOP_INTERMEDIATE].X())
            {
                intermediatePoints.emplace_back(m_blueWaypoints[WAYPOINTS::GRID_COOP_INTERMEDIATE]);
            }
        }
        else //the only place we can be is the HP grid or above
        {
            switch (endPoint)
            {
                case TARGET_POSITION::COLUMN_ONE:
                    endWaypoint = WAYPOINTS::GRID_HP_COL_ONE;
                    break;
                case TARGET_POSITION::COLUMN_TWO:
                    endWaypoint = WAYPOINTS::GRID_HP_COL_TWO;
                    break;
                case TARGET_POSITION::COLUMN_THREE:
                    endWaypoint = WAYPOINTS::GRID_HP_COL_THREE;
                    break;
                default:
                    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, std::string("TrajectoryGenerator"), std::string("Grid HP"), std::string("Could not find target position"));
                    break;
            }

            //check if we are behind charging pad, if so, add intermediate point

            //FIRST DIFFERENTIATE BETWEEN RED AND BLUE ALLIANCE
            if(currentPose.X() > m_blueWaypoints[WAYPOINTS::GRID_HP_INTERMEDIATE].X())
            {
                intermediatePoints.emplace_back(m_blueWaypoints[WAYPOINTS::GRID_HP_INTERMEDIATE]);
            }
        }
    }
    else //we are going to human player substation
    {

    }

    /// @TODO: Differentiate between red and blue alliance
    /*if(allianceColor == blue)
    {
        frc::TrajectoryGenerator::GenerateTrajectory(currentPose, intermediatePoints, frc::Pose2d{m_blueWaypoints[endWaypoint], frc::Rotation2d(0, 0)}, m_config);
        set correct heading for blue side
    }
    else if(allianceColor == red)
    {
        frc::TrajectoryGenerator::GenerateTrajectory(currentPose, intermediatePoints, frc::Pose2d{m_redWaypoints[endWaypoint], frc::Rotation2d(0, 0)}, m_config);
        set correct heading for red side
    }
    */
    return frc::TrajectoryGenerator::GenerateTrajectory(currentPose, intermediatePoints, frc::Pose2d{m_blueWaypoints[endWaypoint], frc::Rotation2d(0, 0)}, m_config);
}