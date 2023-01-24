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

#pragma once

//C++ Includes
#include <map>

//FRC Includes
#include <frc/geometry/Pose2d.h>
#include <units/velocity.h>
#include <units/acceleration.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>

//Team 302 Includes
#include <utils/Waypoint2d.h>

class TrajectoryGenerator
{
    public:
        enum WAYPOINTS
        {
            GRID_ONE_COL_ONE,
            GRID_ONE_COL_TWO,
            GRID_ONE_COL_THREE,
            GRID_TWO_COL_ONE,
            GRID_TWO_COL_TWO,
            GRID_TWO_COL_THREE,
            GRID_THREE_COL_ONE,
            GRID_THREE_COL_TWO,
            GRID_THREE_COL_THREE,
            GRID_ONE_INTERMEDIATE,
            GRID_TWO_INTERMEDIATE,
            GRID_THREE_INTERMEDIATE
        };

        enum TARGET_POSITION
        {
            COLUMN_ONE,
            COLUMN_TWO,
            COLUMN_THREE,
            HUMAN_PLAYER_SUBSTATION
        };

        TrajectoryGenerator(units::meters_per_second_t maxVelocity,
                            units::meters_per_second_squared_t maxAcceleration);
        ~TrajectoryGenerator() = default;

        /// @brief Generate a trajectory to be fed into TrajectoryDrive
        /// @param currentPose current robot position
        /// @param endPoint ending/goal point
        /// @return frc::Trajectory - the calculated trajectory based on given points
        frc::Trajectory GenerateTrajectory(frc::Pose2d currentPose, TARGET_POSITION endPoint);

    private:
        frc::TrajectoryConfig m_config;

        std::unordered_map<WAYPOINTS, frc::Translation2d>   m_waypoints;
};