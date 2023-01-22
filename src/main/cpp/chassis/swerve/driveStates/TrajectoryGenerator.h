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
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>

//FRC Includes
#include <frc/geometry/Pose2d.h>
#include <units/velocity.h>
#include <units/acceleration.h>

//Team 302 Includes

class TrajectoryGenerator
{
    public:
        TrajectoryGenerator(units::meters_per_second_t maxVelocity,
                            units::meters_per_second_squared_t maxAcceleration);
        ~TrajectoryGenerator() = default;

        /// @brief Generate a trajectory to be fed into TrajectoryDrive
        /// @param startPoint starting point, most likely current robot position
        /// @param endPoint ending/goal point
        /// @return frc::Trajectory - the calculated trajectory based on given points
        frc::Trajectory GenerateTrajectory(frc::Pose2d startPoint, frc::Pose2d endPoint);

        /// @brief Get the waypoints to avoid obstacles if path intersects them
        /// @param lower lower bounds of path boundary
        /// @param upper upper bounds of path boundary
        /// @return std::vector<frc::Translation2d> - waypoints to include in path to avoid obstacles
        std::vector<frc::Translation2d> AvoidObstacles(frc::Translation2d lower, frc::Translation2d upper);

    private:
        frc::TrajectoryConfig m_config;

        std::vector<frc::Translation2d> m_obstacles;
};