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
#include <vector>

//FRC Includes
#include <frc/geometry/Translation2d.h>

//Team 302 Includes
#include <chassis/swerve/driveStates/TrajectoryGenerator.h>
#include <utils/Waypoint2d.h>

class WaypointXmlParser
{
    public:
        WaypointXmlParser() = default;
        ~WaypointXmlParser() = default;

        /// @brief  Find or create the Waypoint parser
		static WaypointXmlParser* GetInstance();

        std::vector<Waypoint2d> ParseWaypoints();

        std::unordered_map<TrajectoryGenerator::WAYPOINTS, frc::Translation2d> GetWaypoints() const {return m_waypoints;};
    private:
        static WaypointXmlParser*	m_instance;

        std::unordered_map<TrajectoryGenerator::WAYPOINTS, frc::Translation2d> m_waypoints;
};