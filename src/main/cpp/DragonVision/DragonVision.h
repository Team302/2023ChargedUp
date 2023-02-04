
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

#pragma once

//C++
#include <string>
#include <map>

//FRC Includes
#include <frc/geometry/Pose2d.h>

//Team302 Includes
#include <hw/DragonLimelight.h>
#include <DragonVision/LimelightState.h>


class DragonLimelight;
class DragonVision
{
    public:
        static DragonVision* GetDragonVision();

    enum LIMELIGHT_STATES
        {
            RETROREFLECTIVE,
            APRILTAG,
            CUBE,
            CONE
        };

    bool AlignedWithCubeNode();
    bool AlignedWithConeNode();

    bool AlignedWithSubstation();
    bool AlignedWithCubeGamePiece();
    bool AlignedWithConeGamePiece();

    units::length::inch_t DistanceFromCubeNode();
    units::length::inch_t DistanceFromConeNode();
    units::length::inch_t DistanceFromSubstation();
    units::length::inch_t DistanceFromCubeGamePiece();
    units::length::inch_t DistanceFromConeGamePiece();

    units::angle::degree_t AngleFromCubeNode();
    units::angle::degree_t AngleFromConeNode();
    units::angle::degree_t AngleFromSubstation();
    units::angle::degree_t AngleFromCubeGamePiece();
    units::angle::degree_t AngleFromConeGamePiece();

    frc::Pose2d GetRobotPosition();

~DragonVision() = default;
private:
    void SetCurrentState
    (
        DragonVision::LIMELIGHT_STATES limelightstate
    );
    
    DragonVision(std::string     stateName,
                int              stateId
                );
    

    static DragonVision*	m_dragonVision;
    DragonLimelight*        m_frontDragonLimelight;
    LimelightState*         m_currentstate;
    std::map<LIMELIGHT_STATES, LimelightState*> m_limelightstates;
    double                  m_tolerance = 2.0;  //allows for 2 degrees of error
};


