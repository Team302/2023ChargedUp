
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
#include <string>
#include <State.h>
#include <hw/DragonLimelight.h>
#include <DragonVision/DragonVisionTarget.h>

class DragonLimelight;
class DragonVision
{
public:
    static DragonVision *GetDragonVision();

    enum PIPELINE_MODE
    {
        RETRO_REFLECTIVE,
        APRIL_TAG,
        CONE,
        CUBE
    };

    enum LIMELIGHT_POSITION
    {
        FRONT,
        BACK
    };

    void setPipeline(PIPELINE_MODE mode, LIMELIGHT_POSITION position);
    units::length::inch_t GetDistanceToTarget(LIMELIGHT_POSITION position) const;
    units::angle::degree_t GetHorizontalAngleToTarget(LIMELIGHT_POSITION position) const;
    units::angle::degree_t GetVerticalAngleToTarget(LIMELIGHT_POSITION position) const;

    int GetRobotPosition() const;

private:
    DragonVision();
    ~DragonVision() = default;

    static DragonVision *m_dragonVision;

    DragonLimelight *m_frontDragonLimelight;
    DragonLimelight *m_backDragonLimelight;
};
