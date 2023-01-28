
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
/**
#pragma once

#include <DragonVision/AprilTag/AprilTag.h>

using namespace std;

bool AprilTag::HasTarget() const
{
   auto nt = m_networktable.get();
    if (nt != nullptr)
    {
        return ( nt->GetNumber("tv", 0.0) > 0.1 );
    }
    return false;
}

units::angle::degree_t AprilTag::GetTargetHorizontalOffset() const
{
    if ( abs(m_rotation.to<double>()) < 1.0 )
    {
        return GetTx();
    }
    else if ( abs(m_rotation.to<double>()-90.0) < 1.0 )
    {
        return -1.0 * GetTy();
    }
    else if ( abs(m_rotation.to<double>()-180.0) < 1.0 )
    {
        return -1.0 * GetTx();
    }
    else if ( abs(m_rotation.to<double>()-270.0) < 1.0 )
    {
        return GetTy();
    }
    Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, string("DragonLimelight"), string("GetTargetVerticalOffset"), string("Invalid limelight rotation"));
    return GetTx();
}

units::angle::degree_t AprilTag::GetTargetVerticalOffset() const
{
    if ( abs(m_rotation.to<double>()) < 1.0 )
    {
        return GetTy();
    }
    else if ( abs(m_rotation.to<double>()-90.0) < 1.0 )
    {
        return GetTx();
    }
    else if ( abs(m_rotation.to<double>()-180.0) < 1.0 )
    {
        return -1.0 * GetTy();
    }
    else if ( abs(m_rotation.to<double>()-270.0) < 1.0 )
    {
        return -1.0 * GetTx();
    }
    Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, string("DragonLimelight"), string("GetTargetVerticalOffset"), string("Invalid limelight rotation"));
    return GetTy();   
}

double AprilTag::GetTargetArea() const
{
    auto nt = m_networktable.get();
    if (nt != nullptr)
    {
        return nt->GetNumber("ta", 0.0);
    }
    return 0.0;
}

units::angle::degree_t AprilTag::GetTargetSkew() const
{
 //   auto nt = m_networktable.get();
    if (m_networktable != nullptr)
    {
        return units::angle::degree_t(m_networktable->GetNumber("ts", 0.0));
    }
    return units::angle::degree_t(0.0);
}

units::time::microsecond_t AprilTag::GetPipelineLatency() const
{
    auto nt = m_networktable.get();
    if (nt != nullptr)
    {
        return units::time::second_t(nt->GetNumber("tl", 0.0));
    }
    return units::time::second_t(0.0);
}

std::vector<double> AprilTag::Get3DSolve() const
{
    std::vector<double> output;
    return output;
}

units::length::inch_t AprilTag::EstimateTargetDistance() const
{
    units::angle::degree_t angleFromHorizon = (GetMountingAngle() + GetTargetVerticalOffset());
    units::angle::radian_t angleRad = angleFromHorizon;
    double tanAngle = tan(angleRad.to<double>());

    auto deltaHgt = GetTargetHeight()-GetMountingHeight();

    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("DragonLimelight"), string("mounting angle "), GetMountingAngle().to<double>());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("DragonLimelight"), string("target vertical angle "), GetTargetVerticalOffset().to<double>());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("DragonLimelight"), string("angle radians "), angleRad.to<double>());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("DragonLimelight"), string("deltaH "), deltaHgt.to<double>());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("DragonLimelight"), string("tan angle "), tanAngle);
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("DragonLimelight"), string("distance "), ((GetTargetHeight()-GetMountingHeight()) / tanAngle).to<double>());

    return (GetTargetHeight()-GetMountingHeight()) / tanAngle;
}
*/
