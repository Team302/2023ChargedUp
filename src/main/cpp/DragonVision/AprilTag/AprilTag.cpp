
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

//FRC Includes
#include <networktables/DoubleArrayTopic.h>

#include <DragonVision/AprilTag/AprilTag.h>

using namespace std;
AprilTag::AprilTag(DragonLimelight*    dragonlimelight) : LimelightState(dragonlimelight), m_dragonlimelight(dragonlimelight)             /// <I> - height of second target)
{

}
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
    
    if ( abs(m_limelight->GetRotation().to<double>()) < 1.0 )
    {
        return m_dragonlimelight->GetTx();
    }
    else if ( abs(m_limelight->GetRotation().to<double>()-90.0) < 1.0 )
    {
        return -1.0 * m_dragonlimelight->GetTy();
    }
    else if ( abs(m_limelight->GetRotation().to<double>()-180.0) < 1.0 )
    {
        return -1.0 * m_dragonlimelight->GetTx();
    }
    else if ( abs(m_limelight->GetRotation().to<double>()-270.0) < 1.0 )
    {
        return m_dragonlimelight->GetTy();
    }
    Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, string("DragonLimelight"), string("GetTargetVerticalOffset"), string("Invalid limelight rotation"));
    return m_dragonlimelight->GetTx();
}

units::angle::degree_t AprilTag::GetTargetVerticalOffset() const
{
    if ( abs(m_limelight->GetRotation().to<double>()) < 1.0 )
    {
        return m_dragonlimelight->GetTy();
    }
    else if ( abs(m_limelight->GetRotation().to<double>()-90.0) < 1.0 )
    {
        return m_dragonlimelight->GetTx();
    }
    else if ( abs(m_limelight->GetRotation().to<double>()-180.0) < 1.0 )
    {
        return -1.0 * m_dragonlimelight->GetTy();
    }
    else if ( abs(m_limelight->GetRotation().to<double>()-270.0) < 1.0 )
    {
        return -1.0 * m_dragonlimelight->GetTx();
    }
    Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, string("DragonLimelight"), string("GetTargetVerticalOffset"), string("Invalid limelight rotation"));
    return m_dragonlimelight->GetTy();   
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

frc::Pose2d AprilTag::GetRobotPose() const
{
    if(m_networktable != nullptr)
    {
        nt::DoubleArrayTopic arrayTopic = m_networktable->GetDoubleArrayTopic("botpose");
        auto subscriber = arrayTopic.Subscribe(std::span<const double>());
        auto value = subscriber.Get(std::span<const double>());
        if(value.size() > 0)
        {
            /// DEBUGGING
            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("AprilTag"), string("GetRobotPose"), "Span size greater than 0");
            
            double xPos = value[0] + 16.459; //add 16.459 to accoutn for limelight origin being in center
                                    //cut the two offset in half
            double yPos = value[1] + 8.23; //add 16.459 to accoutn for limelight origin being in center
            double zRotation = value[5];
            frc::Pose2d botPose = frc::Pose2d(units::length::meter_t(xPos), units::length::meter_t(yPos), frc::Rotation2d(units::angle::degree_t(zRotation)));
            return botPose;
        }
        else
        {
            /// DEBUGGING
            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("AprilTag"), string("GetRobotPose"), "Span size not greater than 0");
            return frc::Pose2d();
        }
    }
}

units::length::inch_t AprilTag::EstimateTargetDistance() const
{
    units::angle::degree_t angleFromHorizon = (m_limelight->GetMountingAngle() + GetTargetVerticalOffset());
    units::angle::radian_t angleRad = angleFromHorizon;
    double tanAngle = tan(angleRad.to<double>());

    auto deltaHgt = m_limelight->GetTargetHeight()-m_limelight->GetMountingHeight();

    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("DragonLimelight"), string("mounting angle "), m_limelight->GetMountingAngle().to<double>());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("DragonLimelight"), string("target vertical angle "), GetTargetVerticalOffset().to<double>());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("DragonLimelight"), string("angle radians "), angleRad.to<double>());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("DragonLimelight"), string("deltaH "), deltaHgt.to<double>());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("DragonLimelight"), string("tan angle "), tanAngle);
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("DragonLimelight"), string("distance "), ((m_limelight->GetTargetHeight()-m_limelight->GetMountingHeight()) / tanAngle).to<double>());

    return (m_limelight->GetTargetHeight()-m_limelight->GetMountingHeight()) / tanAngle;
}

