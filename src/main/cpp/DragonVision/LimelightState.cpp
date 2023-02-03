

#include <DragonVision/LimelightState.h>

using namespace nt;
using namespace std;


LimelightState::LimelightState
(
    DragonLimelight*     dragonlimelight,            /// <I> - height of second target
    int index
) : m_limelight(dragonlimelight),
    m_networktable(dragonlimelight->GetNetworkTable()),
    m_index(index)
{

}

bool LimelightState::HasTarget() const
{
   auto nt = m_networktable.get();
    if (nt != nullptr)
    {
        return ( nt->GetNumber("tv", 0.0) > 0.1 );
    }
    return false;
}

units::angle::degree_t LimelightState::GetTargetHorizontalOffset() const
{
    if ( abs(m_limelight->GetRotation().to<double>()) < 1.0 )
    {
        return m_limelight->GetTx();
    }
    else if ( abs(m_limelight->GetRotation().to<double>()-90.0) < 1.0 )
    {
        return -1.0 * m_limelight->GetTy();
    }
    else if ( abs(m_limelight->GetRotation().to<double>()-180.0) < 1.0 )
    {
        return -1.0 * m_limelight->GetTx();
    }
    else if ( abs(m_limelight->GetRotation().to<double>()-270.0) < 1.0 )
    {
        return m_limelight->GetTy();
    }
    Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, string("DragonLimelight"), string("GetTargetVerticalOffset"), string("Invalid limelight rotation"));
    return m_limelight->GetTx();
}

units::angle::degree_t LimelightState::GetTargetVerticalOffset() const
{
    if ( abs(m_limelight->GetRotation().to<double>()) < 1.0 )
    {
        return m_limelight->GetTy();
    }
    else if ( abs(m_limelight->GetRotation().to<double>()-90.0) < 1.0 )
    {
        return m_limelight->GetTx();
    }
    else if ( abs(m_limelight->GetRotation().to<double>()-180.0) < 1.0 )
    {
        return -1.0 * m_limelight->GetTy();
    }
    else if ( abs(m_limelight->GetRotation().to<double>()-270.0) < 1.0 )
    {
        return -1.0 * m_limelight->GetTx();
    }
    Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, string("DragonLimelight"), string("GetTargetVerticalOffset"), string("Invalid limelight rotation"));
    return m_limelight->GetTy();   
}

double LimelightState::GetTargetArea() const
{
    auto nt = m_networktable.get();
    if (nt != nullptr)
    {
        return nt->GetNumber("ta", 0.0);
    }
    return 0.0;
}

units::angle::degree_t LimelightState::GetTargetSkew() const
{
 //   auto nt = m_networktable.get();
    if (m_networktable != nullptr)
    {
        return units::angle::degree_t(m_networktable->GetNumber("ts", 0.0));
    }
    return units::angle::degree_t(0.0);
}

units::time::microsecond_t LimelightState::GetPipelineLatency() const
{
    auto nt = m_networktable.get();
    if (nt != nullptr)
    {
        return units::time::second_t(nt->GetNumber("tl", 0.0));
    }
    return units::time::second_t(0.0);
}

units::length::inch_t LimelightState::EstimateTargetDistance() const
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