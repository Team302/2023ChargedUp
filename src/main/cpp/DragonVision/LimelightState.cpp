



#include <DragonVision/LimelightState.h>










using namespace nt;
using namespace std;


LimelightState::LimelightState
(
    DragonLimelight*     dragonlimelight            /// <I> - height of second target
) : m_limelight(dragonlimelight)
{
    //SetLEDMode( DragonLimelight::LED_MODE::LED_OFF);
}

std::vector<double> DragonLimelight::Get3DSolve() const
{
    std::vector<double> output;
    return output;
}

bool DragonLimelight::HasTarget() const
{
    auto nt = m_networktable.get();
    if (nt != nullptr)
    {
        return ( nt->GetNumber("tv", 0.0) > 0.1 );
    }
    return false;
}

units::angle::degree_t DragonLimelight::GetTx() const
{
    auto nt = m_networktable.get();
    if (nt != nullptr)
    {
        return units::angle::degree_t(nt->GetNumber("tx", 0.0));
    }
    return units::angle::degree_t(0.0);
}
 
units::angle::degree_t DragonLimelight::GetTy() const
{
    auto nt = m_networktable.get();
    if (nt != nullptr)
    {
        return units::angle::degree_t(nt->GetNumber("ty", 0.0));
    }
    return units::angle::degree_t(0.0);
}

units::angle::degree_t DragonLimelight::GetTargetHorizontalOffset() const
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

units::angle::degree_t DragonLimelight::GetTargetVerticalOffset() const
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

double DragonLimelight::GetTargetArea() const
{
    auto nt = m_networktable.get();
    if (nt != nullptr)
    {
        return nt->GetNumber("ta", 0.0);
    }
    return 0.0;
}

units::angle::degree_t DragonLimelight::GetTargetSkew() const
{
 //   auto nt = m_networktable.get();
    if (m_networktable != nullptr)
    {
        return units::angle::degree_t(m_networktable->GetNumber("ts", 0.0));
    }
    return units::angle::degree_t(0.0);
}

units::time::microsecond_t DragonLimelight::GetPipelineLatency() const
{
    auto nt = m_networktable.get();
    if (nt != nullptr)
    {
        return units::time::second_t(nt->GetNumber("tl", 0.0));
    }
    return units::time::second_t(0.0);
}

void DragonLimelight::PrintValues()
{
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("DragonLimelight"), string("PrintValues HasTarget"), to_string( HasTarget() ) );    
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("DragonLimelight"), string("PrintValues XOffset"), to_string( GetTargetHorizontalOffset().to<double>() ) ); 
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("DragonLimelight"), string("PrintValues YOffset"), to_string( GetTargetVerticalOffset().to<double>() ) ); 
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("DragonLimelight"), string("PrintValues Area"), to_string( GetTargetArea() ) ); 
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("DragonLimelight"), string("PrintValues Skew"), to_string( GetTargetSkew().to<double>() ) ); 
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("DragonLimelight"), string(":PrintValues Latency"), to_string( GetPipelineLatency().to<double>() ) ); 
}

units::length::inch_t DragonLimelight::EstimateTargetDistance() const
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
