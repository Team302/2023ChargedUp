
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

//========================================================================================================
/// @class PDPFactory
/// @brief This creates the pigeon
//========================================================================================================

// C++ Includes
#include <string>

// FRC includes
#include <frc/PowerDistribution.h>
#include <utils/logging/Logger.h>

// Team 302 includes
#include <hw/factories/PDPFactory.h>

// Third Party Includes

using namespace std;
using namespace frc;

/// @brief  Find or create the analog input factory
/// @return PDPFactory* pointer to the factory
PDPFactory *PDPFactory::m_factory = nullptr;
PDPFactory *PDPFactory::GetFactory()
{
    if (PDPFactory::m_factory == nullptr)
    {
        PDPFactory::m_factory = new PDPFactory();
    }
    return PDPFactory::m_factory;
}

PDPFactory::PDPFactory()
{
    m_pdp = nullptr;
}

/// @brief  Create the requested analog input
/// @return shared_ptr<DragonPigeon>   the mechanism or nullptr if mechanism doesn't
///         exist and cannot be created.
PowerDistribution *PDPFactory::CreatePDP(
    int canID,
    PowerDistribution::ModuleType type)

{
    if (m_pdp == nullptr)
    {
        m_pdp = new PowerDistribution(canID, type);
        ClearStickyFaults();
    }
    return m_pdp;
}

void PDPFactory::ClearStickyFaults()
{
    if (m_pdp != nullptr)
    {
        m_pdp->ClearStickyFaults();
    }
}

bool PDPFactory::PDPHasStickyFaults()
{
    if (m_pdp != nullptr)
    {
        frc::PowerDistribution::StickyFaults StickyFaultsValue = m_pdp->GetStickyFaults();

        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "PDP Sticky Faults Value", "Channel 0 Breaker Fault", StickyFaultsValue.Channel0BreakerFault != 0);
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "PDP Sticky Faults Value", "Channel 1 Breaker Fault", StickyFaultsValue.Channel1BreakerFault != 0);
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "PDP Sticky Faults Value", "Channel 2 Breaker Fault", StickyFaultsValue.Channel2BreakerFault != 0);
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "PDP Sticky Faults Value", "Channel 3 Breaker Fault", StickyFaultsValue.Channel3BreakerFault != 0);
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "PDP Sticky Faults Value", "Channel 4 Breaker Fault", StickyFaultsValue.Channel4BreakerFault != 0);
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "PDP Sticky Faults Value", "Channel 5 Breaker Fault", StickyFaultsValue.Channel5BreakerFault != 0);
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "PDP Sticky Faults Value", "Channel 6 Breaker Fault", StickyFaultsValue.Channel6BreakerFault != 0);
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "PDP Sticky Faults Value", "Channel 7 Breaker Fault", StickyFaultsValue.Channel7BreakerFault != 0);
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "PDP Sticky Faults Value", "Channel 8 Breaker Fault", StickyFaultsValue.Channel8BreakerFault != 0);
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "PDP Sticky Faults Value", "Channel 9 Breaker Fault", StickyFaultsValue.Channel9BreakerFault != 0);
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "PDP Sticky Faults Value", "Channel 10 Breaker Fault", StickyFaultsValue.Channel10BreakerFault != 0);
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "PDP Sticky Faults Value", "Channel 11 Breaker Fault", StickyFaultsValue.Channel11BreakerFault != 0);
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "PDP Sticky Faults Value", "Channel 12 Breaker Fault", StickyFaultsValue.Channel12BreakerFault != 0);
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "PDP Sticky Faults Value", "Channel 13 Breaker Fault", StickyFaultsValue.Channel13BreakerFault != 0);
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "PDP Sticky Faults Value", "Channel 14 Breaker Fault", StickyFaultsValue.Channel14BreakerFault != 0);
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "PDP Sticky Faults Value", "Channel 15 Breaker Fault", StickyFaultsValue.Channel15BreakerFault != 0);
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "PDP Sticky Faults Value", "Channel 16 Breaker Fault", StickyFaultsValue.Channel16BreakerFault != 0);
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "PDP Sticky Faults Value", "Channel 17 Breaker Fault", StickyFaultsValue.Channel17BreakerFault != 0);
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "PDP Sticky Faults Value", "Channel 18 Breaker Fault", StickyFaultsValue.Channel18BreakerFault != 0);
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "PDP Sticky Faults Value", "Channel 19 Breaker Fault", StickyFaultsValue.Channel19BreakerFault != 0);
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "PDP Sticky Faults Value", "Channel 20 Breaker Fault", StickyFaultsValue.Channel20BreakerFault != 0);
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "PDP Sticky Faults Value", "Channel 21 Breaker Fault", StickyFaultsValue.Channel21BreakerFault != 0);
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "PDP Sticky Faults Value", "Channel 22 Breaker Fault", StickyFaultsValue.Channel22BreakerFault != 0);
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "PDP Sticky Faults Value", "Channel 23 Breaker Fault", StickyFaultsValue.Channel23BreakerFault != 0);
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "PDP Sticky Faults Value", "Brown Out", StickyFaultsValue.Brownout != 0);
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "PDP Sticky Faults Value", "Can Bus Off", StickyFaultsValue.CanBusOff != 0);
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "PDP Sticky Faults Value", "Can Warning", StickyFaultsValue.CanWarning != 0);
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "PDP Sticky Faults Value", "Has Reset", StickyFaultsValue.HasReset != 0);

        return (StickyFaultsValue.Channel0BreakerFault || StickyFaultsValue.Channel1BreakerFault || StickyFaultsValue.Channel2BreakerFault || StickyFaultsValue.Channel3BreakerFault || StickyFaultsValue.Channel4BreakerFault || StickyFaultsValue.Channel5BreakerFault || StickyFaultsValue.Channel6BreakerFault || StickyFaultsValue.Channel7BreakerFault || StickyFaultsValue.Channel8BreakerFault || StickyFaultsValue.Channel9BreakerFault || StickyFaultsValue.Channel10BreakerFault || StickyFaultsValue.Channel11BreakerFault || StickyFaultsValue.Channel12BreakerFault || StickyFaultsValue.Channel13BreakerFault || StickyFaultsValue.Channel14BreakerFault || StickyFaultsValue.Channel15BreakerFault || StickyFaultsValue.Channel16BreakerFault || StickyFaultsValue.Channel17BreakerFault || StickyFaultsValue.Channel18BreakerFault || StickyFaultsValue.Channel19BreakerFault || StickyFaultsValue.Channel20BreakerFault || StickyFaultsValue.Channel21BreakerFault || StickyFaultsValue.Channel22BreakerFault || StickyFaultsValue.Channel23BreakerFault || StickyFaultsValue.Brownout || StickyFaultsValue.CanWarning || StickyFaultsValue.CanBusOff || StickyFaultsValue.HasReset);
    }
    return false;
}
