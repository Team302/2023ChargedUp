
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

// FRC includes
#include <frc/PowerDistribution.h>

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
        bool AllStickyFaults = StickyFaultsValue.Channel0BreakerFault || StickyFaultsValue.Channel1BreakerFault || StickyFaultsValue.Channel12BreakerFault || StickyFaultsValue.Channel13BreakerFault || StickyFaultsValue.Channel14BreakerFault || StickyFaultsValue.Channel15BreakerFault || StickyFaultsValue.Channel16BreakerFault || StickyFaultsValue.Channel17BreakerFault || StickyFaultsValue.Channel18BreakerFault || StickyFaultsValue.Channel19BreakerFault || StickyFaultsValue.Channel20BreakerFault || StickyFaultsValue.Channel21BreakerFault || StickyFaultsValue.Channel22BreakerFault || StickyFaultsValue.Channel23BreakerFault || StickyFaultsValue.Brownout || StickyFaultsValue.CanWarning || StickyFaultsValue.CanBusOff || StickyFaultsValue.HasReset;
    }
    return PDPFactory::PDPHasStickyFaults();
}