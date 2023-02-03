

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




