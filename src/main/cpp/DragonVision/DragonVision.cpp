//====================================================================================================================================================
/// Copyright 2022 Lake Orion Robotics FIRST Team 302
///
/// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
/// to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
/// and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
///
/// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
///
/// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
/// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
/// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE
/// OR OTHER DEALINGS IN THE SOFTWARE.
//====================================================================================================================================================

// C++ Includes

// FRC includes

// Team 302 includes

#include <DragonVision/DragonVision.h>
#include <hw\factories\LimelightFactory.h>

#include <string>
// Third Party Includes

using namespace std;
DragonVision *DragonVision::m_dragonVision = nullptr;
DragonVision *DragonVision::GetDragonVision()
{
	if (DragonVision::m_dragonVision == nullptr)
	{
		DragonVision::m_dragonVision = new DragonVision();
	}
	return DragonVision::m_dragonVision;
}

// state functions

DragonVision::DragonVision() : m_frontDragonLimelight(LimelightFactory::GetLimelightFactory()->GetLimelight(LimelightUsages::PRIMARY)),
							   m_backDragonLimelight(LimelightFactory::GetLimelightFactory()->GetLimelight(LimelightUsages::SECONDARY))
{
}

void DragonVision::setPipeline(PIPELINE_MODE mode, LIMELIGHT_POSITION position)
{
}

units::length::inch_t DragonVision::GetDistanceToTarget(LIMELIGHT_POSITION position) const
{
	return units::length::inch_t(0);
}

units::angle::degree_t DragonVision::GetHorizontalAngleToTarget(LIMELIGHT_POSITION position) const
{
	return units::angle::degree_t(0);
}

units::angle::degree_t DragonVision::GetVerticalAngleToTarget(LIMELIGHT_POSITION position) const
{
	return units::angle::degree_t(0);
}

int DragonVision::GetRobotPosition() const
{
	return 0;
}
