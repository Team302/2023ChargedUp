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


#include <string>
// Third Party Includes

using namespace std;

DragonVision* DragonVision::m_dragonVision = nullptr;
DragonVision* DragonVision::GetDragonVision
(
	
)
{
	if ( DragonVision::m_dragonVision == nullptr )
	{
		DragonVision::m_dragonVision = new DragonVision(std::string("DragonVision"), int(-1));
	}
	return DragonVision::m_dragonVision;
}

//state functions


DragonVision::DragonVision(std::string stateName, int stateId): State(stateName, stateId),
							  m_frontDragonLimelight(LimelightFactory::GetLimelightFactory()->GetLimelight())				
{
	
}
void DragonVision::Init() 
{

}
void DragonVision::Exit() 
{

}
bool DragonVision::AtTarget() const
{
	return false;
}
void DragonVision::Run() 
{

}


//Alligned-with functions

bool DragonVision::AlignedWithCubeNode() const
{
	return m_frontDragonLimelight->HasTarget();
}
bool DragonVision::AlignedWithConeNode() const
{
	return m_frontDragonLimelight->HasTarget();
}
bool DragonVision::AlignedWithSubstation() const
{
	return m_frontDragonLimelight->HasTarget();
}
bool DragonVision::AlignedWithCubeGamePiece() const
{
	return m_frontDragonLimelight->HasTarget();
}   
bool DragonVision::AlignedWithConeGamePiece() const
{
	return m_frontDragonLimelight->HasTarget();
}

// Distance methods

units::length::inch_t DragonVision::DistanceFromCubeNode() const
{
	return m_frontDragonLimelight->EstimateTargetDistance();
}
units::length::inch_t DragonVision::DistanceFromConeNode() const
{
	return m_frontDragonLimelight->EstimateTargetDistance();
}
units::length::inch_t DragonVision::DistanceFromSubstation() const
{
	return m_frontDragonLimelight->EstimateTargetDistance();
}
units::length::inch_t DragonVision::DistanceFromCubeGamePiece() const
{
	return m_frontDragonLimelight->EstimateTargetDistance();
}
units::length::inch_t DragonVision::DistanceFromConeGamePiece() const
{
	return m_frontDragonLimelight->EstimateTargetDistance();
}

//Angel Functions

// Ah yes, the spelling skills of Ben.


units::angle::degree_t DragonVision::AngleFromCubeNode() const
{
	return m_frontDragonLimelight->GetTargetSkew();
}
units::angle::degree_t DragonVision::AngleFromConeNode() const
{
	return m_frontDragonLimelight->GetTargetSkew();
}
units::angle::degree_t DragonVision::AngleFromCubeGamePiece() const
{
	return m_frontDragonLimelight->GetTargetSkew();
} 
units::angle::degree_t DragonVision::AngleFromConeGamePiece() const
{
	return m_frontDragonLimelight->GetTargetSkew();
}
units::angle::degree_t DragonVision::AngleFromSubstation() const
{
	return m_frontDragonLimelight->GetTargetSkew();
}

// position function

int DragonVision::GetRobotPosition()  const
{
	return 0;
}

