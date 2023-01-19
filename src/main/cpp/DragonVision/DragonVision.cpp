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

// m_frontDragonLimelight->MethodName
// C++ Includes
#include <string>

// FRC includes

// Team 302 includes

#include <mechanisms/controllers/ControlData.h>
#include <DragonVision/DragonVision.h>



// Third Party Includes

using namespace std;

DragonVision* DragonVision::m_dragonvision = nullptr;
DragonVision* DragonVision::GetDragonVision()
{
	if ( DragonVision::m_dragonvision == nullptr )
	{
		DragonVision::m_dragonvision = new DragonVision();
	}
	return DragonVision::m_dragonvision;
}

//state functions


DragonVision::DragonVision()
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



//ALligned-with functions

bool AlignedWithCubeNode()
{
	return false;
}
bool AlignedWithConeNode() 
{
	
}
bool AlignedWithSubstation() 
{
	return false;
}
bool AlignedWithCubeGamePiece()
{
	return false;
}   
bool AlignedWithConeGamePiece()
{
	return false;


// Distance methods

}
int DistanceFromCubeNode()
{
	return 0;
}
int DistanceFromConeNode()
{
	return 0;
}
int DistanceFromSubstation()
{
	return 0;
}
int DistanceFromCubeGamePiece()
{
	return 0;
}
int DistanceFromConeGamePiece()
{
	return 0;
}

//Angel Functions


int AngelFromCubeNode()
{
	return 0;
}
units::angle::degree_t AngelFromConeNode()
{
	if (GetTargetSkew() < units::angle::degree_t(30))
	{
		return units::angle::degree_t(0.0);
	}
	else
	{
		return units::angle::degree_t(0.0);
	}
}
int AngelFromCubeGamePiece()
{
	return 0;
}
int AngelFromConeGamePiece()
{
	return 0;
}
int AngelFromSubstation()
{
	return 0;
}

// position function

int GetRobotPosition()
{
	return 0;
}

