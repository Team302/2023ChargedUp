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
#include <string>

// FRC includes

// Team 302 includes

#include <DragonVision/DragonVision.h>
#include <hw\factories\LimelightFactory.h>
#include <DragonVision\RetroReflective\RetroReflective.h>
#include <DragonVision\Cube\Cube.h>
#include <DragonVision\Cone\Cone.h>
#include <DragonVision\AprilTag\AprilTag.h>
#include <utils/Logger.h>
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
	Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, std::string("DragonVision"), std::string("GetDragonVision"), "Got");
	return DragonVision::m_dragonVision;
}

//state functions
DragonVision::DragonVision(std::string stateName, int stateId): State(stateName, stateId),
						   m_frontDragonLimelight(LimelightFactory::GetLimelightFactory()->GetLimelight())			
{
	/// @TODO: Need to find the real indexes of each pipeline and put them in constructor
	m_limelightstates[RETROREFLECTIVE] = new RetroReflective(m_frontDragonLimelight, 0);
	m_limelightstates[APRILTAG] = new AprilTag(m_frontDragonLimelight, 1);
	m_limelightstates[CUBE] = new Cube(m_frontDragonLimelight, 2);
	m_limelightstates[CONE] = new Cone(m_frontDragonLimelight, 3);
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
	m_frontDragonLimelight->SetPipeline(m_currentstate->GetPipelineIndex());
}

void DragonVision::SetLimelightStates(DragonVision::LIMELIGHT_STATES limelightstate)
{
    m_currentstate = m_limelightstates[limelightstate];
} 

void DragonVision::SetCurrentState(DragonVision::LIMELIGHT_STATES limelightstate)
{
	auto itr = m_limelightstates.find(limelightstate);
	if (itr != m_limelightstates.end())
	{
		if (m_currentstate != itr->second)
		{
			m_currentstate = itr->second;
		}
	}
}

//Aligned-with functions

bool DragonVision::AlignedWithCubeNode()
{
	SetCurrentState(LIMELIGHT_STATES::APRILTAG);

	if(m_currentstate->HasTarget())
	{
		int id = static_cast<AprilTag*>(m_currentstate)->GetTagID();
		if(id != 5 && id !=4) //4 and 5 are the ids of the substations
		{
			return m_currentstate->GetTargetHorizontalOffset().to<double>() < m_tolerance;
		}
		else
		{
			return false;
		}
	}
	else
	{
		return false;
	}
}
bool DragonVision::AlignedWithConeNode() 
{
	SetCurrentState(LIMELIGHT_STATES::RETROREFLECTIVE);
	
	if(m_currentstate->HasTarget())
	{
		return m_currentstate->GetTargetHorizontalOffset().to<double>() < m_tolerance;
	}
	else
	{
		return false;
	}
}

bool DragonVision::AlignedWithSubstation() 
{
	SetCurrentState(LIMELIGHT_STATES::APRILTAG);

	if(m_currentstate->HasTarget())
	{
		int id = static_cast<AprilTag*>(m_currentstate)->GetTagID();
		if(id == 5 || id == 4) //4 and 5 are the ids of the substations
		{	
			return m_currentstate->GetTargetHorizontalOffset().to<double>() < m_tolerance;
		}
		else
		{
			return false;
		}
	}
	else
	{
		return false;
	}
}

bool DragonVision::AlignedWithCubeGamePiece() 
{
	SetCurrentState(LIMELIGHT_STATES::CUBE);

	if(m_currentstate->HasTarget())
	{
		return m_currentstate->GetTargetHorizontalOffset().to<double>() < m_tolerance;
	}
	else
	{
		return false;
	}
}   

bool DragonVision::AlignedWithConeGamePiece() 
{
	SetCurrentState(LIMELIGHT_STATES::CONE);

	if(m_currentstate->HasTarget())
	{
		return m_currentstate->GetTargetHorizontalOffset().to<double>() < m_tolerance;
	}
	else
	{
		return false;
	}
}

// Distance methods
units::length::inch_t DragonVision::DistanceFromCubeNode() 
{
	SetCurrentState(LIMELIGHT_STATES::APRILTAG);

	if(m_currentstate->HasTarget())
	{
		int id = static_cast<AprilTag*>(m_currentstate)->GetTagID();
		if(id != 5 && id !=4) //4 and 5 are the ids of the substations
		{
			return m_currentstate->EstimateTargetDistance();
		}
		else
		{
			units::length::inch_t(-1.0);
		}
	}
	else
	{
		return units::length::inch_t(-1.0);
	}
}

units::length::inch_t DragonVision::DistanceFromConeNode()
{
	SetCurrentState(LIMELIGHT_STATES::RETROREFLECTIVE);

	if(m_currentstate->HasTarget())
	{
		return m_currentstate->EstimateTargetDistance();
	}
	else
	{
		return units::length::inch_t(-1.0);
	}
}

units::length::inch_t DragonVision::DistanceFromSubstation() 
{
	SetCurrentState(LIMELIGHT_STATES::APRILTAG);

	if(m_currentstate->HasTarget())
	{
		int id = static_cast<AprilTag*>(m_currentstate)->GetTagID();
		if(id == 5 || id == 4) //4 and 5 are the ids of the substations
		{	
			return m_currentstate->EstimateTargetDistance();
		}
		else
		{
			return units::length::inch_t(-1.0);
		}
	}
	else
	{
		return units::length::inch_t(-1.0);
	}
}

units::length::inch_t DragonVision::DistanceFromCubeGamePiece() 
{
	SetCurrentState(LIMELIGHT_STATES::CUBE);

	if(m_currentstate->HasTarget())
	{
		return m_currentstate->EstimateTargetDistance();
	}
	else
	{
		return units::length::inch_t(-1.0);
	}
}

units::length::inch_t DragonVision::DistanceFromConeGamePiece() 
{
	SetCurrentState(LIMELIGHT_STATES::CONE);

	if(m_currentstate->HasTarget())
	{
		return m_currentstate->EstimateTargetDistance();
	}
	else
	{
		return units::length::inch_t(-1.0);
	}
}

//Angle Functions
units::angle::degree_t DragonVision::AngleFromCubeNode() 
{
	SetCurrentState(LIMELIGHT_STATES::APRILTAG);

	if(m_currentstate->HasTarget())
	{
		int id = static_cast<AprilTag*>(m_currentstate)->GetTagID();
		if(id != 5 && id !=4) //4 and 5 are the ids of the substations
		{
			return m_currentstate->GetTargetHorizontalOffset();
		}
		else
		{
			return units::angle::degree_t(180.0); //default "no-target" value
		}
	}
	else
	{
		return units::angle::degree_t(180.0); //default "no-target" value
	}
}

units::angle::degree_t DragonVision::AngleFromConeNode() 
{
	SetCurrentState(LIMELIGHT_STATES::RETROREFLECTIVE);

	if(m_currentstate->HasTarget())
	{
		return m_currentstate->GetTargetHorizontalOffset();
	}
	else
	{
		return units::angle::degree_t(180.0); //default "no-target" value
	}
}

units::angle::degree_t DragonVision::AngleFromCubeGamePiece() 
{
	SetCurrentState(LIMELIGHT_STATES::CUBE);

	if(m_currentstate->HasTarget())
	{
		return m_currentstate->GetTargetHorizontalOffset();
	}
	else
	{
		return units::angle::degree_t(180.0); //default "no-target" value
	}
} 

units::angle::degree_t DragonVision::AngleFromConeGamePiece() 
{
	SetCurrentState(LIMELIGHT_STATES::CONE);

	if(m_currentstate->HasTarget())
	{
		return m_currentstate->GetTargetHorizontalOffset();
	}
	else
	{
		return units::angle::degree_t(180.0); //default "no-target" value
	}
}

units::angle::degree_t DragonVision::AngleFromSubstation() 
{
	SetCurrentState(LIMELIGHT_STATES::APRILTAG);

	if(m_currentstate->HasTarget())
	{
		int id = static_cast<AprilTag*>(m_currentstate)->GetTagID();
		if(id == 5 || id == 4) //4 and 5 are the ids of the substations
		{	
			return m_currentstate->GetTargetHorizontalOffset();
		}
		else
		{
			return units::angle::degree_t(180.0); //default "no-target" value
		}
	}
	else
	{
		return units::angle::degree_t(180.0); //default "no-target" value
	}
}

// position function

frc::Pose2d DragonVision::GetRobotPosition()
{
	SetCurrentState(LIMELIGHT_STATES::APRILTAG);

	if(m_currentstate->HasTarget())
	{
		return static_cast<AprilTag*>(m_currentstate)->GetRobotPose();
	}
	else
	{
		return frc::Pose2d(); //default "no-target" value
	}
}

