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

// C++ Includes
#include <memory>
#include <string>

// Team 302 includes
#include <auton/drivePrimitives/DragonTrajectoryUtils.h>
#include <auton/drivePrimitives/ResetPositionPathPlanner.h>
#include <auton/PrimitiveParams.h>
#include <auton/drivePrimitives/IPrimitive.h>
#include <chassis/ChassisFactory.h>
#include <hw/factories/PigeonFactory.h>
#include <utils/logging/Logger.h>
#include <DragonVision/DragonVision.h>

using namespace std;
using namespace frc;

ResetPositionPathPlanner::ResetPositionPathPlanner() : m_chassis(ChassisFactory::GetChassisFactory()->GetIChassis())
{
}

void ResetPositionPathPlanner::Init(PrimitiveParams *params)
{

    // m_trajectory = pathplanner::PathPlanner::loadPath(params->GetPathName(), pathplanner::PathPlanner::getConstraintsFromPath(params->GetPathName()));
    m_trajectory = pathplanner::PathPlanner::loadPath(params->GetPathName(), pathplanner::PathConstraints(4.0_mps, 2.0_mps_sq));

    auto pigeon = PigeonFactory::GetFactory()->GetCenterPigeon();
    // pigeon->ReZeroPigeon(m_trajectory.getInitialState().holonomicRotation.Degrees().to<double>());

    m_chassis->ResetPose(m_trajectory.getInitialHolonomicPose());

    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("Reset Position PathPlanner"), string("Auton Info: ResetPosX"), m_chassis.get()->GetPose().X().to<double>());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("Reset Position PathPlanner"), string("Auton Info: ResetPosY"), m_chassis.get()->GetPose().Y().to<double>());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("Reset Position PathPlanner"), string("Auton Info: ResetPosRot"), m_chassis.get()->GetPose().Rotation().Degrees().to<double>());

    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("Reset Position PathPlanner"), string("Auton Info: InitialPoseX"), m_trajectory.getInitialHolonomicPose().X().to<double>());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("Reset Position PathPlanner"), string("Auton Info: InitialPoseY"), m_trajectory.getInitialHolonomicPose().Y().to<double>());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("Reset Position PathPlanner"), string("Auton Info: InitialPoseOmega"), m_trajectory.getInitialHolonomicPose().Rotation().Degrees().to<double>());
}

void ResetPositionPathPlanner::Run()
{
}

bool ResetPositionPathPlanner::IsDone()
{
    return true;
}