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
//======================================================\==============================================================================================

#include <RobotState.h>

#include <string>

#include <cameraserver/CameraServer.h>
#include <auton/CyclePrimitives.h>
#include <chassis/ChassisFactory.h>
#include <chassis/IChassis.h>
#include <mechanisms/StateMgrHelper.h>
#include <teleopcontrol/TeleopControl.h>
#include <utils/FMSData.h>
#include <utils/DragonField.h>
#include <auton/AutonPreviewer.h>
#include <mechanisms/DriverFeedback/DriverFeedback.h>

RobotState* RobotState::m_instance = nullptr;

RobotState* RobotState::GetInstance(){
    if (RobotState::m_instance == nullptr){
        RobotState::m_instance = new RobotState();
    }
    return RobotState::m_instance;
}


void RobotState::Init(){
    m_fmsData = FMSData::GetInstance();
    m_field = DragonField::GetInstance();

    auto factory = ChassisFactory::GetChassisFactory();
    m_chassis = factory->GetIChassis();
    
    StateMgrHelper::InitStateMgrs();

    m_driveTeamFeedback = DriverFeedback::GetInstance();
        
}

void RobotState::Run(){
    if(frc::DriverStation::IsEnabled()){
        m_driveTeamFeedback->TeleopEnabled(frc::DriverStation::IsTeleopEnabled());
        m_driveTeamFeedback->AutonomousEnabled(frc::DriverStation::IsAutonomousEnabled());

        if (m_chassis != nullptr)
        {
            m_chassis->UpdateOdometry();
            m_field->UpdateRobotPosition(m_chassis->GetPose());
        }
        m_driveTeamFeedback->AlignedWithConeNode(true);
        m_driveTeamFeedback->UpdateFeedback();


    }
    if(frc::DriverStation::IsTeleopEnabled()){

    }

    if(frc::DriverStation::IsAutonomousEnabled()){

    }

    if(frc::DriverStation::IsDisabled()){
        m_driveTeamFeedback->m_LEDStates->LEDsOff();
    }
}
