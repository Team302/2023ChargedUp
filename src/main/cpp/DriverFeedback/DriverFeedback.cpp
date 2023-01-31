
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

#include <DriverFeedback/DriverFeedback.h>
#include <DriverFeedback/DriverFeedbackStruct.h>
#include <utils/Logger.h>
#include <string>

 void DriverFeedback::UpdateFeedback(){

    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT,std::string("Driverfeedback "),std::string("AlignedWithConeNode "),DriveteamFeedbackOptions.AlignedWithConeNode);
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT,std::string("Driverfeedback "),std::string("AlignedWithCubeNode "),DriveteamFeedbackOptions.AlignedWithCubeNode);
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT,std::string("Driverfeedback "),std::string("GamePieceInGrabber "),DriveteamFeedbackOptions.GamePieceInGrabber);
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT,std::string("Driverfeedback "),std::string("WantCone "),DriveteamFeedbackOptions.WantCone);
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT,std::string("Driverfeedback "),std::string("WantCube "),DriveteamFeedbackOptions.WantCube);
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT,std::string("Driverfeedback "),std::string("GamePieceReadyToPickUp"),DriveteamFeedbackOptions.GamePieceReadyToPickUp);


    if(DriveteamFeedbackOptions.AlignedWithConeNode){

        if(currentState!=DriverFeedbackStates::ALIGNED_WITH_CONE_NODE){
            m_LEDStates->ResetVariables();
        }
        m_LEDStates->AlternatingBlinkingPattern(LED::YELLOW);
        currentState = DriverFeedbackStates::ALIGNED_WITH_CONE_NODE;

    }else if(DriveteamFeedbackOptions.AlignedWithCubeNode){
        
        if(currentState!=DriverFeedbackStates::ALIGNED_WITH_CUBE_NODE){
            m_LEDStates->ResetVariables();
        }
        m_LEDStates->AlternatingBlinkingPattern(LED::PURPLE);
        currentState = DriverFeedbackStates::ALIGNED_WITH_CUBE_NODE;

    }else if(DriveteamFeedbackOptions.GamePieceInGrabber){
         
        if(currentState!=DriverFeedbackStates::GAME_PIECE_IN_GRABBER){
            m_LEDStates->ResetVariables();
        }
        m_LEDStates->AlternatingBlinkingPattern(LED::YELLOW, LED::PURPLE);
        currentState = DriverFeedbackStates::ALIGNED_WITH_CUBE_NODE;

    }else if(DriveteamFeedbackOptions.WantCube){
        currentState = DriverFeedbackStates::WANT_CUBE;

    }else if(DriveteamFeedbackOptions.WantCone){
        currentState = DriverFeedbackStates::WANT_CONE;

    }else if(DriveteamFeedbackOptions.GamePieceReadyToPickUp){
        currentState = DriverFeedbackStates::GAME_PIECE_READY_TO_PICK_UP;
    
    }else{
        currentState = DriverFeedbackStates::NONE;
    
    }
}
