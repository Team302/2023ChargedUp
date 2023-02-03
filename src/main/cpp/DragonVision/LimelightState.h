
//====================================================================================================================================================
// Copyright 2022 Lake Orion Robotics FIRST Team 302 
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


#pragma once

#include <hw\DragonLimelight.h>
#include <networktables/NetworkTable.h>

#include <utils/Logger.h>

#include <string>
#include <units/angle.h>
#include <units/length.h>
#include <units/time.h>
#include <vector>
class LimelightState
{
   public:
        

        ///-----------------------------------------------------------------------------------
        /// Method:         DragonLimelight (constructor)
        /// Description:    Create the object
        ///-----------------------------------------------------------------------------------
        LimelightState
        (
            DragonLimelight* dragonLimelight,            /// <I> - height of second target
            int index
        );
        ~LimelightState();

        ///-----------------------------------------------------------------------------------
        /// Method:         ~DragonLimelight (destructor)
        /// Description:    Delete the object
        ///----------------------------------------------------------------------------------
        
        // Retroreflective tape Getters
        bool HasTarget() const;
        units::angle::degree_t GetTargetHorizontalOffset() const;
        units::angle::degree_t GetTargetVerticalOffset() const;
        double GetTargetArea() const;
        units::angle::degree_t GetTargetSkew() const;
        units::time::microsecond_t GetPipelineLatency() const;
        units::length::inch_t EstimateTargetDistance() const;
        int GetPipelineIndex() const {return m_index;};

    protected:
        units::angle::degree_t GetTx() const;
        units::angle::degree_t GetTy() const;
        
        DragonLimelight*        m_limelight;
        std::shared_ptr<nt::NetworkTable> m_networktable;
        int m_index;
        double PI = 3.14159265;
};
