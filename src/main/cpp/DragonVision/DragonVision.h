
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
#include <string>
#include <State.h>
#include <hw/DragonLimelight.h>
#include <hw\factories\LimelightFactory.h>
class DragonLimeLight;
class DragonVision : public State
{
    public:
        static DragonVision* GetDragonVision();
        
    
    
    

    void Init() override;
    void Run() override;
    void Exit() override;
    bool AtTarget() const override;

    bool AlignedWithCubeNode() const;
    bool AlignedWithConeNode() const;
    bool AlignedWithSubstation() const;
    bool AlignedWithCubeGamePiece() const;
    bool AlignedWithConeGamePiece() const;

    int DistanceFromCubeNode() const;
    int DistanceFromConeNode() const;
    int DistanceFromSubstation() const;
    int AngleFromCubeNode() const;
    int AngleFromConeNode() const;
    int AngleFromSubstation() const;
    int DistanceFromCubeGamePiece() const;
    int DistanceFromConeGamePiece() const;
    int AngleFromCubeGamePiece() const;
    int AngleFromConeGamePiece() const;
    int GetRobotPosition() const;
   
private:
    
    DragonVision();
    ~DragonVision() = default;

    static DragonVision*	m_dragonvision;
    DragonVision*    m_frontdragonlimelight;
 
    
};


