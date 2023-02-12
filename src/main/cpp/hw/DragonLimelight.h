
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

#pragma once

// C++ Includes
#include <string>
#include <vector>

// FRC includes
#include <networktables/NetworkTable.h>
#include <units/angle.h>
#include <units/length.h>
#include <units/time.h>

// Team 302 includes

// Third Party Includes

class DragonLimelight
{
public:
    // set the values according to the pipeline indices in the limelights
    // this assumes that each of the limelights has the same pipelines and they are in the same order
    enum PIPELINE_MODE
    {
        UNKNOWN = -1,
        RETRO_REFLECTIVE = 0,
        APRIL_TAG = 1,
        CONE = 2,
        CUBE = 3,
        MAX_PIPELINE_MODE
    };

    enum LED_MODE
    {
        LED_DEFAULT,
        LED_OFF,
        LED_BLINK,
        LED_ON
    };

    enum CAM_MODE
    {
        CAM_VISION,
        CAM_DRIVER
    };

    enum STREAM_MODE
    {
        STREAM_DEFAULT,         // side by side if two cams
        STREAM_MAIN_AND_SECOND, // Second Cam bottom right of Main Cam
        STREAM_SECOND_AND_MAIN  // Main Cam bottom right of Second Cam
    };

    enum SNAPSHOT_MODE
    {
        SNAP_OFF,
        SNAP_ON
    };

    ///-----------------------------------------------------------------------------------
    /// Method:         DragonLimelight (constructor)
    /// Description:    Create the object
    ///-----------------------------------------------------------------------------------
    DragonLimelight() = delete;
    DragonLimelight(
        std::string tableName,                          /// <I> - network table name
        units::length::inch_t mountingHeight,           /// <I> - mounting height of the limelight
        units::length::inch_t mountingHorizontalOffset, /// <I> - mounting horizontal offset from the middle of the robot
        units::length::inch_t LLForward,                /// <I> mounting offset forward/back
        units::angle::degree_t LLPitch,                 /// <I> - Pitch of limelight
        units::angle::degree_t LLYaw,                   /// <I> - Yaw of limelight
        units::angle::degree_t LLRoll,                  /// <I> - Roll of limelight
        units::length::inch_t targetHeight,             /// <I> - height the target
        units::length::inch_t targetHeight2,            /// <I> - height of second target
        LED_MODE ledMode,
        CAM_MODE camMode,
        STREAM_MODE streamMode,
        SNAPSHOT_MODE snapMode);

    ///-----------------------------------------------------------------------------------
    /// Method:         ~DragonLimelight (destructor)
    /// Description:    Delete the object
    ///-----------------------------------------------------------------------------------
    ~DragonLimelight() = default;

    bool HasTarget() const;
    units::angle::degree_t GetTargetHorizontalOffset() const;
    units::angle::degree_t GetTargetVerticalOffset() const;
    double GetTargetArea() const;
    units::angle::degree_t GetTargetSkew() const;
    units::time::microsecond_t GetPipelineLatency() const;
    units::length::inch_t EstimateTargetDistance() const;
    std::vector<double> Get3DSolve() const;
    PIPELINE_MODE getPipeline() const;

    // Setters
    void SetTargetHeight(units::length::inch_t targetHeight);
    void SetLEDMode(DragonLimelight::LED_MODE mode);
    void SetCamMode(DragonLimelight::CAM_MODE mode);
    void SetPipeline(int pipeline);
    void SetStreamMode(DragonLimelight::STREAM_MODE mode);
    void ToggleSnapshot(DragonLimelight::SNAPSHOT_MODE toggle);
    void SetCrosshairPos(double crosshairPosX, double crosshairPosY);
    void SetSecondaryCrosshairPos(double crosshairPosX, double crosshairPosY);
    void SetPitch(units::angle::degree_t newPitch);
    void SetYaw(units::angle::degree_t newYaw);
    void SetRoll(units::angle::degree_t newRoll);

    void PrintValues(); // Prints out all values to ensure everything is working and connected

    units::angle::degree_t GetLimelightPitch() const { return m_pitch; }
    units::angle::degree_t GetLimelightYaw() const { return m_yaw; }
    units::angle::degree_t GetLimelightRoll() const { return m_roll; }
    units::length::inch_t GetMountingHeight() const { return m_mountHeight; }
    units::length::inch_t GetTargetHeight() const { return m_targetHeight; }

protected:
    units::angle::degree_t GetTx() const;
    units::angle::degree_t GetTy() const;

    void SetLimelightPosition(units::length::inch_t mountHeight,
                              units::length::inch_t mountHorizontalOffset,
                              units::length::inch_t mountForwardOffset,
                              units::angle::degree_t pitch,
                              units::angle::degree_t yaw,
                              units::angle::degree_t roll);

    std::shared_ptr<nt::NetworkTable> m_networktable;
    units::length::inch_t m_mountHeight;
    units::length::inch_t m_mountingHorizontalOffset;
    units::length::inch_t m_mountingForwardOffset;
    units::angle::degree_t m_yaw;
    units::angle::degree_t m_pitch;
    units::angle::degree_t m_roll;
    units::length::inch_t m_targetHeight;
    units::length::inch_t m_targetHeight2;

    double PI = 3.14159265;
};
