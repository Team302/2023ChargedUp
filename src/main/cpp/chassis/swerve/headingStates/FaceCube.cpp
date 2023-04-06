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

// Team302 Includes
#include <chassis/swerve/headingStates/FaceCube.h>
#include <DragonVision/LimelightFactory.h>

FaceCube::FaceCube() : ISwerveDriveOrientation(ChassisOptionEnums::HeadingOption::FACE_APRIL_TAG),
                       m_pipelineMode(DragonLimelight::CUBE),
                       m_vision(DragonVision::GetDragonVision())
{
}

void FaceCube::UpdateChassisSpeeds(ChassisMovement &chassisMovement)
{
    units::angular_velocity::radians_per_second_t omega = units::angular_velocity::radians_per_second_t(0.0);

    units::angle::radian_t angleError = units::angle::radian_t(0.0);

    // get targetdata from the vision system
    auto targetData = m_vision->getTargetInfo();

    if ((targetData != nullptr) && (m_pipelineMode == targetData->getTargetType()))
    {
        if (!AtTargetAngle(targetData, &angleError))
        {
            omega = units::angle::radian_t(angleError * m_visionKP_Angle) / 1_s;
            omega = limitAngularVelocityToBetweenMinAndMax(omega);

            chassisMovement.chassisSpeeds.omega = omega;
        }
    }
}

bool FaceCube::AtTargetAngle(std::shared_ptr<DragonVisionTarget> targetData, units::angle::radian_t *error)
{
    if (targetData != nullptr)
    {
        units::length::inch_t yError = targetData->getYdistanceToTargetRobotFrame();
        units::length::inch_t xError = targetData->getXdistanceToTargetRobotFrame();

        if (std::abs(xError.to<double>()) > 0.01)
        {
            *error = units::angle::radian_t(std::atan2(yError.to<double>(), xError.to<double>()));

            if (std::abs((*error).to<double>()) < m_AngularTolerance_rad)
            {
                return true;
            }
        }
    }
    return false;
}

units::angular_velocity::radians_per_second_t FaceCube::limitAngularVelocityToBetweenMinAndMax(units::angular_velocity::radians_per_second_t angularVelocity)
{
    double sign = angularVelocity.to<double>() < 0 ? -1 : 1;

    if (std::abs(angularVelocity.to<double>()) < m_minimumOmega_radps)
        angularVelocity = units::angular_velocity::radians_per_second_t(m_minimumOmega_radps * sign);

    if (std::abs(angularVelocity.to<double>()) > m_maximumOmega_radps)
        angularVelocity = units::angular_velocity::radians_per_second_t(m_maximumOmega_radps * sign);

    return angularVelocity;
}