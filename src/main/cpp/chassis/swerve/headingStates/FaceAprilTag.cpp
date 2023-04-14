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
#include <chassis/swerve/headingStates/FaceAprilTag.h>
#include <DragonVision/LimelightFactory.h>
#include <hw/factories/PigeonFactory.h>

/// debugging
#include <utils/logging/Logger.h>

FaceAprilTag::FaceAprilTag() : ISwerveDriveOrientation(ChassisOptionEnums::HeadingOption::FACE_APRIL_TAG),
                               m_pipelineMode(DragonLimelight::APRIL_TAG),
                               m_vision(DragonVision::GetDragonVision())
{
}

void FaceAprilTag::UpdateChassisSpeeds(ChassisMovement &chassisMovement)
{
    units::angular_velocity::radians_per_second_t omega = units::angular_velocity::radians_per_second_t(0.0);

    units::angle::radian_t angleError = units::angle::radian_t(0.0);

    // get targetdata from the vision system
    auto targetData = m_vision->getTargetInfo();

    if ((targetData != nullptr) && (m_vision->getPipeline(DragonVision::LIMELIGHT_POSITION::FRONT) == targetData->getTargetType()))
    {
        if (!AtTargetAngle(targetData, &angleError))
        {
            omega = units::angle::radian_t(angleError * m_visionKP_Angle) / 1_s;
            omega = limitAngularVelocityToBetweenMinAndMax(omega);

            chassisMovement.chassisSpeeds.omega = omega;
        }
    }
}

bool FaceAprilTag::AtTargetAngle(std::shared_ptr<DragonVisionTarget> targetData, units::angle::radian_t *error)
{
    if (targetData != nullptr)
    {
        units::length::inch_t yError = targetData->getYdistanceToTargetRobotFrame();
        units::length::inch_t xError = targetData->getXdistanceToTargetRobotFrame();

        if (std::abs(xError.to<double>()) > 0.01)
        {
            /// Math
            // First get the pigeon angle to later get field, this is considered
            // Next, get the angle to the tag, this is considered alpha
            // Calculate alpha by taking the arc/inverse tangent of our yError and xError (robot oriented) to the tag
            // Calculate field oriented error by taking the cosine and sine of alpha + theta
            // From there, we can get the angle to the back of the node (considered beta)
            // This is calculated by taking arc/inverse tangent of our field oriented yError, divided by our field oriented xError
            // and the offset to the back of the cube node

            auto pigeon = PigeonFactory::GetFactory()->GetCenterPigeon();
            units::angle::degree_t robotYaw = units::angle::degree_t(pigeon->GetYaw());

            auto angleToTag = units::angle::radian_t(std::atan2(yError.to<double>(), xError.to<double>()));

            units::length::inch_t xErrorFieldOriented = xError * units::math::cos(angleToTag + robotYaw);
            units::length::inch_t yErrorFieldOriented = xError * units::math::sin(angleToTag + robotYaw);

            auto angleToBackOfNode = units::math::atan2(yErrorFieldOriented, xErrorFieldOriented + m_cubeNodeLength);

            *error = -1.0 * (robotYaw - angleToBackOfNode); // negate to turn correctly

            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "ANickDebugging", "RobotYaw (Deg)", robotYaw.to<double>());
            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "ANickDebugging", "AngleToTag (Rad)", angleToTag.to<double>());
            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "ANickDebugging", "AngleToTag (Deg)", units::angle::degree_t(angleToTag).to<double>());
            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "ANickDebugging", "YError (Inches)", yError.to<double>());
            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "ANickDebugging", "XError (Inches)", xError.to<double>());
            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "ANickDebugging", "XErrorFieldOriented (Inches)", xErrorFieldOriented.to<double>());
            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "ANickDebugging", "YErrorFieldOriented (Inches)", yErrorFieldOriented.to<double>());
            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "ANickDebugging", "Beta (Rad)", angleToBackOfNode.to<double>());
            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "ANickDebugging", "Beta (Deg)", units::angle::degree_t(angleToBackOfNode).to<double>());
            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "ANickDebugging", "Angle Error (Rad)", (*error).to<double>());
            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "ANickDebugging", "Angle Error (Deg)", units::angle::degree_t(*error).to<double>());

            if (std::abs((*error).to<double>()) < m_AngularTolerance_rad)
            {
                return true;
            }
        }
    }
    return false;
}

units::angular_velocity::radians_per_second_t FaceAprilTag::limitAngularVelocityToBetweenMinAndMax(units::angular_velocity::radians_per_second_t angularVelocity)
{
    double sign = angularVelocity.to<double>() < 0 ? -1 : 1;

    if (std::abs(angularVelocity.to<double>()) < m_minimumOmega_radps)
        angularVelocity = units::angular_velocity::radians_per_second_t(m_minimumOmega_radps * sign);

    if (std::abs(angularVelocity.to<double>()) > m_maximumOmega_radps)
        angularVelocity = units::angular_velocity::radians_per_second_t(m_maximumOmega_radps * sign);

    return angularVelocity;
}