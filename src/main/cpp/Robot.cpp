// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <Robot.h>

#include <string>

#include <cameraserver/CameraServer.h>

#include <auton/AutonPreviewer.h>
#include <auton/CyclePrimitives.h>
#include <chassis/ChassisFactory.h>
#include <chassis/holonomic/HolonomicDrive.h>
#include <chassis/swerve/SwerveChassis.h>
#include <chassis/mecanum/MecanumChassis.h>
#include <driveteamfeedback/DriverFeedback.h>
#include <DragonVision/LimelightFactory.h>
#include <mechanisms/StateMgrHelper.h>
#include <Robot.h>
#include <robotstate/RobotState.h>
#include <RobotXmlParser.h>
#include <teleopcontrol/TeleopControl.h>
#include <utils/DragonField.h>
#include <utils/FMSData.h>
#include <utils/logging/LoggableItemMgr.h>
#include <utils/logging/Logger.h>
#include <utils/logging/LoggerData.h>
#include <utils/logging/LoggerEnums.h>
#include <utils/WaypointXmlParser.h>

#include <AdjustableItemMgr.h>

/// DEBUGGING
#include <hw/factories/PigeonFactory.h>

using namespace std;

void Robot::RobotInit()
{
    Logger::GetLogger()->PutLoggingSelectionsOnDashboard();
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("RobotInit"), string("arrived"));

    m_controller = nullptr;

    // Read the XML file to build the robot
    auto XmlParser = new RobotXmlParser();
    XmlParser->ParseXML();

    auto waypointParser = WaypointXmlParser::GetInstance();
    waypointParser->ParseWaypoints();

    // Get AdjustableItemMgr instance
    m_tuner = nullptr;
    // m_tuner = AdjustableItemMgr::GetInstance();

    m_robotState = RobotState::GetInstance();
    m_robotState->Init();

    m_chassis = ChassisFactory::GetChassisFactory()->GetSwerveChassis();

    m_holonomic = nullptr;
    if (m_chassis != nullptr)
    {
        m_holonomic = new HolonomicDrive();
    }

    m_cyclePrims = new CyclePrimitives();
    m_previewer = new AutonPreviewer(m_cyclePrims); // TODO:: Move to DriveTeamFeedback
    m_field = DragonField::GetInstance();           // TODO: move to drive team feedback

    //    m_dragonLimeLight = LimelightFactory::GetLimelightFactory()->GetLimelight(); // ToDo:: Move to Dragon Vision

    StateMgrHelper::InitStateMgrs();

    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("RobotInit"), string("end"));
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic()
{
    LoggableItemMgr::GetInstance()->LogData();
    Logger::GetLogger()->PeriodicLog();

    if (m_chassis != nullptr)
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "balance info", "yaw", m_chassis->GetYaw().to<double>());
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "balance info", "pitch", m_chassis->GetPitch().to<double>());
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "balance info", "roll", m_chassis->GetRoll().to<double>());
    }
    // if (m_tuner != nullptr)
    // {
    //     m_tuner->ListenForUpdates();
    // }
    if (m_robotState != nullptr)
    {
        m_robotState->Run();
    }

#define ENABLE_VISION
#ifdef ENABLE_VISION
    // This is an example of how to use RobotVision

    static int mycounter = 0;
    mycounter++;

    // First get the DragonVision singleton
    auto vision = DragonVision::GetDragonVision();

    LoggerIntValue count = {string("counter"), mycounter};
    if (vision != nullptr)
    {
        LoggerStringValue status = {string("HW connection Status"), "Dragon vision is not null"};

        // Set the pipeline. You do not need to call this function repeatedly
        // vision->setPipeline(DragonLimelight::APRIL_TAG);

        // Next get a pointer
        std::shared_ptr<DragonVisionTarget> dvt = vision->getTargetInfo();

        if (dvt == nullptr)
        {
            status = {string("Status"), "No target found or missing limelight"};
            LoggerData data = {LOGGER_LEVEL::PRINT, string("DragonLimelight"), {}, {}, {count}, {status}};
            Logger::GetLogger()->LogData(data);
        }
        else
        {
            status = {string("Status"), "Target found"};
            LoggerDoubleValue vertAngle = {string("VertAngle"), dvt.get()->getVerticalAngleToTarget().to<double>()};
            LoggerDoubleValue horAngle = {string("HorizAngle"), dvt.get()->getHorizontalAngleToTarget().to<double>()};
            LoggerDoubleValue xDistance = {string("x distance RF "), dvt.get()->getXdistanceToTargetRobotFrame().to<double>()};
            LoggerDoubleValue yDistance = {string("y distance RF "), dvt.get()->getYdistanceToTargetRobotFrame().to<double>()};
            LoggerData data = {LOGGER_LEVEL::PRINT, string("DragonLimelight"), {}, {vertAngle, horAngle, yDistance, xDistance}, {count}, {status}};
            Logger::GetLogger()->LogData(data);
        }
    }
    else
    {
        LoggerStringValue status = {string("HW connection Status"), "Dragon vision is  null"};

        LoggerData data = {LOGGER_LEVEL::PRINT, string("DragonLimelight"), {}, {}, {count}, {status}};
        Logger::GetLogger()->LogData(data);
    }
#endif

    // ToDo:: Move to DriveTeamFeedback
    if (m_previewer != nullptr)
    {
        m_previewer->CheckCurrentAuton();
    }
    if (m_field != nullptr && m_chassis != nullptr)
    {
        m_field->UpdateRobotPosition(m_chassis->GetPose()); // ToDo:: Move to DriveTeamFeedback (also don't assume m_field isn't a nullptr)
    }

    // m_tuner->ListenForUpdates();

    auto feedback = DriverFeedback::GetInstance();
    if (feedback != nullptr)
    {
        feedback->UpdateFeedback();
    }

    auto pigeon = PigeonFactory::GetFactory()->GetCenterPigeon();
    if (pigeon == nullptr)
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("DEUBGGING"), string("Pigeon Nullptr?"), "true");
    }
    else
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("Pigeon"), string("Pigeon Yaw"), pigeon->GetYaw());
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("Pigeon"), string("Pigeon Pitch"), pigeon->GetPitch());
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("Pigeon"), string("Pigeon Roll"), pigeon->GetRoll());
    }
}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit()
{
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("AutonomousInit"), string("arrived"));

    StateMgrHelper::SetCheckGamepadInputsForStateTransitions(false);
    if (m_cyclePrims != nullptr)
    {
        m_cyclePrims->Init();
    }
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("AutonomousInit"), string("end"));
}

void Robot::AutonomousPeriodic()
{
    if (m_cyclePrims != nullptr)
    {
        m_cyclePrims->Run();
    }
}

void Robot::TeleopInit()
{
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("TeleopInit"), string("arrived"));

    if (m_controller == nullptr)
    {
        m_controller = TeleopControl::GetInstance();
    }

    StateMgrHelper::SetCheckGamepadInputsForStateTransitions(true);
    if (m_chassis != nullptr && m_controller != nullptr)
    {
        if (m_holonomic != nullptr)
        {
            m_holonomic->Init();
        }

        // Create chassismovement to flush out any drive options from auton
        ChassisMovement resetMoveInfo;
        resetMoveInfo.driveOption = ChassisOptionEnums::DriveStateType::FIELD_DRIVE;
        resetMoveInfo.headingOption = ChassisOptionEnums::HeadingOption::MAINTAIN;

        m_chassis->Drive();
    }
    StateMgrHelper::RunCurrentMechanismStates();

    if (m_chassis != nullptr)
    {
        dynamic_cast<VisionDrive *>(m_chassis->GetSpecifiedDriveState(ChassisOptionEnums::DriveStateType::VISION_DRIVE))->setInAutonMode(false);
    }

    // now in teleop, clear field of trajectories
    if (m_field != nullptr)
    {
        m_field->ResetField(); // ToDo:  Move to DriveTeamFeedback
    }

    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("TeleopInit"), string("end"));
}

void Robot::TeleopPeriodic()
{
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("TeleopPeriodic"), string("arrived"));
    if (m_chassis != nullptr && m_controller != nullptr)
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("HolonomicRun"), string("arrived"));
        if (m_holonomic != nullptr)
        {
            m_holonomic->Run();
        }
    }
    StateMgrHelper::RunCurrentMechanismStates();

    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("TeleopPeriodic"), string("end"));
}

void Robot::DisabledInit()
{

    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("DisabledInit"), string("arrived"));
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("DisabledInit"), string("end"));
}

void Robot::DisabledPeriodic()
{
}

void Robot::TestInit()
{
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("TestInit"), string("arrived"));
}

void Robot::TestPeriodic()
{
}

#ifndef RUNNING_FRC_TESTS
int main()
{
    return frc::StartRobot<Robot>();
}
#endif
