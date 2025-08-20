// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.commands.FollowPathCommand;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.urcl.URCL;

/** Main robot class */
public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;
  private final RobotContainer robotContainer;
  private final DeceiverRobotState robotState;

  /** Constructor */
  public Robot() {

    // Create new robot state
    robotState = new DeceiverRobotState();

    // Record metadata about the git version for future reference
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    // Set up logging based on robot mode
    switch (RobotConstants.currentMode) {
      case COMMISIONING:
        // Don't log to network tables during real match
        // Probably always false at this point, but doesn't hurt to check
        if (!DriverStation.isFMSAttached()) {
          Logger.addDataReceiver(new NT4Publisher());
        }
        // Fall through
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // Start CTRE Logger
    SignalLogger.start();
    // Start Rev Logger with AdvantageKit
    Logger.registerURCL(URCL.startExternal());

    // Start AdvantageKit Logger
    Logger.start();

    // Remove controller disconnected message, we handle this on our own
    DriverStation.silenceJoystickConnectionWarning(true);

    // Lower brownout voltage
    RobotController.setBrownoutVoltage(6.0);

    // Create robot container
    robotContainer = new RobotContainer(robotState);
  }

  // Runs every loop cycle
  @Override
  public void robotPeriodic() {
    // Run command scheduler
    CommandScheduler.getInstance().run();
  }

  // Runs when robot is first started
  @Override
  public void robotInit() {
    // Warm up PathPlanner to reduce delay on auto init
    FollowPathCommand.warmupCommand().schedule();
  }

  // Runs when entering disabled mode
  @Override
  public void disabledInit() {
    robotState.setDisabled(true);
  }

  // Runs every loop cycle while disabled
  @Override
  public void disabledPeriodic() {}

  // Runs when leaving disabled
  @Override
  public void disabledExit() {
    robotState.setDisabled(false);
  }

  // Runs when entering autonomous
  @Override
  public void autonomousInit() {
    robotState.setAuto(true);
    m_autonomousCommand = robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  // Runs every loop cycle while enabled in autonmous mode
  @Override
  public void autonomousPeriodic() {}

  // Runs when leaving autonomous mode
  @Override
  public void autonomousExit() {
    robotState.setAuto(false);
  }

  // Runs when entering autonomous mode
  @Override
  public void teleopInit() {
    robotState.setTeleop(true);
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  // Runs every loop cycle while enabled in teleoperated mode
  @Override
  public void teleopPeriodic() {
    robotContainer.OperatorLoop();
  }

  // Runs when leaving autonomous mode
  @Override
  public void teleopExit() {
    robotState.setTeleop(false);
  }

  // Runs when entering test mode
  @Override
  public void testInit() {
    robotState.setTest(true);
    CommandScheduler.getInstance().cancelAll();
  }

  // Runs every loop cycle while enabled in test mode
  @Override
  public void testPeriodic() {}

  // Runs when leaving test mode
  @Override
  public void testExit() {
    robotState.setTest(false);
  }
}
