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

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;
  private final RobotContainer robotContainer;
  private final RobotState robotState;

  public Robot() {

    robotState = new RobotState();

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

    switch (RobotConstants.currentMode) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        if (!DriverStation.isFMSAttached()) {
          // Don't log to network tables during real match
          Logger.addDataReceiver(new NT4Publisher());
        }
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

    SignalLogger.start(); // Start CTRE Logger
    Logger.registerURCL(URCL.startExternal()); // Start Rev Logger with AdvantageKit

    Logger.start(); // Start AdvantageKit Logger

    // Remove controller disconnected message
    DriverStation.silenceJoystickConnectionWarning(true);

    // Lower brownout voltage
    RobotController.setBrownoutVoltage(6.0);

    robotContainer = new RobotContainer(robotState);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    robotContainer.updateAlerts();
    robotContainer.updateDashboard();
  }

  @Override
  public void robotInit() {
    // Warm up PathPlanner to reduce delay on auto init
    FollowPathCommand.warmupCommand().schedule();
  }

  @Override
  public void disabledInit() {
    robotState.setDisabled(true);
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {
    robotState.setDisabled(false);
  }

  @Override
  public void autonomousInit() {
    robotState.setAuto(true);
    m_autonomousCommand = robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {
    robotState.setAuto(false);
  }

  @Override
  public void teleopInit() {
    robotState.setTeleop(true);
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    robotContainer.OperatorLoop();
  }

  @Override
  public void teleopExit() {
    robotState.setTeleop(false);
  }

  @Override
  public void testInit() {
    robotState.setTest(true);
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {
    robotState.setTest(false);
  }
}
