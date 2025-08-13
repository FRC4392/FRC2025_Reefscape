// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmPosition;
import frc.robot.subsystems.arm.ArmCommands;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.arm.ArmIOTalonFX;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.gripper.GripperCommands;
import frc.robot.subsystems.gripper.GripperIO;
import frc.robot.subsystems.gripper.GripperIOSIm;
import frc.robot.subsystems.gripper.GripperIOTalon;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.swerve.GyroIO;
import frc.robot.subsystems.swerve.GyroIOPigeon2;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveCommands;
import frc.robot.subsystems.swerve.SwerveCommands.ReefSide;
import frc.robot.subsystems.swerve.SwerveModuleIO;
import frc.robot.subsystems.swerve.SwerveModuleIODeceivers;
import frc.robot.subsystems.swerve.SwerveModuleIOSim;
import frc.robot.subsystems.swerve.SwerveState;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

/** Class to contain all the parts/subsystems of the robot */
public class RobotContainer {

  private final DeceiverRobotState robotState;

  // Subsystems
  public final Swerve swerve;
  public final Vision vision;
  public final Arm arm;
  public final Gripper gripper;
  public final Leds leds;

  // Controllers
  private final CommandXboxController driveController = new CommandXboxController(0);
  private final CommandXboxController operateController = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  LoggedNetworkBoolean resetRobotStateBoolean = new LoggedNetworkBoolean("resetRobotState");

  // Roobot Alerts
  Alert driverControllerAlert = new Alert("Driver Controller Disconnected 🎮", AlertType.kError);
  Alert operatorControllerAlert =
      new Alert("Operator Controller Disconnected 🎮", AlertType.kError);
  Alert autoAlert = new Alert("Select an autonomous mode! 😟", AlertType.kError);

  // Permanant autos
  private Command noAuto = Commands.none();

  /**
   * Constructor
   *
   * @param state RobotState object to track the state of the robot
   */
  public RobotContainer(DeceiverRobotState state) {

    robotState = state;

    resetRobotStateBoolean.setDefault(false);

    leds = new Leds();

    switch (RobotConstants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        swerve =
            new Swerve(
                new GyroIOPigeon2(),
                new SwerveModuleIODeceivers(0),
                new SwerveModuleIODeceivers(1),
                new SwerveModuleIODeceivers(2),
                new SwerveModuleIODeceivers(3),
                robotState);
        vision =
            new Vision(
                swerve::addVisionMeasurement,
                new VisionIOLimelight(camera0Name, swerve::getRotation),
                new VisionIOLimelight(camera1Name, swerve::getRotation),
                new VisionIOLimelight(camera2Name, swerve::getRotation));

        arm = new Arm(new ArmIOTalonFX());
        gripper = new Gripper(new GripperIOTalon());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        swerve =
            new Swerve(
                new GyroIO() {},
                new SwerveModuleIOSim(),
                new SwerveModuleIOSim(),
                new SwerveModuleIOSim(),
                new SwerveModuleIOSim(),
                robotState);
        vision =
            new Vision(
                swerve::addVisionMeasurement,
                new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, swerve::getPose),
                new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, swerve::getPose),
                new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, swerve::getPose));
        arm = new Arm(new ArmIOSim());
        gripper = new Gripper(new GripperIOSIm());
        break;

      default:
        // Replayed robot, disable IO implementations
        swerve =
            new Swerve(
                new GyroIO() {},
                new SwerveModuleIO() {},
                new SwerveModuleIO() {},
                new SwerveModuleIO() {},
                new SwerveModuleIO() {},
                robotState);
        vision = new Vision(swerve::addVisionMeasurement, new VisionIO() {});
        arm = new Arm(new ArmIO() {});
        gripper = new Gripper(new GripperIO() {});
        break;
    }

    // Set up named commands
    NamedCommands.registerCommand(
        "MoveToUpTravel",
        ArmCommands.setArmPosition(arm, Rotation2d.fromDegrees(90), 0, new Rotation2d()));
    NamedCommands.registerCommand(
        "MoveToL4",
        ArmCommands.setArmPosition(arm, Arm.ArmPosition.L4)
            .until(() -> arm.getArmInPosition())
            .withTimeout(5));
    NamedCommands.registerCommand(
        "AutoAlignRightWithTimeout",
        SwerveCommands.autoAlignCommand3D(swerve, vision, ReefSide.right)
            .until(() -> swerve.getSwerveState() == SwerveState.autoAlignDone)
            .withTimeout(1.0));
    NamedCommands.registerCommand(
        "AutoAlignLeftWithTimeout",
        SwerveCommands.autoAlignCommand3D(swerve, vision, ReefSide.left)
            .until(() -> swerve.getSwerveState() == SwerveState.autoAlignDone)
            .withTimeout(1.0));
    NamedCommands.registerCommand(
        "ejectCoral", GripperCommands.coralAutoOuttake(gripper).withTimeout(2));
    NamedCommands.registerCommand(
        "MoveToPickup", ArmCommands.setArmPosition(arm, ArmPosition.PROCESSOR));
    NamedCommands.registerCommand("IntakeCoral", GripperCommands.coralIntakeAuto(gripper));

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", SwerveCommands.wheelRadiusCharacterization(swerve));
    autoChooser.addOption(
        "Drive Simple FF Characterization", SwerveCommands.feedforwardCharacterization(swerve));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        swerve.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        swerve.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", swerve.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", swerve.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Set up auto routines
    autoChooser.addOption("Secret Auto", new PathPlannerAuto("Secret Auto"));

    autoChooser.addOption("Left 1", new PathPlannerAuto("Preset_Left_Test_1"));
    autoChooser.addOption("Left 2", new PathPlannerAuto("Preset_Left_Test_2"));
    autoChooser.addOption("Left 3", new PathPlannerAuto("Preset_Left_Test_3"));
    autoChooser.addOption("Left 4", new PathPlannerAuto("Preset_Left_Test_4"));

    autoChooser.addOption("Right 1", new PathPlannerAuto("Preset_Right_Test_1"));
    autoChooser.addOption("Right 2", new PathPlannerAuto("Preset_Right_Test_2"));
    autoChooser.addOption("Right 3", new PathPlannerAuto("Preset_Right_Test_3"));
    autoChooser.addOption("Right 4", new PathPlannerAuto("Preset_Right_Test_4"));
    autoChooser.addOption("Test Auto", new PathPlannerAuto("New Auto"));
    autoChooser.addDefaultOption("None", noAuto);

    // Set up LED suppliers
    leds.setGripperSupplier(gripper::getState);
    leds.setSwerveSupplier(swerve::getSwerveState);

    configureBindings();
  }

  /** Used to set up bidings for triggers, joystick buttons, default commands, etc */
  private void configureBindings() {
    Trigger robotWasEnabled = new Trigger(robotState::getWasEnabled);

    robotWasEnabled
        .onTrue(
            Commands.runOnce(
                    () -> {
                      // When robot enables for the first time, use MEGATAG 2 for loacalization, use
                      // MEGATAG 1 before enabled to localize robot and rotation
                      vision.setIfPoseTypeAllowed(PoseObservationType.MEGATAG_1, false);
                      vision.setIfPoseTypeAllowed(PoseObservationType.MEGATAG_2, true);
                    })
                .ignoringDisable(true))
        .onFalse(
            Commands.runOnce(
                    () -> {
                      // When robot enables for the first time, use MEGATAG 2 for loacalization, use
                      // MEGATAG 1 before enabled to localize robot and rotation
                      vision.setIfPoseTypeAllowed(PoseObservationType.MEGATAG_1, true);
                      vision.setIfPoseTypeAllowed(PoseObservationType.MEGATAG_2, false);
                    })
                .ignoringDisable(true));

    // Default command, normal field-relative drive
    swerve.setDefaultCommand(
        SwerveCommands.joystickDrive(
            swerve,
            () -> -driveController.getLeftY(),
            () -> -driveController.getLeftX(),
            () -> driveController.getLeftTriggerAxis() - driveController.getRightTriggerAxis(),
            () -> driveController.getHID().getAButton()));

    // Smart Intake
    driveController.leftStick().whileTrue(GripperCommands.algaeIntake(gripper));

    // Smart Outttake
    driveController.rightStick().whileTrue(GripperCommands.algaeOutake(gripper));

    driveController.b().whileTrue(Commands.run(() -> gripper.setClimberVoltage(12), gripper));

    // Auto align to left branch from driver view
    driveController
        .leftBumper()
        .whileTrue(SwerveCommands.autoAlignCommand3D(swerve, vision, ReefSide.left));

    // Auto align to right branch from driver view
    driveController
        .rightBumper()
        .whileTrue(SwerveCommands.autoAlignCommand3D(swerve, vision, ReefSide.right));

    // Reset gyro rotation, maintin position
    driveController.start().onTrue(Commands.runOnce(() -> swerve.resetGyro()));

    // Put drive in X position
    driveController.x().whileTrue(SwerveCommands.stopWithX(swerve));

    Trigger testTrigger =
        new Trigger(
            () -> {
              return operateController.getLeftTriggerAxis()
                      + operateController.getRightTriggerAxis()
                  > 0;
            });
    testTrigger.onTrue(
        Commands.sequence(
            ArmCommands.setArmPosition(
                    arm, Rotation2d.fromDegrees(90), 0, Rotation2d.fromDegrees(50))
                .until(() -> arm.getArmInPosition()),
            ArmCommands.joystickArmControl(
                    arm,
                    () -> {
                      return operateController.getLeftY();
                    },
                    () -> {
                      return 0.0;
                    },
                    () -> {
                      return 0.0;
                    })
                .until(testTrigger.negate())));
  }

  // This need to be replaced
  public void OperatorLoop() {
    if (operateController.getLeftTriggerAxis() + operateController.getRightTriggerAxis() == 0) {
      if (operateController.getHID().getAButton()) {
        arm.setArmPostion(ArmPosition.L2);
      } else if (operateController.getHID().getXButton()) {
        arm.setArmPostion(ArmPosition.L3);
      } else if (operateController.getHID().getYButton()
          && !driveController.getHID().getRightStickButton()) {
        arm.setArmPostion(ArmPosition.L4);
      } else if (operateController.getHID().getBButton()) {
        arm.setArmPostion(ArmPosition.L1);
      } else if (operateController.getHID().getStartButton()) {
        arm.setArmPostion(ArmPosition.HOME);
      } else if (operateController.getHID().getLeftBumperButton()
          || operateController.getHID().getRightBumperButton()) {
        arm.setArmPostion(ArmPosition.INTAKE);
      } else if (operateController.getHID().getPOV() == 180) {
        arm.setArmPostion(ArmPosition.ALGAE1);
      } else if (operateController.getHID().getPOV() == 0) {
        arm.setArmPostion(ArmPosition.ALGAE2);
      } else if (operateController.getHID().getPOV() == 90) {
        arm.setArmPostion(ArmPosition.BARGE);
      } else if (operateController.getHID().getPOV() == 270) {
        arm.setArmPostion(ArmPosition.PROCESSOR);
      }
    }
  }

  /**
   * Update robot alerts.
   *
   * <p>Should be called periodically
   */
  private void updateAlerts() {
    // Check if joysticks are unplugged
    driverControllerAlert.set(!driveController.isConnected());
    operatorControllerAlert.set(!operateController.isConnected());

    // Check that an auto has been selected
    autoAlert.set(!robotState.getWasAuto() && autoChooser.get() == noAuto);
  }

  /**
   * Update dashboard data.
   *
   * <p>Should be called periodically
   */
  private void updateDashboard() {
    // Send match time to dashboard
    SmartDashboard.putNumber("MatchTime", DriverStation.getMatchTime());

    if (resetRobotStateBoolean.get() && robotState.isDisabled() && !DriverStation.isFMSAttached()) {
      robotState.resetState();
      resetRobotStateBoolean.set(false);
    }
  }

  /**
   * Place code here that should be run every loop cycle
   *
   * <p>Should be called robotPeriodic
   */
  public void periodic() {
    updateDashboard();
    updateAlerts();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
