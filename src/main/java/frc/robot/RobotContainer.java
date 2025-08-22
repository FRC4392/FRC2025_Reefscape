// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.events.EventTrigger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotConstants.Mode;
import frc.robot.operatorinterface.OperatorInterface;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmPosition;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.arm.ArmIOTalonFX;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.gripper.GripperIO;
import frc.robot.subsystems.gripper.GripperIOSIm;
import frc.robot.subsystems.gripper.GripperIOSpark;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.swerve.GyroIO;
import frc.robot.subsystems.swerve.GyroIOPigeon2;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveConstants.ReefSide;
import frc.robot.subsystems.swerve.SwerveModuleIO;
import frc.robot.subsystems.swerve.SwerveModuleIODeceivers;
import frc.robot.subsystems.swerve.SwerveModuleIOSim;
import frc.robot.subsystems.swerve.SwerveState;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;

/** Class to contain all the parts/subsystems of the robot */
public class RobotContainer {

  private final DeceiverRobotState robotState;

  // Subsystems
  public final Swerve swerve;
  public final Vision vision;
  public final Arm arm;
  public final Gripper gripper;
  public final Leds leds;

  // Operator Interface
  private final OperatorInterface operatorInterface;

  /**
   * Constructor
   *
   * @param state RobotState object to track the state of the robot
   */
  public RobotContainer(DeceiverRobotState state) {

    // Setup robot state
    robotState = state;

    // Configure subsystems
    leds = new Leds();

    switch (RobotConstants.currentMode) {
      case COMMISIONING:
        // Fall through
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
        gripper = new Gripper(new GripperIOSpark());
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

    // TODO: transfer to the new LED class
    // Set up LED suppliers
    leds.setGripperSupplier(gripper::getState);
    leds.setSwerveSupplier(swerve::getSwerveState);

    // Configure Operator interface and set up command bindings
    operatorInterface = new OperatorInterface(robotState);
    configureAutoModes();
    configureBindings();
  }

  private void configureAutoModes() {
    // Set up Auto Triggers

    new EventTrigger("MoveToUpTravel")
        .onTrue(arm.setArmPosition(Rotation2d.fromDegrees(90), 0, new Rotation2d()));
    new EventTrigger("MoveToL4")
        .onTrue(
            arm.setArmPosition(Arm.ArmPosition.L4)
                .until(() -> arm.getArmInPosition())
                .withTimeout(5));
    new EventTrigger("AutoAlignRightWithTimeout")
        .onTrue(
            swerve
                .autoAlignCommand3D(vision, ReefSide.right)
                .until(() -> swerve.getSwerveState() == SwerveState.autoDriveDone)
                .withTimeout(1.0));
    new EventTrigger("AutoAlignLeftWithTimeout")
        .onTrue(
            swerve
                .autoAlignCommand3D(vision, ReefSide.left)
                .until(() -> swerve.getSwerveState() == SwerveState.autoDriveDone)
                .withTimeout(1.0));
    // new
    // EventTrigger("ejectCoral").onTrue(GripperCommands.coralAutoOuttake(gripper).withTimeout(2));
    new EventTrigger("MoveToPickup").onTrue(arm.setArmPosition(ArmPosition.PROCESSOR));
    // new EventTrigger("IntakeCoral").onTrue(GripperCommands.coralIntakeAuto(gripper));

    // Set up auto other routines
    if (RobotConstants.currentMode == Mode.COMMISIONING) {
      // Set up SysId routines
      operatorInterface.addAutoOption(
          "Drive Wheel Radius Characterization", swerve.wheelRadiusCharacterization());
      operatorInterface.addAutoOption(
          "Drive Simple FF Characterization", swerve.feedforwardCharacterization());
      operatorInterface.addAutoOption(
          "Drive SysId (Quasistatic Forward)",
          swerve.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
      operatorInterface.addAutoOption(
          "Drive SysId (Quasistatic Reverse)",
          swerve.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
      operatorInterface.addAutoOption(
          "Drive SysId (Dynamic Forward)", swerve.sysIdDynamic(SysIdRoutine.Direction.kForward));
      operatorInterface.addAutoOption(
          "Drive SysId (Dynamic Reverse)", swerve.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    }
  }

  /** Used to set up bidings for triggers, joystick buttons, default commands, etc */
  private void configureBindings() {

    // Trigger to toggle between camera types
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
    swerve.setDefaultCommand(swerve.joystickDrive(operatorInterface.getSwerveControlSignal()));

    // Smart Intake
    // operatorInterface.intakeTrigger().whileTrue(GripperCommands.coralIntake(gripper));

    // Smart Outttake
    // operatorInterface.outtakeTrigger().whileTrue(GripperCommands.coralOuttake(gripper));

    // Auto align to left branch from driver view
    operatorInterface
        .autoAlignLeftTrigger()
        .whileTrue(swerve.autoAlignCommand3D(vision, ReefSide.left));

    // Auto align to right branch from driver view
    operatorInterface
        .autoAlignRightTrigger()
        .whileTrue(swerve.autoAlignCommand3D(vision, ReefSide.right));

    // Reset gyro rotation, maintin position
    operatorInterface.restGyroTrigger().onTrue(Commands.runOnce(() -> swerve.resetGyro()));

    // Put drive in X position
    operatorInterface.stopWithXTrigger().whileTrue(swerve.stopWithX());

    // Rumble at the start of end game
    Trigger endGameTrigger = new Trigger(() -> DriverStation.getMatchTime() < 20);
    endGameTrigger.onTrue(operatorInterface.joystickRumbleCommand().withTimeout(Seconds.of(1)));

    // Generate a path to the specified point on the fly
    operatorInterface
        .pathPlanToPointTrigger()
        .whileTrue(
            swerve.pathfindToPose(
                new Pose2d(),
                MetersPerSecond.of(0),
                () -> robotState.getAlliance().orElse(Alliance.Blue)));
  }

  // This need to be replaced
  public void OperatorLoop() {
    if (operatorInterface.L2PositionTrigger()) {
      arm.setArmPostion(ArmPosition.L2);
    } else if (operatorInterface.L3PositionTrigger()) {
      arm.setArmPostion(ArmPosition.L3);
    } else if (operatorInterface.L4PositionTrigger()) {
      arm.setArmPostion(ArmPosition.L4);
    } else if (operatorInterface.L1PositionTrigger()) {
      arm.setArmPostion(ArmPosition.L1);
    } else if (operatorInterface.homePositionTrigger()) {
      arm.setArmPostion(ArmPosition.HOME);
    } else if (operatorInterface.intakePositionTrigger()) {
      arm.setArmPostion(ArmPosition.INTAKE);
    } else if (operatorInterface.algae1PositionTrigger()) {
      arm.setArmPostion(ArmPosition.ALGAE1);
    } else if (operatorInterface.algae2PositionTrigger()) {
      arm.setArmPostion(ArmPosition.ALGAE2);
    } else if (operatorInterface.bargePositionTrigger()) {
      arm.setArmPostion(ArmPosition.BARGE);
    } else if (operatorInterface.processorPositionTrigger()) {
      arm.setArmPostion(ArmPosition.PROCESSOR);
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return operatorInterface.getAutoCommand();
  }
}
