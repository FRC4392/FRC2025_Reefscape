// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
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
import frc.robot.subsystems.gripper.GripperIOSpark;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.swerve.GyroIO;
import frc.robot.subsystems.swerve.GyroIOPigeon2;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveCommands;
import frc.robot.subsystems.swerve.SwerveModuleIO;
import frc.robot.subsystems.swerve.SwerveModuleIODeceivers;
import frc.robot.subsystems.swerve.SwerveModuleIOSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {

  // Subsystems
  public final Swerve swerve;
  public final Vision vision;
  public final Arm arm;
  public final Gripper gripper;
  public final Leds leds;

  // Controller
  private final CommandXboxController driveController = new CommandXboxController(0);
  private final CommandXboxController operateController = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  public RobotContainer() {

    driveController.setRumble(RumbleType.kBothRumble, 0);
    operateController.setRumble(RumbleType.kBothRumble, 0);

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
                new SwerveModuleIODeceivers(3));

        vision =
            new Vision(
                swerve::addVisionMeasurement,
                new VisionIOLimelight(camera0Name, swerve::getRotation));

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
                new SwerveModuleIOSim());

        vision =
            new Vision(
                swerve::addVisionMeasurement,
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
                new SwerveModuleIO() {});

        vision = new Vision(swerve::addVisionMeasurement, new VisionIO() {});

        arm = new Arm(new ArmIO() {});
        gripper = new Gripper(new GripperIO() {});

        NamedCommands.registerCommand("DropOffLow", arm.getDropOffLowCommand());
        NamedCommands.registerCommand(
            "ejectCoral", GripperCommands.coralOuttake(gripper).withTimeout(.5));

        break;
    }

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
    autoChooser.addOption("19 automatic", new PathPlannerAuto("19 automatic"));
    autoChooser.addOption("14 Score", new PathPlannerAuto("14 Score"));
    autoChooser.addOption("8 Score", new PathPlannerAuto("8 Score"));
    autoChooser.addOption("Test Drive Forward", new PathPlannerAuto("Test Drive Forward"));

    configureBindings();
  }

  private void configureBindings() {
    // Default command, normal field-relative drive
    swerve.setDefaultCommand(
        SwerveCommands.joystickDrive(
            swerve,
            () -> -driveController.getLeftY(),
            () -> -driveController.getLeftX(),
            () -> driveController.getLeftTriggerAxis() - driveController.getRightTriggerAxis()));
    // arm.setDefaultCommand(
    //     ArmCommands.joystickArmControl(
    //         arm,
    //         () -> operateController.getLeftY() * -1,
    //         () -> operateController.getRightY() * -1,
    //         () ->
    //             operateController.getLeftTriggerAxis() -
    // operateController.getRightTriggerAxis()));

    // operateController
    //     .a()
    //     .onTrue(
    //         ArmCommands.setArmPosition(
    //             arm,
    //             new Rotation2d(Units.degreesToRadians(120)),
    //             Units.inchesToMeters(0),
    //             new Rotation2d(Units.degreesToRadians(20))));
    // operateController
    //     .b()
    //     .onTrue(
    //         ArmCommands.setArmPosition(
    //             arm,
    //             new Rotation2d(Units.degreesToRadians(100)),
    //             Units.inchesToMeters(5),
    //             new Rotation2d(Units.degreesToRadians(15))));
    // operateController
    //     .x()
    //     .onTrue(
    //         ArmCommands.setArmPosition(
    //             arm,
    //             new Rotation2d(Units.degreesToRadians(80)),
    //             Units.inchesToMeters(15),
    //             new Rotation2d(Units.degreesToRadians(15))));
    // operateController
    //     .y()
    //     .onTrue(
    //         ArmCommands.setArmPosition(
    //             arm,
    //             new Rotation2d(Units.degreesToRadians(90)),
    //             Units.inchesToMeters(10),
    //             new Rotation2d(Units.degreesToRadians(90))));
    // operateController
    //     .start()
    //     .onTrue(
    //         ArmCommands.setArmPosition(
    //             arm,
    //             new Rotation2d(Units.degreesToRadians(5)),
    //             Units.inchesToMeters(0),
    //             new Rotation2d(Units.degreesToRadians(1))));

    driveController.a().whileTrue(GripperCommands.algaeIntake(gripper));
    driveController.b().whileTrue(GripperCommands.algaeOutake(gripper));
    driveController.leftStick().whileTrue(GripperCommands.coralIntake(gripper));
    driveController.rightStick().whileTrue(GripperCommands.coralOuttake(gripper));

    driveController
        .rightStick()
        .and(operateController.y())
        .onTrue(
            Commands.sequence(
                Commands.waitSeconds(.1),
                ArmCommands.setArmPosition(
                    arm,
                    new Rotation2d(Units.degreesToRadians(80)),
                    Units.inchesToMeters(15),
                    new Rotation2d(Units.degreesToRadians(30)))));

    Trigger testTrigger = new Trigger(() -> DriverStation.getMatchTime() < 40);
    testTrigger.onTrue(Commands.run(() -> driveController.setRumble(RumbleType.kBothRumble, 1.0)));
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void OperatorLoop() {
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
    } else if (operateController.getHID().getLeftBumperButton()) {
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
