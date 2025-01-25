// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.*;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.swerve.GyroIO;
import frc.robot.subsystems.swerve.GyroIOPigeon2;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveModuleIO;
import frc.robot.subsystems.swerve.SwerveModuleIODeceivers;
import frc.robot.subsystems.swerve.SwerveModuleIOSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;

public class RobotContainer {

    // Subsystems
    public final Swerve swerve;
    public final Vision vision;

    // Controller
    private final CommandXboxController driveController = new CommandXboxController(0);

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;

    public RobotContainer() {

        driveController.setRumble(RumbleType.kBothRumble, 0);

        switch (RobotConstants.currentMode) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                swerve = new Swerve(
                        new GyroIOPigeon2(),
                        new SwerveModuleIODeceivers(0),
                        new SwerveModuleIODeceivers(1),
                        new SwerveModuleIODeceivers(2),
                        new SwerveModuleIODeceivers(3));

                vision = new Vision(swerve::addVisionMeasurement,
                        new VisionIOLimelight(camera0Name, swerve::getRotation));

                break;

            case SIM:
                // Sim robot, instantiate physics sim IO implementations
                swerve = new Swerve(
                        new GyroIO() {
                        },
                        new SwerveModuleIOSim(),
                        new SwerveModuleIOSim(),
                        new SwerveModuleIOSim(),
                        new SwerveModuleIOSim());

                vision = new Vision(swerve::addVisionMeasurement,
                        new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, swerve::getPose));
                break;

            default:
                // Replayed robot, disable IO implementations
                swerve = new Swerve(
                        new GyroIO() {
                        },
                        new SwerveModuleIO() {
                        },
                        new SwerveModuleIO() {
                        },
                        new SwerveModuleIO() {
                        },
                        new SwerveModuleIO() {
                        });

                vision = new Vision(swerve::addVisionMeasurement,
                        new VisionIO() {
                        });
                break;
        }

        // Set up auto routines
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        // Set up SysId routines
        autoChooser.addOption(
                "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(swerve));
        autoChooser.addOption(
                "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(swerve));
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
        autoChooser.addOption(
                "19 Score", new PathPlannerAuto("19 Score"));
        autoChooser.addOption(
                "14 Score", new PathPlannerAuto("14 Score"));
        autoChooser.addOption(
                "8 Score", new PathPlannerAuto("8 Score"));

        configureBindings();
    }

    private void configureBindings() {
        // Default command, normal field-relative drive
        swerve.setDefaultCommand(
                DriveCommands.joystickDrive(
                        swerve,
                        () -> -driveController.getLeftY(),
                        () -> -driveController.getLeftX(),
                        () -> driveController.getLeftTriggerAxis() - driveController.getRightTriggerAxis()));

    }

    public Command getAutonomousCommand() {
        return autoChooser.get();
    }
}
