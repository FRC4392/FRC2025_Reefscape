// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.gripper;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/** Add your docs here. */
public class GripperCommands {

  public static Command coralIntake(Gripper gripper) {
    return Commands.runEnd(
        () -> {
          gripper.setCoralVoltage(12);
        },
        () -> {
          gripper.setCoralVoltage(0);
        },
        gripper);
  }

  public static Command coralOuttake(Gripper gripper) {
    return Commands.runEnd(
        () -> {
          gripper.setCoralVoltage(-12);
        },
        () -> {
          gripper.setCoralVoltage(0);
        },
        gripper);
  }

  public static Command algaeIntake(Gripper gripper) {
    return Commands.runEnd(
        () -> {
          gripper.setAlgaeVoltage(12);
        },
        () -> {
          gripper.setAlgaeVoltage(2);
        },
        gripper);
  }

  public static Command algaeOutake(Gripper gripper) {
    return Commands.runEnd(
        () -> {
          gripper.setAlgaeVoltage(-12);
        },
        () -> {
          gripper.setAlgaeVoltage(-2);
        },
        gripper);
  }
}
