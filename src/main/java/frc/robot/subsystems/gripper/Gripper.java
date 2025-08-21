// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.gripper;

import static frc.robot.subsystems.gripper.GripperConstants.*;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Gripper extends SubsystemBase {

  private final GripperIO gripperIO;
  private final GripperIOInputsAutoLogged inputs = new GripperIOInputsAutoLogged();

  private final Alert coralMotorConnectionAlert =
      new Alert("Gripper Motor Disconnected, gripper may not work", AlertType.kError);

  private GripperState state = GripperState.OFF;

  /** Creates a new Gripper. */
  public Gripper(GripperIO gripperIO) {
    this.gripperIO = gripperIO;
  }

  @Override
  public void periodic() {
    // Update inputs
    gripperIO.updateInputs(inputs);
    Logger.processInputs("Gripper", inputs);

    // Check for disconnections
    coralMotorConnectionAlert.set(!inputs.motorConnected);
  }

  // Set the coral motor voltage
  public void setVoltage(double voltage) {
    gripperIO.setGripperVoltage(voltage);
  }

  // Get the current state of the gripper
  @AutoLogOutput(key = "Gripper/State")
  public GripperState getState() {
    return state;
  }

  public void stop() {
    setVoltage(0);
  }

  public Command intakeCommand() {
    return Commands.sequence(
        this.run(() -> setVoltage(12)).until(() -> inputs.motorCurrentAmps > stopIntakeAmps),
        this.run(() -> setVoltage(3)));
  }

  public Command outtakeCommand() {
    return this.startEnd(() -> setVoltage(-12), this::stop);
  }

  public Command stopCommand() {
    return this.runOnce(this::stop);
  }

  // State of the gripper
  public enum GripperState {
    OFF,
    IntakeOuttakeWithCoral,
    IntakeOuttakeWithAlgae,
    IntakeOuttakeWithBoth,
    IntakeOuttakeWithNone,
    HasAlgae,
    HasCoral,
    HasBoth;
  }
}
