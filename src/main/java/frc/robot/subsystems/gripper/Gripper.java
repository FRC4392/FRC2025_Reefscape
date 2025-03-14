// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.gripper;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Gripper extends SubsystemBase {

  private final GripperIO gripperIO;
  private final GripperIOInputsAutoLogged inputs = new GripperIOInputsAutoLogged();

  private final Alert coralMotorConnectionAlert =
      new Alert("Coral Motor Disconnected, coral manipulator may not work", AlertType.kError);
  private final Alert algaeMotorConnectionAlert =
      new Alert("Algae Motor Disconnected, algae manipulator may not work", AlertType.kError);

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
    coralMotorConnectionAlert.set(!inputs.coralMotorConnected);
    algaeMotorConnectionAlert.set(!inputs.algaeMotorConnected);

    // Determine state
    if (Math.abs(inputs.coralMotorAppliedVolts) < 5
        && Math.abs(inputs.algaeMotorAppliedVolts) < 5) {
      if (getAlgaePresent() && getCoralPresent()) {
        state = GripperState.HasBoth;
      } else if (getAlgaePresent()) {
        state = GripperState.HasAlgae;
      } else if (getCoralPresent()) {
        state = GripperState.HasCoral;
      } else {
        state = GripperState.OFF;
      }
    } else {
      if (getAlgaePresent() && getCoralPresent()) {
        state = GripperState.IntakeOuttakeWithBoth;
      } else if (getAlgaePresent()) {
        state = GripperState.IntakeOuttakeWithAlgae;
      } else if (getCoralPresent()) {
        state = GripperState.IntakeOuttakeWithCoral;
      } else {
        state = GripperState.IntakeOuttakeWithNone;
      }
    }
  }

  // Set the coral motor voltage
  public void setCoralVoltage(double voltage) {
    gripperIO.setCoralMotorVoltage(voltage);
  }

  // Set the algae motor voltage
  public void setAlgaeVoltage(double voltage) {
    gripperIO.setAlgaeMotorVoltage(voltage);
  }

  // Get if coral is in the gripper
  public boolean getCoralPresent() {
    return gripperIO.getCoralPresent();
  }

  // Get if algae is in the gripper
  public boolean getAlgaePresent() {
    return gripperIO.getAlgaePresent();
  }

  // Get the current state of the gripper
  @AutoLogOutput(key = "Gripper/State")
  public GripperState getState() {
    return state;
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
