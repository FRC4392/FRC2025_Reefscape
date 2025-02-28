// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.gripper;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Gripper extends SubsystemBase {

  private final GripperIO gripperIO;
  private final GripperIOInputsAutoLogged inputs = new GripperIOInputsAutoLogged();

  private final Alert coralMotorConnectionAlert =
      new Alert("Coral Motor Disconnected, coral manipulator may not work", AlertType.kError);
  private final Alert algaeMotorConnectionAlert =
      new Alert("Algae Motor Disconnected, algae manipulator may not work", AlertType.kError);
  /** Creates a new Gripper. */
  public Gripper(GripperIO gripperIO) {
    this.gripperIO = gripperIO;
  }

  @Override
  public void periodic() {
    gripperIO.updateInputs(inputs);
    Logger.processInputs("Gripper", inputs);

    coralMotorConnectionAlert.set(!inputs.coralMotorConnected);
    algaeMotorConnectionAlert.set(!inputs.algaeMotorConnected);
  }

  public void setCoralVoltage(double voltage) {
    gripperIO.setCoralMotorVoltage(voltage);
  }

  public void setAlgaeVoltage(double voltage) {
    gripperIO.setAlgaeMotorVoltage(voltage);
  }

  public boolean getCoralPresent() {
    return gripperIO.getCoralPresent();
  }

  public boolean getAlgaePresent() {
    return gripperIO.getAlgaePresent();
  }
}
