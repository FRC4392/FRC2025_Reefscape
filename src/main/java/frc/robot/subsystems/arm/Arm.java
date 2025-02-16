// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {

  private final ArmIO armIO;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  private final Alert pivot1DisconnectedAlert = new Alert("Pivot Motor 1 Disconnected, arm may fail to work",
      AlertType.kError);
  private final Alert pivot2DisconnectedAlert = new Alert(
      "Pivot Motor 2 Disconnected, arm may have reduced performance", AlertType.kError);
  private final Alert pivot3DisconnectedAlert = new Alert(
      "Pivot Motor 3 Disconnected, arm may have reduced performance", AlertType.kError);

  private final Alert extension1DisconnectedAlert = new Alert(
      "Extension Motor 1 Disconnected, extension may fail to work", AlertType.kError);
  private final Alert extension2DisconnectedAlert = new Alert(
      "Extension Motor 2 Disconnected, extension may have reduced performance", AlertType.kError);

  private final Alert wristDisconnectedAlert = new Alert("Wrist Motor Disconnected, wrist may fail to work",
      AlertType.kError);

  /** Creates a new Arm. */
  public Arm(ArmIO armIO) {
    this.armIO = armIO;
  }

  @Override
  public void periodic() {
    armIO.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);

    pivot1DisconnectedAlert.set(!inputs.basePivotMotor1Connected);
    pivot2DisconnectedAlert.set(!inputs.basePivotMotor2Connected);
    pivot3DisconnectedAlert.set(!inputs.basePivotMotor3Connected);

    extension1DisconnectedAlert.set(!inputs.extensionMotor1Connected);
    extension2DisconnectedAlert.set(!inputs.extensionMotor2Connected);

    wristDisconnectedAlert.set(!inputs.wristMotorConnected);
  }

  public void setPosition(Rotation2d pivotRotation, double length, Rotation2d wristAngle) {
    armIO.setAngle(pivotRotation);
    armIO.setLength(length);
    armIO.setWrist(wristAngle);
  }
  public void setPivotVoltage(double voltage){
    armIO.setPivotVoltage(voltage);
  }
  public void setExtensionVoltage(double voltage){
    armIO.setExtensionVoltage(voltage);
  }
  public void setWristVoltage(double voltage){
    armIO.setWristVoltage(voltage);
  }
}
