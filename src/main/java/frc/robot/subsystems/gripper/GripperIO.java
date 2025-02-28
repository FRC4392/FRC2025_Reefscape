// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.gripper;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface GripperIO {
  @AutoLog
  public static class GripperIOInputs {
    public boolean coralMotorConnected = false;
    public double coralMotorPositionRad = 0.0;
    public double coralMotorVelocityRadPerSec = 0.0;
    public double coralMotorAppliedVolts = 0.0;
    public double coralMotorCurrentAmps = 0.0;
    public double coralMotorTemp = 0.0;

    public boolean algaeMotorConnected = false;
    public double algaeMotorPositionRad = 0.0;
    public double algaeMotorVelocityRadPerSec = 0.0;
    public double algaeMotorAppliedVolts = 0.0;
    public double algaeMotorCurrentAmps = 0.0;
    public double algaeMotorTemp = 0.0;

    public boolean coralPresent = false;
    public boolean algaePresent = false;
  }

  public default void updateInputs(GripperIOInputs inputs) {}

  public default void setAlgaeMotorVoltage(double voltage) {}

  public default void setCoralMotorVoltage(double voltage) {}

  public default boolean getCoralPresent() {
    return false;
  }

  public default boolean getAlgaePresent() {
    return false;
  }
}
