// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.gripper;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface GripperIO {
  @AutoLog
  public static class GripperIOInputs {
    public boolean motorConnected = false;
    public double motorPositionRad = 0.0;
    public double motorVelocityRadPerSec = 0.0;
    public double motorAppliedVolts = 0.0;
    public double motorCurrentAmps = 0.0;
    public double motorTemp = 0.0;
  }

  public default void updateInputs(GripperIOInputs inputs) {}

  public default void setGripperVoltage(double voltage) {}
}
