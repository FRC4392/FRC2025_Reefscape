// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.gripper;

/** Add your docs here. */
public final class GripperConstants {
  private GripperConstants() {}

  // CAN IDs
  public static final int MotorCanId = 51;

  public static final double GripperReduction = 8;

  public static final boolean GripperInverted = false;
  public static final boolean AlgaeInverted = true;

  public static final int GripperCurrentLimit = 40;
  public static final int algaeCurrentLimit = 40;

  public static final double GripperVelocityConversionFactor = 8;
  public static final double GripperPositionConversionFactor = 8;

  public static final double stopIntakeAmps = 20;
}
