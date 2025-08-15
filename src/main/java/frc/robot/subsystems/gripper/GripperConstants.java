// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.gripper;

/** Add your docs here. */
public final class GripperConstants {
  private GripperConstants() {}

  // CAN IDs
  public static final int CoralCanId = 51;
  public static final int AlgaeCanId = 41;

  public static final double CoralReduction = 2;
  public static final double AlgaeReduction = 2;

  public static final boolean CoralInverted = false;
  public static final boolean AlgaeInverted = true;

  public static final int coralCurrentLimit = 40;
  public static final int algaeCurrentLimit = 40;

  public static final double coralVelocityConversionFactor = 2;
  public static final double coralPositionConversionFactor = 2;
  public static final double algaeVelocityConversionFactor = 2;
  public static final double algaePositionConversionFactor = 2;

  public static final int coralSensorPort = 0;
  public static final int algaeSensorPort = 1;
}
