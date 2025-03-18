// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.DoubleSupplier;

public class ArmCommands {

  private static final double DEADBAND = 0.01;
  private static final double MAX_OUTPUT = 1;

  private ArmCommands() {}

  public static Command joystickArmControl(
      Arm arm,
      DoubleSupplier pivotSupplier,
      DoubleSupplier extensionSupplier,
      DoubleSupplier wristSupplier) {
    return Commands.run(
        () -> {
          double pivotVelocity = MathUtil.applyDeadband(pivotSupplier.getAsDouble(), DEADBAND);

          pivotVelocity = pivotVelocity * MAX_OUTPUT;

          // Square rotation value for more precise control
          pivotVelocity = Math.copySign(pivotVelocity * pivotVelocity, pivotVelocity);

          double pivotVoltage = pivotVelocity * 12.0;

          arm.setPivotVoltage(pivotVoltage);

          double extensionVelocity =
              MathUtil.applyDeadband(extensionSupplier.getAsDouble(), DEADBAND);

          extensionVelocity = extensionVelocity * MAX_OUTPUT;

          // Square rotation value for more precise control
          extensionVelocity =
              Math.copySign(extensionVelocity * extensionVelocity, extensionVelocity);

          double extensionVoltage = extensionVelocity * 12.0;

          arm.setExtensionVoltage(extensionVoltage);

          double wristVelocity = MathUtil.applyDeadband(wristSupplier.getAsDouble(), DEADBAND);

          wristVelocity = wristVelocity * .5;

          wristVelocity = Math.copySign(wristVelocity * wristVelocity, wristVelocity);

          double wristVoltage = wristVelocity * 12.0;

          arm.setWristVoltage(wristVoltage);
        },
        arm);
  }

  public static Command setArmPosition(
      Arm arm, Rotation2d armPivot, double armExtension, Rotation2d wristPosition) {
    return Commands.run(
            () -> {
              arm.setPosition(armPivot, armExtension, wristPosition);
            },
            arm)
        .until(arm::getArmInPosition);
  }

  public static Command setArmPosition(Arm arm, Arm.ArmPosition position) {
    return Commands.run(
            () -> {
              arm.setPosition(position);
            },
            arm)
        .until(arm::getArmInPosition);
  }
}
