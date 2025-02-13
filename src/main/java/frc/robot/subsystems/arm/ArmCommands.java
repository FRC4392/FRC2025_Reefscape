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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.function.DoubleSupplier;

public class ArmCommands {

  private static final double DEADBAND = 0.01;
  private static final double MAX_OUTPUT = 0.1;

  private ArmCommands() {}

  public static Command joystickPivot(
      Arm arm,
      DoubleSupplier velocitySupplier) {
    return Commands.run(
        () -> {
          double velocity = MathUtil.applyDeadband(velocitySupplier.getAsDouble(), DEADBAND);

          velocity = velocity * MAX_OUTPUT;

          // Square rotation value for more precise control
          velocity = Math.copySign(velocity * velocity, velocity);

          double voltage = velocity*12.0;

          arm.setPivotVoltage(voltage);
        },
        arm);
  }

  
}
