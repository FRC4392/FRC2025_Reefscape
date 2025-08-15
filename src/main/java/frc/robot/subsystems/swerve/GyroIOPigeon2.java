// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import static frc.robot.subsystems.swerve.SwerveConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import java.util.Queue;

/** Gyro implementation using a pigeon2 */
public class GyroIOPigeon2 implements GyroIO {
  private final Pigeon2 gyro = new Pigeon2(pigeonCanId);
  private final StatusSignal<Angle> yaw = gyro.getYaw();
  private final StatusSignal<AngularVelocity> yawVelocity = gyro.getAngularVelocityZWorld();
  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;

  /** Constructor */
  public GyroIOPigeon2() {
    gyro.getConfigurator().apply(new Pigeon2Configuration());
    gyro.getConfigurator().setYaw(0.0);
    yaw.setUpdateFrequency(odometryFrequencyHz);
    yawVelocity.setUpdateFrequency(50.0);
    gyro.optimizeBusUtilization();
    yawTimestampQueue = SwerveOdometryThread.getInstance().makeTimestampQueue();
    yawPositionQueue = SwerveOdometryThread.getInstance().registerSignal(yaw);
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.isConnected = BaseStatusSignal.refreshAll(yaw, yawVelocity).equals(StatusCode.OK);
    inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());

    inputs.odometryYawTimestamps =
        yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryYawPositions =
        yawPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromDegrees(value))
            .toArray(Rotation2d[]::new);
    yawTimestampQueue.clear();
    yawPositionQueue.clear();
  }
}
