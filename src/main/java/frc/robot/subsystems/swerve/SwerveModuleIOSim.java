// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import static frc.robot.subsystems.swerve.SwerveConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Add your docs here. */
public class SwerveModuleIOSim implements SwerveModuleIO {
  private final DCMotorSim driveSim;
  private final DCMotorSim azimuthSim;

  private boolean driveClosedLoop = false;
  private boolean azimuthClosedLoop = false;
  private PIDController driveController = new PIDController(driveSimP, 0, driveSimD);
  private PIDController azimuthController = new PIDController(azimuthSimP, 0, azimuthSimD);
  private double driveFFVolts = 0.0;
  private double driveAppliedVolts = 0.0;
  private double azimuthAppliedVolts = 0.0;

  public SwerveModuleIOSim() {
    // Create drive and azimuth sim models
    driveSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(driveGearbox, 0.025, driveMotorReduction),
            driveGearbox);
    azimuthSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(azimuthGearbox, 0.004, azimuthMotorReduction),
            azimuthGearbox);

    // Enable wrapping for azimuth PID
    azimuthController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void updateInputs(SwerveModuleIOInputs inputs) {
    // Run closed-loop control
    if (driveClosedLoop) {
      driveAppliedVolts =
          driveFFVolts + driveController.calculate(driveSim.getAngularVelocityRadPerSec());
    } else {
      driveController.reset();
    }
    if (azimuthClosedLoop) {
      azimuthAppliedVolts = azimuthController.calculate(azimuthSim.getAngularPositionRad());
    } else {
      azimuthController.reset();
    }

    // Update simulation state
    driveSim.setInputVoltage(MathUtil.clamp(driveAppliedVolts, -12.0, 12.0));
    azimuthSim.setInputVoltage(MathUtil.clamp(azimuthAppliedVolts, -12.0, 12.0));
    driveSim.update(0.02);
    azimuthSim.update(0.02);

    // Update drive inputs
    inputs.driveConnected = true;
    inputs.drivePositionRad = driveSim.getAngularPositionRad();
    inputs.driveVelocityRadPerSec = driveSim.getAngularVelocityRadPerSec();
    inputs.driveAppliedVolts = driveAppliedVolts;
    inputs.driveCurrentAmps = Math.abs(driveSim.getCurrentDrawAmps());

    // Update azimuth inputs
    inputs.azimuthConnected = true;
    inputs.azimuthPosition = new Rotation2d(azimuthSim.getAngularPositionRad());
    inputs.azimuthVelocityRadPerSec = azimuthSim.getAngularVelocityRadPerSec();
    inputs.azimuthAppliedVolts = azimuthAppliedVolts;
    inputs.azimuthCurrentAmps = Math.abs(azimuthSim.getCurrentDrawAmps());

    // Update odometry inputs (50Hz because high-frequency odometry in sim doesn't
    // matter)
    inputs.odometryTimestamps = new double[] {Timer.getFPGATimestamp()};
    inputs.odometryDrivePositionsRad = new double[] {inputs.drivePositionRad};
    inputs.odometryAzimuthPositions = new Rotation2d[] {inputs.azimuthPosition};
  }

  @Override
  public void setDriveOpenLoop(double output) {
    driveClosedLoop = false;
    driveAppliedVolts = output;
  }

  @Override
  public void setAzimuthOpenLoop(double output) {
    azimuthClosedLoop = false;
    azimuthAppliedVolts = output;
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    driveClosedLoop = true;
    driveFFVolts = driveSimKs * Math.signum(velocityRadPerSec) + driveSimKv * velocityRadPerSec;
    driveController.setSetpoint(velocityRadPerSec);
  }

  @Override
  public void setAzimuthPosition(Rotation2d rotation) {
    azimuthClosedLoop = true;
    azimuthController.setSetpoint(rotation.getRadians());
  }
}
