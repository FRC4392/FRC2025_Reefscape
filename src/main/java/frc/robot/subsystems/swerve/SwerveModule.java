// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import static frc.robot.subsystems.swerve.SwerveConstants.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

/** Add your docs here. */
public class SwerveModule {
    private final SwerveModuleIO io;
    private final SwerveModuleIOInputsAutoLogged inputs = new SwerveModuleIOInputsAutoLogged();
    private final int index;

    private final Alert driveDisconnectedAlert;
    private final Alert azimuthDisconnectedAlert;

    private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

    public SwerveModule(SwerveModuleIO io, int index) {
        this.io = io;
        this.index = index;

        driveDisconnectedAlert = new Alert("Disconnected drive motor on module " + Integer.toString(index) + ".",
                AlertType.kError);
        azimuthDisconnectedAlert = new Alert("Disconnected azimuth motor on module " + Integer.toString(index) + ".",
                AlertType.kError);
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Drive/Module" + Integer.toString(index), inputs);

        // Calculate positions for odometry
        int sampleCount = inputs.odometryTimestamps.length; // All signals are sampled together
        odometryPositions = new SwerveModulePosition[sampleCount];
        for (int i = 0; i < sampleCount; i++) {
            double positionMeters = inputs.odometryDrivePositionsRad[i] * wheelRadiusMeters;
            Rotation2d angle = inputs.odometryAzimuthPositions[i];
            odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
        }

        driveDisconnectedAlert.set(!inputs.driveConnected);
        azimuthDisconnectedAlert.set(!inputs.azimuthConnected);
    }

    /**
     * Runs the module with the specified setpoint state. Mutates the state to
     * optimize it.
     */
    public void runSetpoint(SwerveModuleState state) {
        // Optimize velocity setpoint
        state.optimize(getAngle());
        state.cosineScale(getAngle());

        // Apply setpoints
        io.setDriveVelocity(state.speedMetersPerSecond / wheelRadiusMeters);
        io.setAzimuthPosition(state.angle);
    }

    /**
     * Runs the module with the specified output while controlling to zero degrees.
     */
    public void runCharacterization(double output) {
        io.setDriveOpenLoop(output);
        io.setAzimuthPosition(new Rotation2d());
    }

    /** Disables all outputs to motors. */
    public void stop() {
        io.setDriveOpenLoop(0.0);
        io.setAzimuthOpenLoop(0.0);
    }

    /** Returns the current turn angle of the module. */
    public Rotation2d getAngle() {
        return inputs.azimuthPosition;
    }

    /** Returns the current drive position of the module in meters. */
    public double getPositionMeters() {
        return inputs.drivePositionRad * wheelRadiusMeters;
    }

    /** Returns the current drive velocity of the module in meters per second. */
    public double getVelocityMetersPerSec() {
        return inputs.driveVelocityRadPerSec * wheelRadiusMeters;
    }

    /** Returns the module position (turn angle and drive position). */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getPositionMeters(), getAngle());
    }

    /** Returns the module state (turn angle and drive velocity). */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
    }

    /** Returns the module positions received this cycle. */
    public SwerveModulePosition[] getOdometryPositions() {
        return odometryPositions;
    }

    /** Returns the timestamps of the samples received this cycle. */
    public double[] getOdometryTimestamps() {
        return inputs.odometryTimestamps;
    }

    /** Returns the module position in radians. */
    public double getWheelRadiusCharacterizationPosition() {
        return inputs.drivePositionRad;
    }

    /** Returns the module velocity in rad/sec. */
    public double getFFCharacterizationVelocity() {
        return inputs.driveVelocityRadPerSec;
    }
}
