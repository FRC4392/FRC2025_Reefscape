// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import static frc.robot.subsystems.swerve.SwerveConstants.*;
import static frc.robot.util.SparkUtil.*;
import static frc.robot.util.SparkUtil.tryUntilOk;
import static frc.robot.util.PhoenixUtil.*;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import java.util.Queue;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class SwerveModuleIODeceivers implements SwerveModuleIO {
        private final Rotation2d zeroRotation;

        // Hardware
        private final SparkMax azimuthMotor;
        private final TalonFX driveMotor;
        private final AbsoluteEncoder azimuthEncoder;
        private final SparkClosedLoopController azimuthController;

        // Voltage control requests
        private final VoltageOut voltageRequest = new VoltageOut(0);
        private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0.0);

        // Torque-current control requests
        private final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0);
        private final VelocityTorqueCurrentFOC velocityTorqueCurrentRequest = new VelocityTorqueCurrentFOC(0.0);

        //Duty Cycle Control Requests
        private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0.0);
        private final VelocityDutyCycle velocityDutyCycle = new VelocityDutyCycle(0.0);

        // Inputs from drive motor
        private final StatusSignal<Angle> drivePosition;
        private final StatusSignal<AngularVelocity> driveVelocity;
        private final StatusSignal<Voltage> driveAppliedVolts;
        private final StatusSignal<Current> driveCurrent;
        private final StatusSignal<Temperature> driveTemp;

        // Odometry queue
        private final Queue<Double> timestampQueue;
        private final Queue<Double> drivePositionQueue;
        private final Queue<Double> turnPositionQueue;

        // Connection debouncers
        private final Debouncer driveConnectedDebounce = new Debouncer(0.5);
        private final Debouncer turnConnectedDebounce = new Debouncer(0.5);

        public SwerveModuleIODeceivers(int module) {

                zeroRotation = switch (module) {
                        case 0 -> frontLeftZeroRotation;
                        case 1 -> frontRightZeroRotation;
                        case 2 -> backLeftZeroRotation;
                        case 3 -> backRightZeroRotation;
                        default -> new Rotation2d();
                };

                azimuthMotor = new SparkMax(
                                switch (module) {
                                        case 0 -> frontLeftAzimuthCanId;
                                        case 1 -> frontRightAzimuthCanId;
                                        case 2 -> backLeftAzimuthCanId;
                                        case 3 -> backRightAzimuthCanId;
                                        default -> 0;
                                },
                                MotorType.kBrushless);

                driveMotor = new TalonFX(
                                switch (module) {
                                        case 0 -> frontLeftDriveCanId;
                                        case 1 -> frontRightDriveCanId;
                                        case 2 -> backLeftDriveCanId;
                                        case 3 -> backRightDriveCanId;
                                        default -> 0;
                                });

                azimuthEncoder = azimuthMotor.getAbsoluteEncoder();
                azimuthController = azimuthMotor.getClosedLoopController();

                tryUntilOk(5, () -> driveMotor.getConfigurator().apply(driveConfiguration, 0.25));
                tryUntilOk(5, () -> driveMotor.setPosition(0.0, 0.25));

                // Configure turn motor
                var azimuthConfig = new SparkMaxConfig();
                azimuthConfig
                                .inverted(azimuthInverted)
                                .idleMode(IdleMode.kBrake)
                                .smartCurrentLimit(azimuthMotorCurrentLimit)
                                .voltageCompensation(12.0);
                azimuthConfig.absoluteEncoder
                                .inverted(azimuthEncoderInverted)
                                .positionConversionFactor(azimuthEncoderPositionFactor)
                                .velocityConversionFactor(azimuthEncoderVelocityFactor)
                                .averageDepth(2);
                azimuthConfig.closedLoop
                                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                                .positionWrappingEnabled(true)
                                .positionWrappingInputRange(azimuthPIDMinInput, azimuthPIDMaxInput)
                                .pidf(azimuthKp, 0.0, azimuthKd, 0.0);
                azimuthConfig.signals
                                .absoluteEncoderPositionAlwaysOn(true)
                                .absoluteEncoderPositionPeriodMs((int) (1000.0 / odometryFrequencyHz))
                                .absoluteEncoderVelocityAlwaysOn(true)
                                .absoluteEncoderVelocityPeriodMs(20)
                                .appliedOutputPeriodMs(20)
                                .busVoltagePeriodMs(20)
                                .outputCurrentPeriodMs(20);
                tryUntilOk(
                                azimuthMotor,
                                5,
                                () -> azimuthMotor.configure(
                                                azimuthConfig, ResetMode.kResetSafeParameters,
                                                PersistMode.kPersistParameters));

                // Create drive status signals
                drivePosition = driveMotor.getPosition();
                driveVelocity = driveMotor.getVelocity();
                driveAppliedVolts = driveMotor.getMotorVoltage();
                driveCurrent = driveMotor.getStatorCurrent();
                driveTemp = driveMotor.getDeviceTemp();

                // Configure periodic frames
                BaseStatusSignal.setUpdateFrequencyForAll(
                                odometryFrequencyHz, drivePosition);
                BaseStatusSignal.setUpdateFrequencyForAll(
                                50.0,
                                driveVelocity,
                                driveAppliedVolts,
                                driveCurrent);
                ParentDevice.optimizeBusUtilizationForAll(driveMotor);

                // Create odometry queues
                timestampQueue = SwerveOdometryThread.getInstance().makeTimestampQueue();
                drivePositionQueue = SwerveOdometryThread.getInstance().registerSignal(driveMotor.getPosition());
                turnPositionQueue = SwerveOdometryThread.getInstance().registerSignal(azimuthMotor,
                                azimuthEncoder::getPosition);

        }

        @Override
        public void updateInputs(SwerveModuleIOInputs inputs) {
                // Update drive inputs
                var driveStatus = BaseStatusSignal.refreshAll(drivePosition, driveVelocity, driveAppliedVolts,
                                driveCurrent, driveTemp);

                inputs.driveConnected = driveConnectedDebounce.calculate(driveStatus.isOK());
                inputs.drivePositionRad = Units.rotationsToRadians(drivePosition.getValueAsDouble());
                inputs.driveVelocityRadPerSec = Units.rotationsToRadians(driveVelocity.getValueAsDouble());
                inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
                inputs.driveCurrentAmps = driveCurrent.getValueAsDouble();
                inputs.driveMotorTemp = driveTemp.getValueAsDouble();

                // Update turn inputs
                sparkStickyFault = false;
                ifOk(
                                azimuthMotor,
                                azimuthEncoder::getPosition,
                                (value) -> inputs.azimuthPosition = new Rotation2d(value).minus(zeroRotation));
                ifOk(azimuthMotor, azimuthEncoder::getVelocity, (value) -> inputs.azimuthVelocityRadPerSec = value);
                ifOk(
                                azimuthMotor,
                                new DoubleSupplier[] { azimuthMotor::getAppliedOutput, azimuthMotor::getBusVoltage },
                                (values) -> inputs.azimuthAppliedVolts = values[0] * values[1]);
                ifOk(azimuthMotor, azimuthMotor::getOutputCurrent, (value) -> inputs.azimuthCurrentAmps = value);
                ifOk(azimuthMotor, azimuthMotor::getMotorTemperature, (value) -> inputs.azimuthMotorTemp = value);
                inputs.azimuthConnected = turnConnectedDebounce.calculate(!sparkStickyFault);

                // Update odometry inputs
                inputs.odometryTimestamps = timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
                inputs.odometryDrivePositionsRad = drivePositionQueue.stream()
                                .mapToDouble((Double value) -> Units.rotationsToRadians(value)).toArray();
                inputs.odometryAzimuthPositions = turnPositionQueue.stream()
                                .map((Double value) -> new Rotation2d(value).minus(zeroRotation))
                                .toArray(Rotation2d[]::new);
                timestampQueue.clear();
                drivePositionQueue.clear();
                turnPositionQueue.clear();
        }

        @Override
        public void setAzimuthOpenLoop(double output) {
                azimuthMotor.setVoltage(output);
        }

        @Override
        public void setAzimuthPosition(Rotation2d rotation) {
                double setpoint = MathUtil.inputModulus(
                                rotation.plus(zeroRotation).getRadians(), azimuthPIDMinInput, azimuthPIDMaxInput);
                azimuthController.setReference(setpoint, ControlType.kPosition);
        }

        @Override
        public void setDriveVelocity(double velocityRadPerSec) {
                double velocityRotPerSec = Units.radiansToRotations(velocityRadPerSec);
                driveMotor.setControl(
                                switch (driveMotorClosedLoopOutput) {
                                        case Voltage -> velocityVoltageRequest.withVelocity(velocityRotPerSec);
                                        case TorqueCurrentFOC ->
                                                velocityTorqueCurrentRequest.withVelocity(velocityRotPerSec);
                                        case DutyCyle -> velocityDutyCycle.withVelocity(velocityRotPerSec);
                                });
        }

        @Override
        public void setDriveOpenLoop(double output) {
                driveMotor.setControl(
                                switch (driveMotorClosedLoopOutput) {
                                        case Voltage -> voltageRequest.withOutput(output);
                                        case TorqueCurrentFOC -> torqueCurrentRequest.withOutput(output);
                                        case DutyCyle -> dutyCycleRequest.withOutput(output);
                                });
        }

}
