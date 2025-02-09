// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class SwerveConstants {
    public static final double maxSpeedMetersPerSec = 4.8;
    public static final double odometryFrequencyHz = 100.0; // Hz
    public static final double trackWidth = Units.inchesToMeters(23.5);
    public static final double wheelBase = Units.inchesToMeters(25.5);
    public static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);
    public static final Translation2d[] moduleTranslations = new Translation2d[] {
            new Translation2d(trackWidth / 2.0, wheelBase / 2.0),
            new Translation2d(trackWidth / 2.0, -wheelBase / 2.0),
            new Translation2d(-trackWidth / 2.0, wheelBase / 2.0),
            new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0)
    };

    // Zeroed rotation values for each module, see setup instructions
    public static final Rotation2d frontLeftZeroRotation = new Rotation2d(0.0);
    public static final Rotation2d frontRightZeroRotation = new Rotation2d(0.0);
    public static final Rotation2d backLeftZeroRotation = new Rotation2d(0.0);
    public static final Rotation2d backRightZeroRotation = new Rotation2d(0.0);

    // Device CAN IDs
    public static final int pigeonCanId = 11;

    public static final int frontLeftDriveCanId = 12;
    public static final int backLeftDriveCanId = 11;
    public static final int frontRightDriveCanId = 13;
    public static final int backRightDriveCanId = 14;

    public static final int frontLeftAzimuthCanId = 12;
    public static final int backLeftAzimuthCanId = 11;
    public static final int frontRightAzimuthCanId = 13;
    public static final int backRightAzimuthCanId = 14;

    // Drive motor configuration
    public static final int driveMotorStatorLimit = 80;
    public static final int driveMotorSupplyLimitHigh = 80;
    public static final int driveMotorSupplyLimitLow = 40;
    public static final double wheelDiameterMeters = Units.inchesToMeters(2.84);
    public static final double wheelRadiusMeters = wheelDiameterMeters / 2.0;
    public static final double driveMotorReduction = (45.0 * 28.0) / (20.0 * 15.0);
    public static final DCMotor driveGearbox = DCMotor.getKrakenX60Foc(1);
//     public static final boolean driveMotorInverted = true;
    public static final ClosedLoopOutputType driveMotorClosedLoopOutput = ClosedLoopOutputType.Voltage;

    // Drive PID configuration
    public static final double driveKp = 0.05;
    public static final double driveKi = 0.0;
    public static final double driveKd = 0.0;
    
    public static final double driveKs = 0.15;
    public static final double driveKv = 0.5;
    public static final double driveKa = 0.0;

    public static final double driveSimP = 0.05;
    public static final double driveSimD = 0.0;
    public static final double driveSimKs = 0.0;
    public static final double driveSimKv = 0.0789;

    public static final TalonFXConfiguration driveConfiguration = new TalonFXConfiguration()
            .withAudio(new AudioConfigs()
                    .withAllowMusicDurDisable(true)
                    .withBeepOnBoot(true)
                    .withBeepOnConfig(true))
            .withCurrentLimits(new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(driveMotorStatorLimit)
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(driveMotorSupplyLimitHigh)
                    .withSupplyCurrentLowerLimit(driveMotorSupplyLimitLow)
                    .withSupplyCurrentLowerTime(1)
                    .withSupplyCurrentLimitEnable(true))
            .withFeedback(new FeedbackConfigs()
                    .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                    .withSensorToMechanismRatio(driveMotorReduction))
            .withMotorOutput(new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake))
            .withSlot0(new Slot0Configs()
                    .withKP(driveKp)
                    .withKI(driveKi)
                    .withKD(driveKd)
                    .withKG(0)
                    .withKV(driveKv)
                    .withKS(driveKs)
                    .withKA(driveKa))
            .withTorqueCurrent(new TorqueCurrentConfigs()
                    .withPeakForwardTorqueCurrent(driveMotorStatorLimit)
                    .withPeakReverseTorqueCurrent(-driveMotorStatorLimit));

    // Azimuth motor configuration
    public static final boolean azimuthInverted = true;
    public static final int azimuthMotorCurrentLimit = 30;
    public static final double azimuthMotorReduction = (32*20*63)/(8*14*18);
    public static final DCMotor azimuthGearbox = DCMotor.getNeo550(1);

    // Azimuth encoder configuration
    public static final boolean azimuthEncoderInverted = false;
    public static final double azimuthEncoderPositionFactor = 2 * Math.PI; // Rotations -> Radians
    public static final double azimuthEncoderVelocityFactor = (2 * Math.PI) / 60.0; // RPM -> Rad/Sec
    // Azimuth PID configuration
    public static final double azimuthKp = 3.0;
    public static final double azimuthKd = 0.0;
    public static final double azimuthSimP = 8.0;
    public static final double azimuthSimD = 0.0;
    public static final double azimuthPIDMinInput = 0; // Radians
    public static final double azimuthPIDMaxInput = 2 * Math.PI; // Radians

    // PathPlanner configuration
    public static final double robotMassKg = Units.lbsToKilograms(70);
    public static final double robotMOI = 7;
    public static final double wheelCOF = .5;
    public static final RobotConfig ppConfig = new RobotConfig(
            robotMassKg,
            robotMOI,
            new ModuleConfig(
                    wheelRadiusMeters,
                    maxSpeedMetersPerSec,
                    wheelCOF,
                    driveGearbox.withReduction(driveMotorReduction),
                    driveMotorStatorLimit,
                    1),
            moduleTranslations);
}
