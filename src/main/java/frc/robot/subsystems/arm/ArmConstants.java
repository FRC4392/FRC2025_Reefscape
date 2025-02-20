// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import frc.robot.util.PhoenixUtil.ClosedLoopControlType;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class ArmConstants {

        // CAN IDs
        public static final int Pivot1CanId = 21;
        public static final int Pivot2CanId = 22;
        public static final int Pivot3CanId = 23;

        public static final int Extension1CanId = 31;
        public static final int Extension2CanId = 32;

        public static final int WristCanId = 41;

        // Pivot Constants
        public static final double pivotGearReduction = (74.0 * 64.0 * 84.0) / (22.0 * 18.0 * 10.0);
        public static final DCMotor pivotGearbox = DCMotor.getKrakenX60Foc(3);
        public static final ClosedLoopControlType pivotControlType = ClosedLoopControlType.Voltage;

        public static final Rotation2d maxAngle = new Rotation2d(Units.degreesToRadians(120));
        public static final Rotation2d minAngle = new Rotation2d(Units.degreesToRadians(-29));

        public static final double pivotMotorStatorCurrentLimit = 40;

        public static final TalonFXConfiguration pivotMotorBaseConfig = new TalonFXConfiguration()
                        .withAudio(new AudioConfigs()
                                        .withAllowMusicDurDisable(true)
                                        .withBeepOnBoot(true)
                                        .withBeepOnConfig(true))
                        .withCurrentLimits(new CurrentLimitsConfigs()
                                        .withStatorCurrentLimit(pivotMotorStatorCurrentLimit)
                                        .withStatorCurrentLimitEnable(true)
                                        .withSupplyCurrentLimit(80)
                                        .withSupplyCurrentLowerLimit(30)
                                        .withSupplyCurrentLowerTime(1)
                                        .withSupplyCurrentLimitEnable(true))
                        .withFeedback(new FeedbackConfigs()
                                        .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                                        .withSensorToMechanismRatio(pivotGearReduction))
                        .withMotorOutput(new MotorOutputConfigs()
                                        .withInverted(InvertedValue.Clockwise_Positive)
                                        .withNeutralMode(NeutralModeValue.Brake))
                        .withSlot0(new Slot0Configs()
                                        .withKP(0.01)
                                        .withKI(0.0)
                                        .withKD(0.0)
                                        .withKG(0)
                                        .withKV(0.0)
                                        .withKS(0.0)
                                        .withKA(0.0)
                                        .withGravityType(GravityTypeValue.Arm_Cosine))
                        .withTorqueCurrent(new TorqueCurrentConfigs()
                                        .withPeakForwardTorqueCurrent(pivotMotorStatorCurrentLimit)
                                        .withPeakReverseTorqueCurrent(-pivotMotorStatorCurrentLimit))
                        .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
                                        .withForwardSoftLimitThreshold(maxAngle.getRotations())
                                        .withForwardSoftLimitEnable(true)
                                        .withReverseSoftLimitThreshold(
                                                        Units.degreesToRotations(minAngle.getRotations()))
                                        .withReverseSoftLimitEnable(true))
                        .withMotionMagic(new MotionMagicConfigs()
                                        .withMotionMagicAcceleration(DegreesPerSecondPerSecond.of(10.0))
                                        .withMotionMagicCruiseVelocity(DegreesPerSecond.of(10.0))
                                        .withMotionMagicJerk(DegreesPerSecondPerSecond.of(100.0)
                                                        .in(RotationsPerSecondPerSecond)));

        // Extension Constants
        public static final double extensionGearReduction = (66.0) / (11.0);
        public static final double driveDiameter = Units.inchesToMeters(1.757);
        public static final double driveRadius = driveDiameter / 2.0;
        public static final DCMotor extensionGearbox = DCMotor.getKrakenX60Foc(2);

        public static final ClosedLoopControlType extensionControlType = ClosedLoopControlType.Voltage;

        public static final double extensionMotorStatorCurrentLimit = 40;

        public static final TalonFXConfiguration extensionMotorBaseConfig = new TalonFXConfiguration()
                        .withAudio(new AudioConfigs()
                                        .withAllowMusicDurDisable(true)
                                        .withBeepOnBoot(true)
                                        .withBeepOnConfig(true))
                        .withCurrentLimits(new CurrentLimitsConfigs()
                                        .withStatorCurrentLimit(extensionMotorStatorCurrentLimit)
                                        .withStatorCurrentLimitEnable(true)
                                        .withSupplyCurrentLimit(80)
                                        .withSupplyCurrentLowerLimit(30)
                                        .withSupplyCurrentLowerTime(1)
                                        .withSupplyCurrentLimitEnable(true))
                        .withFeedback(new FeedbackConfigs()
                                        .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                                        .withSensorToMechanismRatio(extensionGearReduction))
                        .withMotorOutput(new MotorOutputConfigs()
                                        .withInverted(InvertedValue.CounterClockwise_Positive)
                                        .withNeutralMode(NeutralModeValue.Brake))
                        .withSlot0(new Slot0Configs()
                                        .withKP(0.01)
                                        .withKI(0.0)
                                        .withKD(0.0)
                                        .withKG(0)
                                        .withKV(0.0)
                                        .withKS(0.0)
                                        .withKA(0.0))
                        .withTorqueCurrent(new TorqueCurrentConfigs()
                                        .withPeakForwardTorqueCurrent(extensionMotorStatorCurrentLimit)
                                        .withPeakReverseTorqueCurrent(-extensionMotorStatorCurrentLimit))
                        .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
                                        .withForwardSoftLimitThreshold(2.5)
                                        .withForwardSoftLimitEnable(true)
                                        .withReverseSoftLimitThreshold(Units.degreesToRotations(.1))
                                        .withReverseSoftLimitEnable(true))
                        .withMotionMagic(new MotionMagicConfigs()
                                        .withMotionMagicAcceleration(DegreesPerSecondPerSecond.of(10.0))
                                        .withMotionMagicCruiseVelocity(DegreesPerSecond.of(10.0))
                                        .withMotionMagicJerk(DegreesPerSecondPerSecond.of(100.0)
                                                        .in(RotationsPerSecondPerSecond)));

        // Wrist Constants
        public static final double wristReduction = (56.0 * 40.0 * 36.0) / (10.0 * 15.0 * 15.0);
        public static final DCMotor wristGearbox = DCMotor.getKrakenX60Foc(1);

        public static final ClosedLoopControlType wristControlType = ClosedLoopControlType.Voltage;

        public static final double wristMotorStatorCurrentLimit = 40;

        public static final TalonFXConfiguration wristnMotorBaseConfig = new TalonFXConfiguration()
                        .withAudio(new AudioConfigs()
                                        .withAllowMusicDurDisable(true)
                                        .withBeepOnBoot(true)
                                        .withBeepOnConfig(true))
                        .withCurrentLimits(new CurrentLimitsConfigs()
                                        .withStatorCurrentLimit(wristMotorStatorCurrentLimit)
                                        .withStatorCurrentLimitEnable(true)
                                        .withSupplyCurrentLimit(80)
                                        .withSupplyCurrentLowerLimit(30)
                                        .withSupplyCurrentLowerTime(1)
                                        .withSupplyCurrentLimitEnable(true))
                        .withFeedback(new FeedbackConfigs()
                                        .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                                        .withSensorToMechanismRatio(wristReduction))
                        .withMotorOutput(new MotorOutputConfigs()
                                        .withInverted(InvertedValue.CounterClockwise_Positive)
                                        .withNeutralMode(NeutralModeValue.Brake))
                        .withSlot0(new Slot0Configs()
                                        .withKP(0.01)
                                        .withKI(0.0)
                                        .withKD(0.0)
                                        .withKG(0)
                                        .withKV(0.0)
                                        .withKS(0.0)
                                        .withKA(0.0))
                        .withTorqueCurrent(new TorqueCurrentConfigs()
                                        .withPeakForwardTorqueCurrent(wristMotorStatorCurrentLimit)
                                        .withPeakReverseTorqueCurrent(-wristMotorStatorCurrentLimit))
                        .withMotionMagic(new MotionMagicConfigs()
                                        .withMotionMagicAcceleration(DegreesPerSecondPerSecond.of(10.0))
                                        .withMotionMagicCruiseVelocity(DegreesPerSecond.of(10.0))
                                        .withMotionMagicJerk(DegreesPerSecondPerSecond.of(100.0)
                                                        .in(RotationsPerSecondPerSecond)));;

}
