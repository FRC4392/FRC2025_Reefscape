// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Degrees;
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
import frc.robot.util.PhoenixUtil.ClosedLoopControlType;

/** Add your docs here. */
public class ArmConstants {

  // CAN IDs
  public static final int Pivot1CanId = 21;
  public static final int Pivot2CanId = 22;
  public static final int Pivot3CanId = 23;

  public static final int Extension1CanId = 31;
  public static final int Extension2CanId = 32;

  public static final int WristCanId = 41;

  public static final int PivotCanCoderID = 21;
  public static final int WristCanCoderID = 41;

  // Pivot Constants
  public static final double pivotGearReduction = (52.0 * 64.0 * 84.0) / (14.0 * 18.0 * 10.0);
  public static final DCMotor pivotGearbox = DCMotor.getKrakenX60Foc(3);
  public static final ClosedLoopControlType pivotControlType = ClosedLoopControlType.Voltage;

  public static final Rotation2d maxAngle = new Rotation2d(Units.degreesToRadians(170.0));
  public static final Rotation2d minAngle = new Rotation2d(Units.degreesToRadians(-21));

  public static final double pivotMotorStatorCurrentLimit = 100;

  public static final TalonFXConfiguration pivotMotorBaseConfig =
      new TalonFXConfiguration()
          .withAudio(
              new AudioConfigs()
                  .withAllowMusicDurDisable(true)
                  .withBeepOnBoot(true)
                  .withBeepOnConfig(true))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(pivotMotorStatorCurrentLimit)
                  .withStatorCurrentLimitEnable(true)
                  .withSupplyCurrentLimit(80)
                  .withSupplyCurrentLowerLimit(40)
                  .withSupplyCurrentLowerTime(3)
                  .withSupplyCurrentLimitEnable(true))
          .withFeedback(
              new FeedbackConfigs()
                  .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
                  // .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                  .withFeedbackRemoteSensorID(PivotCanCoderID)
                  .withSensorToMechanismRatio(1)
                  .withRotorToSensorRatio(pivotGearReduction))
          // .withSensorToMechanismRatio(pivotGearReduction))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withInverted(InvertedValue.Clockwise_Positive)
                  .withNeutralMode(NeutralModeValue.Brake))
          .withSlot0(
              new Slot0Configs()
                  .withKP(50.0)
                  .withKI(0.0)
                  .withKD(0.0)
                  .withKG(.4)
                  .withKV(11.0)
                  .withKS(0.0)
                  .withKA(0.0)
                  .withGravityType(GravityTypeValue.Arm_Cosine))
          .withTorqueCurrent(
              new TorqueCurrentConfigs()
                  .withPeakForwardTorqueCurrent(pivotMotorStatorCurrentLimit)
                  .withPeakReverseTorqueCurrent(-pivotMotorStatorCurrentLimit))
          .withSoftwareLimitSwitch(
              new SoftwareLimitSwitchConfigs()
                  .withForwardSoftLimitThreshold(maxAngle.getRotations())
                  .withForwardSoftLimitEnable(true)
                  .withReverseSoftLimitThreshold(Degrees.of(-29))
                  .withReverseSoftLimitEnable(true))
          .withMotionMagic(
              new MotionMagicConfigs()
                  .withMotionMagicAcceleration(DegreesPerSecondPerSecond.of(300.0))
                  .withMotionMagicCruiseVelocity(DegreesPerSecond.of(300.0))
                  .withMotionMagicJerk(
                      DegreesPerSecondPerSecond.of(500.0).in(RotationsPerSecondPerSecond)));

  // Extension Constants
  public static final double extensionGearReduction = (66.0) / (11.0);
  public static final double driveDiameter = Units.inchesToMeters(1.757);
  public static final double driveRadius = driveDiameter / 2.0;
  public static final DCMotor extensionGearbox = DCMotor.getKrakenX60Foc(2);

  public static final ClosedLoopControlType extensionControlType = ClosedLoopControlType.Voltage;

  public static final double extensionMotorStatorCurrentLimit = 120;

  public static final TalonFXConfiguration extensionMotorBaseConfig =
      new TalonFXConfiguration()
          .withAudio(
              new AudioConfigs()
                  .withAllowMusicDurDisable(true)
                  .withBeepOnBoot(true)
                  .withBeepOnConfig(true))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(extensionMotorStatorCurrentLimit)
                  .withStatorCurrentLimitEnable(true)
                  .withSupplyCurrentLimit(180)
                  .withSupplyCurrentLowerLimit(40)
                  .withSupplyCurrentLowerTime(3)
                  .withSupplyCurrentLimitEnable(true))
          .withFeedback(
              new FeedbackConfigs()
                  .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                  .withSensorToMechanismRatio(extensionGearReduction))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withInverted(InvertedValue.CounterClockwise_Positive)
                  .withNeutralMode(NeutralModeValue.Brake))
          .withSlot0(
              new Slot0Configs()
                  .withKP(300.0)
                  .withKI(0.0)
                  .withKD(0.0)
                  .withKG(0.25)
                  .withKV(.25)
                  .withKS(0.0)
                  .withKA(0.0)
                  .withGravityType(GravityTypeValue.Elevator_Static))
          .withTorqueCurrent(
              new TorqueCurrentConfigs()
                  .withPeakForwardTorqueCurrent(extensionMotorStatorCurrentLimit)
                  .withPeakReverseTorqueCurrent(-extensionMotorStatorCurrentLimit))
          .withSoftwareLimitSwitch(
              new SoftwareLimitSwitchConfigs()
                  .withForwardSoftLimitThreshold(3.0)
                  .withForwardSoftLimitEnable(true)
                  .withReverseSoftLimitThreshold(Units.degreesToRotations(0))
                  .withReverseSoftLimitEnable(true))
          .withMotionMagic(
              new MotionMagicConfigs()
                  .withMotionMagicAcceleration(DegreesPerSecondPerSecond.of(4000.0))
                  .withMotionMagicCruiseVelocity(DegreesPerSecond.of(4000.0))
                  .withMotionMagicJerk(
                      DegreesPerSecondPerSecond.of(10000.0).in(RotationsPerSecondPerSecond)));

  // Wrist Constants
  public static final double wristReduction = (56.0 * 40.0 * 36.0) / (10.0 * 15.0 * 15.0);
  public static final DCMotor wristGearbox = DCMotor.getKrakenX60Foc(1);

  public static final ClosedLoopControlType wristControlType = ClosedLoopControlType.Voltage;

  public static final double wristMotorStatorCurrentLimit = 40;

  public static final TalonFXConfiguration wristnMotorBaseConfig =
      new TalonFXConfiguration()
          .withAudio(
              new AudioConfigs()
                  .withAllowMusicDurDisable(true)
                  .withBeepOnBoot(true)
                  .withBeepOnConfig(true))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(wristMotorStatorCurrentLimit)
                  .withStatorCurrentLimitEnable(true)
                  .withSupplyCurrentLimit(50)
                  .withSupplyCurrentLowerLimit(20)
                  .withSupplyCurrentLowerTime(1)
                  .withSupplyCurrentLimitEnable(true))
          .withFeedback(
              new FeedbackConfigs()
                  .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
                  .withFeedbackRemoteSensorID(WristCanId)
                  .withRotorToSensorRatio(wristReduction)
                  .withSensorToMechanismRatio(1))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withInverted(InvertedValue.CounterClockwise_Positive)
                  .withNeutralMode(NeutralModeValue.Brake))
          .withSlot0(
              new Slot0Configs()
                  .withKP(74)
                  .withKI(0.0)
                  .withKD(0.0)
                  .withKG(0)
                  .withKV(7.5)
                  .withKS(0.0)
                  .withKA(0.0))
          .withTorqueCurrent(
              new TorqueCurrentConfigs()
                  .withPeakForwardTorqueCurrent(wristMotorStatorCurrentLimit)
                  .withPeakReverseTorqueCurrent(-wristMotorStatorCurrentLimit))
          .withSoftwareLimitSwitch(
              new SoftwareLimitSwitchConfigs()
                  .withForwardSoftLimitThreshold(Degrees.of(190))
                  .withForwardSoftLimitEnable(true)
                  .withReverseSoftLimitThreshold(Degrees.of(0))
                  .withReverseSoftLimitEnable(true))
          .withMotionMagic(
              new MotionMagicConfigs()
                  .withMotionMagicAcceleration(DegreesPerSecondPerSecond.of(720.0 / 4.0)) // 2
                  .withMotionMagicCruiseVelocity(DegreesPerSecond.of(360.0 / 2.0)) // 2
                  .withMotionMagicJerk(
                      DegreesPerSecondPerSecond.of(3600.0 / 5.0)
                          .in(RotationsPerSecondPerSecond))); // 4
}
