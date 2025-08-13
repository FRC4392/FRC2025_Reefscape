// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.gripper;

import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

/** Add your docs here. */
public class GripperConstants {

  // CAN IDs
  public static final int CoralCanId = 51;
  public static final int AlgaeCanId = 41;

  public static final double CoralReduction = 2;
  public static final double AlgaeReduction = 2;

  public static final boolean CoralInverted = false;
  public static final boolean AlgaeInverted = true;

  public static final int coralCurrentLimit = 40;
  public static final int algaeCurrentLimit = 80;

  public static final double coralVelocityConversionFactor = 2;
  public static final double coralPositionConversionFactor = 2;
  public static final double algaeVelocityConversionFactor = 2;
  public static final double algaePositionConversionFactor = 2;

  public static final int coralSensorPort = 0;
  public static final int algaeSensorPort = 1;

  public static final TalonFXConfiguration driveConfiguration =
      new TalonFXConfiguration()
          .withAudio(
              new AudioConfigs()
                  .withAllowMusicDurDisable(true)
                  .withBeepOnBoot(true)
                  .withBeepOnConfig(true))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(algaeCurrentLimit)
                  .withStatorCurrentLimitEnable(true)
                  .withSupplyCurrentLimit(algaeCurrentLimit * 2)
                  .withSupplyCurrentLowerLimit(algaeCurrentLimit)
                  .withSupplyCurrentLowerTime(1)
                  .withSupplyCurrentLimitEnable(true))
          .withFeedback(
              new FeedbackConfigs()
                  .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                  .withSensorToMechanismRatio(1))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withInverted(InvertedValue.CounterClockwise_Positive)
                  .withNeutralMode(NeutralModeValue.Brake))
          .withTorqueCurrent(
              new TorqueCurrentConfigs()
                  .withPeakForwardTorqueCurrent(algaeCurrentLimit)
                  .withPeakReverseTorqueCurrent(-algaeCurrentLimit));
}
