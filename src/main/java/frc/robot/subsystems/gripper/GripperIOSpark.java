// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.gripper;

import static frc.robot.subsystems.gripper.GripperConstants.*;
import static frc.robot.subsystems.swerve.SwerveConstants.odometryFrequencyHz;
import static frc.robot.util.SparkUtil.ifOk;
import static frc.robot.util.SparkUtil.sparkStickyFault;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import java.util.function.DoubleSupplier;

/** Add your docs here. */
public class GripperIOSpark implements GripperIO {

  private final SparkFlex coralMotor = new SparkFlex(CoralCanId, MotorType.kBrushless);
  private final SparkFlex algaeMotor = new SparkFlex(AlgaeCanId, MotorType.kBrushless);

  private final RelativeEncoder coralEncoder;
  private final RelativeEncoder algaeEncoder;

  private final Debouncer coralConnectedDebounce = new Debouncer(.5);
  private final Debouncer algaeConnectedDebounce = new Debouncer(.5);

  public GripperIOSpark() {

    var coralConfig = new SparkMaxConfig();
    coralConfig
        .inverted(CoralInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(coralCurrentLimit)
        .voltageCompensation(12.0);
    coralConfig
        .absoluteEncoder
        .inverted(CoralInverted)
        .positionConversionFactor(coralPositionConversionFactor)
        .velocityConversionFactor(coralPositionConversionFactor)
        .averageDepth(2);
    coralConfig
        .signals
        .absoluteEncoderPositionAlwaysOn(true)
        .absoluteEncoderPositionPeriodMs((int) (1000.0 / odometryFrequencyHz))
        .absoluteEncoderVelocityAlwaysOn(true)
        .absoluteEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);

    var algaeConfig = new SparkMaxConfig();
    algaeConfig
        .inverted(AlgaeInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(algaeCurrentLimit)
        .voltageCompensation(12.0);
    algaeConfig
        .absoluteEncoder
        .inverted(AlgaeInverted)
        .positionConversionFactor(algaePositionConversionFactor)
        .velocityConversionFactor(algaePositionConversionFactor)
        .averageDepth(2);
    algaeConfig
        .signals
        .absoluteEncoderPositionAlwaysOn(true)
        .absoluteEncoderPositionPeriodMs((int) (1000.0 / odometryFrequencyHz))
        .absoluteEncoderVelocityAlwaysOn(true)
        .absoluteEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    tryUntilOk(
        algaeMotor,
        5,
        () ->
            algaeMotor.configure(
                coralConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    coralEncoder = coralMotor.getEncoder();
    algaeEncoder = algaeMotor.getEncoder();
  }

  @Override
  public void updateInputs(GripperIOInputs inputs) {
    sparkStickyFault = false;
    ifOk(
        coralMotor,
        coralEncoder::getVelocity,
        (value) -> inputs.coralMotorVelocityRadPerSec = value);
    ifOk(
        coralMotor,
        new DoubleSupplier[] {coralMotor::getAppliedOutput, coralMotor::getBusVoltage},
        (values) -> inputs.coralMotorAppliedVolts = values[0] * values[1]);
    ifOk(coralMotor, coralMotor::getOutputCurrent, (value) -> inputs.coralMotorCurrentAmps = value);
    ifOk(coralMotor, coralMotor::getMotorTemperature, (value) -> inputs.coralMotorTemp = value);
    inputs.coralMotorConnected = coralConnectedDebounce.calculate(!sparkStickyFault);

    sparkStickyFault = false;
    ifOk(
        algaeMotor,
        algaeEncoder::getVelocity,
        (value) -> inputs.algaeMotorVelocityRadPerSec = value);
    ifOk(
        algaeMotor,
        new DoubleSupplier[] {algaeMotor::getAppliedOutput, algaeMotor::getBusVoltage},
        (values) -> inputs.algaeMotorAppliedVolts = values[0] * values[1]);
    ifOk(algaeMotor, algaeMotor::getOutputCurrent, (value) -> inputs.algaeMotorCurrentAmps = value);
    ifOk(algaeMotor, algaeMotor::getMotorTemperature, (value) -> inputs.algaeMotorTemp = value);
    inputs.algaeMotorConnected = algaeConnectedDebounce.calculate(!sparkStickyFault);
  }

  @Override
  public void setAlgaeMotorVoltage(double voltage) {
    algaeMotor.setVoltage(voltage);
  }

  @Override
  public void setCoralMotorVoltage(double voltage) {
    coralMotor.setVoltage(voltage);
  }
}
