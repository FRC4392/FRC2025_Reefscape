// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.gripper;

import static frc.robot.subsystems.gripper.GripperConstants.*;
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

  private final SparkFlex gripperMotor = new SparkFlex(MotorCanId, MotorType.kBrushless);

  private final RelativeEncoder gripperEncoder;

  private final Debouncer motorConnectedDebounce = new Debouncer(.5);

  public GripperIOSpark() {

    var coralConfig = new SparkMaxConfig();
    coralConfig
        .inverted(GripperInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(GripperCurrentLimit)
        .voltageCompensation(12.0);
    coralConfig
        .absoluteEncoder
        .inverted(GripperInverted)
        .positionConversionFactor(GripperPositionConversionFactor)
        .velocityConversionFactor(GripperVelocityConversionFactor)
        .averageDepth(2);
    coralConfig
        .signals
        .absoluteEncoderPositionAlwaysOn(true)
        .absoluteEncoderPositionPeriodMs(20)
        .absoluteEncoderVelocityAlwaysOn(true)
        .absoluteEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);

    tryUntilOk(
        gripperMotor,
        5,
        () ->
            gripperMotor.configure(
                coralConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    gripperEncoder = gripperMotor.getEncoder();
  }

  @Override
  public void updateInputs(GripperIOInputs inputs) {
    sparkStickyFault = false;
    ifOk(
        gripperMotor,
        gripperEncoder::getVelocity,
        (value) -> inputs.motorVelocityRadPerSec = value);
    ifOk(
        gripperMotor,
        new DoubleSupplier[] {gripperMotor::getAppliedOutput, gripperMotor::getBusVoltage},
        (values) -> inputs.motorAppliedVolts = values[0] * values[1]);
    ifOk(gripperMotor, gripperMotor::getOutputCurrent, (value) -> inputs.motorCurrentAmps = value);
    ifOk(gripperMotor, gripperMotor::getMotorTemperature, (value) -> inputs.motorTemp = value);
    inputs.motorConnected = motorConnectedDebounce.calculate(!sparkStickyFault);
  }

  @Override
  public void setGripperVoltage(double voltage) {
    gripperMotor.setVoltage(voltage);
  }
}
