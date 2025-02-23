// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import static frc.robot.subsystems.arm.ArmConstants.*;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

/** Add your docs here. */
public class ArmIOTalonFX implements ArmIO {

  private final ArmFeedforward pivotFeedforward = new ArmFeedforward(0, .23, 0);

  // Motors
  private final TalonFX pivotMotor1 = new TalonFX(Pivot1CanId);
  private final TalonFX pivotMotor2 = new TalonFX(Pivot2CanId);
  private final TalonFX pivotMotor3 = new TalonFX(Pivot3CanId);

  private final TalonFX extensionMotor1 = new TalonFX(Extension1CanId);
  private final TalonFX extensionMotor2 = new TalonFX(Extension2CanId);

  private final TalonFX wristMotor = new TalonFX(WristCanId);

  // Status Signals
  private final StatusSignal<Angle> pivotMotor1Position;
  private final StatusSignal<Angle> pivotMotor2Position;
  private final StatusSignal<Angle> pivotMotor3Position;
  private final StatusSignal<AngularVelocity> pivotMotor1Velocity;
  private final StatusSignal<AngularVelocity> pivotMotor2Velocity;
  private final StatusSignal<AngularVelocity> pivotMotor3Velocity;
  private final StatusSignal<Voltage> pivotMotor1AppliedVolts;
  private final StatusSignal<Voltage> pivotMotor2AppliedVolts;
  private final StatusSignal<Voltage> pivotMotor3AppliedVolts;
  private final StatusSignal<Current> pivotMotor1Current;
  private final StatusSignal<Current> pivotMotor2Current;
  private final StatusSignal<Current> pivotMotor3Current;
  private final StatusSignal<Temperature> pivotMotor1Temp;
  private final StatusSignal<Temperature> pivotMotor2Temp;
  private final StatusSignal<Temperature> pivotMotor3Temp;

  private final StatusSignal<Angle> extensionMotor1Position;
  private final StatusSignal<Angle> extensionMotor2Position;
  private final StatusSignal<AngularVelocity> extensionMotor1Velocity;
  private final StatusSignal<AngularVelocity> extensionMotor2Velocity;
  private final StatusSignal<Voltage> extensionMotor1AppliedVolts;
  private final StatusSignal<Voltage> extensionMotor2AppliedVolts;
  private final StatusSignal<Current> extensionMotor1Current;
  private final StatusSignal<Current> extensionMotor2Current;
  private final StatusSignal<Temperature> extensionMotor1Temp;
  private final StatusSignal<Temperature> extensionMotor2Temp;

  private final StatusSignal<Angle> wristPosition;
  private final StatusSignal<AngularVelocity> wristVelocity;
  private final StatusSignal<Voltage> wristAppliedVolts;
  private final StatusSignal<Current> wristCurrent;
  private final StatusSignal<Temperature> wristTemp;

  // Leader status signals
  private final StatusSignal<Double> pivotLeaderDutyCycle;
  private final StatusSignal<Double> extensionLeaderDutyCycle;
  private final StatusSignal<Current> pivotLeaderTorqueCurrent;
  private final StatusSignal<Current> extensionLeaderTorqueCurrent;

  // Control Requests
  private final MotionMagicDutyCycle pivotMotionMagicDutyCycle;
  private final MotionMagicVoltage pivotMotionMagicVoltage;
  private final MotionMagicTorqueCurrentFOC pivotMotionMagicTorqueCurrentFOC;

  private final MotionMagicDutyCycle extensionMotionMagicDutyCycle;
  private final MotionMagicVoltage extensionMotionMagicVoltage;
  private final MotionMagicTorqueCurrentFOC extensionMotionMagicTorqueCurrentFOC;

  private final MotionMagicDutyCycle wristMotionMagicDutyCycle;
  private final MotionMagicVoltage wristMotionMagicVoltage;
  private final MotionMagicTorqueCurrentFOC wristMotionMagicTorqueCurrentFOC;

  // Connection debouncers
  private final Debouncer pivot1ConnectedDebouncer = new Debouncer(0.5);
  private final Debouncer pivot2ConnectedDebouncer = new Debouncer(0.5);
  private final Debouncer pivot3ConnectedDebouncer = new Debouncer(0.5);
  private final Debouncer extension1ConnectedDebouncer = new Debouncer(0.5);
  private final Debouncer extension2ConnectedDebouncer = new Debouncer(0.5);
  private final Debouncer wristConnectedDebouncer = new Debouncer(0.5);

  public ArmIOTalonFX() {
    // Configure motors
    tryUntilOk(5, () -> pivotMotor1.getConfigurator().apply(pivotMotorBaseConfig, .25));
    tryUntilOk(5, () -> pivotMotor2.getConfigurator().apply(pivotMotorBaseConfig, .25));
    tryUntilOk(5, () -> pivotMotor3.getConfigurator().apply(pivotMotorBaseConfig, .25));

    tryUntilOk(5, () -> pivotMotor1.setPosition(minAngle.getRotations()));
    tryUntilOk(5, () -> pivotMotor2.setPosition(minAngle.getRotations()));
    tryUntilOk(5, () -> pivotMotor3.setPosition(minAngle.getRotations()));

    tryUntilOk(5, () -> pivotMotor2.setControl(new Follower(pivotMotor1.getDeviceID(), false)));
    tryUntilOk(5, () -> pivotMotor3.setControl(new Follower(pivotMotor1.getDeviceID(), false)));

    tryUntilOk(5, () -> extensionMotor1.getConfigurator().apply(extensionMotorBaseConfig, .25));
    tryUntilOk(5, () -> extensionMotor2.getConfigurator().apply(extensionMotorBaseConfig, .25));

    tryUntilOk(
        5, () -> extensionMotor2.setControl(new Follower(extensionMotor1.getDeviceID(), false)));

    tryUntilOk(5, () -> wristMotor.getConfigurator().apply(wristnMotorBaseConfig, .25));

    // Configure status signals
    pivotMotor1Position = pivotMotor1.getPosition();
    pivotMotor2Position = pivotMotor2.getPosition();
    pivotMotor3Position = pivotMotor3.getPosition();
    pivotMotor1Velocity = pivotMotor1.getVelocity();
    pivotMotor2Velocity = pivotMotor2.getVelocity();
    pivotMotor3Velocity = pivotMotor3.getVelocity();
    pivotMotor1AppliedVolts = pivotMotor1.getMotorVoltage();
    pivotMotor2AppliedVolts = pivotMotor2.getMotorVoltage();
    pivotMotor3AppliedVolts = pivotMotor3.getMotorVoltage();
    pivotMotor1Current = pivotMotor1.getStatorCurrent();
    pivotMotor2Current = pivotMotor2.getStatorCurrent();
    pivotMotor3Current = pivotMotor3.getStatorCurrent();
    pivotMotor1Temp = pivotMotor1.getDeviceTemp();
    pivotMotor2Temp = pivotMotor2.getDeviceTemp();
    pivotMotor3Temp = pivotMotor3.getDeviceTemp();

    extensionMotor1Position = extensionMotor1.getPosition();
    extensionMotor2Position = extensionMotor2.getPosition();
    extensionMotor1Velocity = extensionMotor1.getVelocity();
    extensionMotor2Velocity = extensionMotor2.getVelocity();
    extensionMotor1AppliedVolts = extensionMotor1.getMotorVoltage();
    extensionMotor2AppliedVolts = extensionMotor2.getMotorVoltage();
    extensionMotor1Current = extensionMotor1.getStatorCurrent();
    extensionMotor2Current = extensionMotor2.getStatorCurrent();
    extensionMotor1Temp = extensionMotor1.getDeviceTemp();
    extensionMotor2Temp = extensionMotor2.getDeviceTemp();

    wristPosition = wristMotor.getPosition();
    wristVelocity = wristMotor.getVelocity();
    wristAppliedVolts = wristMotor.getMotorVoltage();
    wristCurrent = wristMotor.getStatorCurrent();
    wristTemp = wristMotor.getDeviceTemp();

    // singnals required for follower mode
    pivotLeaderDutyCycle = pivotMotor1.getDutyCycle();
    extensionLeaderDutyCycle = extensionMotor1.getDutyCycle();
    pivotLeaderTorqueCurrent = pivotMotor1.getTorqueCurrent();
    extensionLeaderTorqueCurrent = extensionMotor1.getTorqueCurrent();

    // configure control requests
    pivotMotionMagicVoltage = new MotionMagicVoltage(pivotMotor1.getPosition().getValue());
    pivotMotionMagicDutyCycle = new MotionMagicDutyCycle(pivotMotor1.getPosition().getValue());
    pivotMotionMagicTorqueCurrentFOC =
        new MotionMagicTorqueCurrentFOC(pivotMotor1.getPosition().getValue());

    extensionMotionMagicVoltage = new MotionMagicVoltage(extensionMotor1.getPosition().getValue());
    extensionMotionMagicDutyCycle =
        new MotionMagicDutyCycle(extensionMotor1.getPosition().getValue());
    extensionMotionMagicTorqueCurrentFOC =
        new MotionMagicTorqueCurrentFOC(extensionMotor1.getPosition().getValue());

    wristMotionMagicVoltage = new MotionMagicVoltage(wristMotor.getPosition().getValue());
    wristMotionMagicDutyCycle = new MotionMagicDutyCycle(wristMotor.getPosition().getValue());
    wristMotionMagicTorqueCurrentFOC =
        new MotionMagicTorqueCurrentFOC(wristMotor.getPosition().getValue());

    BaseStatusSignal.setUpdateFrequencyForAll(
        100,
        pivotLeaderDutyCycle,
        extensionLeaderDutyCycle,
        pivotLeaderTorqueCurrent,
        extensionLeaderTorqueCurrent);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        pivotMotor1Position,
        pivotMotor2Position,
        pivotMotor3Position,
        pivotMotor1Velocity,
        pivotMotor2Velocity,
        pivotMotor3Velocity,
        pivotMotor1AppliedVolts,
        pivotMotor2AppliedVolts,
        pivotMotor3AppliedVolts,
        pivotMotor1Current,
        pivotMotor2Current,
        pivotMotor3Current,
        pivotMotor1Temp,
        pivotMotor2Temp,
        pivotMotor3Temp,
        extensionMotor1Position,
        extensionMotor2Position,
        extensionMotor1Velocity,
        extensionMotor2Velocity,
        extensionMotor1AppliedVolts,
        extensionMotor2AppliedVolts,
        extensionMotor1Current,
        extensionMotor2Current,
        extensionMotor1Temp,
        extensionMotor2Temp,
        wristPosition,
        wristVelocity,
        wristAppliedVolts,
        wristCurrent,
        wristTemp);

    ParentDevice.optimizeBusUtilizationForAll(
        pivotMotor1, pivotMotor2, pivotMotor3, extensionMotor1, extensionMotor2, wristMotor);
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    var pivot1Status =
        BaseStatusSignal.refreshAll(
            pivotMotor1AppliedVolts,
            pivotMotor1Current,
            pivotMotor1Position,
            pivotMotor1Velocity,
            pivotMotor1Temp);

    inputs.basePivotMotor1Connected = pivot1ConnectedDebouncer.calculate(pivot1Status.isOK());
    inputs.basePivotMotor1AppliedVolts = pivotMotor1AppliedVolts.getValueAsDouble();
    inputs.basePivotMotor1CurrentAmps = pivotMotor1Current.getValueAsDouble();
    inputs.basePivotMotor1PositionRad =
        Units.rotationsToRadians(pivotMotor1Position.getValueAsDouble());
    inputs.basePivotMotor1VelocityRadPerSec =
        Units.rotationsToRadians(pivotMotor1Velocity.getValueAsDouble());
    inputs.basePivotMotor1Temp = pivotMotor1Temp.getValueAsDouble();

    var pivot2Status =
        BaseStatusSignal.refreshAll(
            pivotMotor2AppliedVolts,
            pivotMotor2Current,
            pivotMotor2Position,
            pivotMotor2Velocity,
            pivotMotor2Temp);

    inputs.basePivotMotor2Connected = pivot2ConnectedDebouncer.calculate(pivot2Status.isOK());
    inputs.basePivotMotor2AppliedVolts = pivotMotor2AppliedVolts.getValueAsDouble();
    inputs.basePivotMotor2CurrentAmps = pivotMotor2Current.getValueAsDouble();
    inputs.basePivotMotor2PositionRad =
        Units.rotationsToRadians(pivotMotor2Position.getValueAsDouble());
    inputs.basePivotMotor2VelocityRadPerSec =
        Units.rotationsToRadians(pivotMotor2Velocity.getValueAsDouble());
    inputs.basePivotMotor2Temp = pivotMotor2Temp.getValueAsDouble();

    var pivot3Status =
        BaseStatusSignal.refreshAll(
            pivotMotor3AppliedVolts,
            pivotMotor3Current,
            pivotMotor3Position,
            pivotMotor3Velocity,
            pivotMotor3Temp);

    inputs.basePivotMotor3Connected = pivot3ConnectedDebouncer.calculate(pivot3Status.isOK());
    inputs.basePivotMotor3AppliedVolts = pivotMotor3AppliedVolts.getValueAsDouble();
    inputs.basePivotMotor3CurrentAmps = pivotMotor3Current.getValueAsDouble();
    inputs.basePivotMotor3PositionRad =
        Units.rotationsToRadians(pivotMotor3Position.getValueAsDouble());
    inputs.basePivotMotor3VelocityRadPerSec =
        Units.rotationsToRadians(pivotMotor3Velocity.getValueAsDouble());
    inputs.basePivotMotor3Temp = pivotMotor3Temp.getValueAsDouble();

    var extension1Status =
        BaseStatusSignal.refreshAll(
            extensionMotor1AppliedVolts,
            extensionMotor1Current,
            extensionMotor1Position,
            extensionMotor1Velocity,
            extensionMotor1Temp);

    inputs.extensionMotor1Connected =
        extension1ConnectedDebouncer.calculate(extension1Status.isOK());
    inputs.extensionMotor1AppliedVolts = extensionMotor1AppliedVolts.getValueAsDouble();
    inputs.extensionMotor1CurrentAmps = extensionMotor1Current.getValueAsDouble();
    inputs.extensionMotor1PositionRad =
        Units.rotationsToRadians(extensionMotor1Position.getValueAsDouble());
    inputs.extensionMotor1VelocityRadPerSec =
        Units.rotationsToRadians(extensionMotor1Velocity.getValueAsDouble());
    inputs.extensionMotor1Temp = extensionMotor1Temp.getValueAsDouble();

    var extension2Status =
        BaseStatusSignal.refreshAll(
            extensionMotor2AppliedVolts,
            extensionMotor2Current,
            extensionMotor2Position,
            extensionMotor2Velocity,
            extensionMotor2Temp);

    inputs.extensionMotor2Connected =
        extension2ConnectedDebouncer.calculate(extension2Status.isOK());
    inputs.extensionMotor2AppliedVolts = extensionMotor2AppliedVolts.getValueAsDouble();
    inputs.extensionMotor2CurrentAmps = extensionMotor2Current.getValueAsDouble();
    inputs.extensionMotor2PositionRad =
        Units.rotationsToRadians(extensionMotor2Position.getValueAsDouble());
    inputs.extensionMotor2VelocityRadPerSec =
        Units.rotationsToRadians(extensionMotor2Velocity.getValueAsDouble());
    inputs.extensionMotor2Temp = extensionMotor2Temp.getValueAsDouble();

    var wristStatus =
        BaseStatusSignal.refreshAll(
            wristAppliedVolts, wristCurrent, wristPosition, wristVelocity, wristTemp);

    inputs.wristMotorConnected = wristConnectedDebouncer.calculate(wristStatus.isOK());
    inputs.wristMotorAppliedVolts = wristAppliedVolts.getValueAsDouble();
    inputs.wristMotorCurrentAmps = wristCurrent.getValueAsDouble();
    inputs.wristMotorPositionRad = Units.rotationsToRadians(wristPosition.getValueAsDouble());
    inputs.wristMotorVelocityRadPerSec = Units.rotationsToRadians(wristVelocity.getValueAsDouble());
    inputs.wristMotorTemp = wristTemp.getValueAsDouble();
  }

  @Override
  public void setPivotVoltage(double volts) {
    double feedforward =
        pivotFeedforward.calculate(
            Units.rotationsToRadians(pivotMotor1Position.getValueAsDouble()), 0);
    pivotMotor1.setVoltage(volts + feedforward);
  }

  @Override
  public void setExtensionVoltage(double volts) {
    extensionMotor1.setVoltage(volts);
  }

  @Override
  public void setWristVoltage(double volts) {
    wristMotor.setVoltage(volts);
  }

  @Override
  public void setAngle(Rotation2d angle) {
    pivotMotor1.setControl(
        switch (pivotControlType) {
          case Voltage -> pivotMotionMagicVoltage.withPosition(angle.getRotations());
          case DutyCyle -> pivotMotionMagicDutyCycle.withPosition(angle.getRotations());
          case TorqueCurrentFOC -> pivotMotionMagicTorqueCurrentFOC.withPosition(
              angle.getRotations());
        });
  }

  @Override
  public void setLength(double length) {

    // radius * radians = distance
    Rotation2d driveRotations = new Rotation2d(length / driveDiameter);

    extensionMotor1.setControl(
        switch (extensionControlType) {
          case Voltage -> extensionMotionMagicVoltage.withPosition(driveRotations.getRotations());
          case DutyCyle -> extensionMotionMagicDutyCycle.withPosition(
              driveRotations.getRotations());
          case TorqueCurrentFOC -> extensionMotionMagicTorqueCurrentFOC.withPosition(
              driveRotations.getRotations());
        });
  }

  @Override
  public void setWrist(Rotation2d angle) {
    wristMotor.setControl(
        switch (wristControlType) {
          case Voltage -> wristMotionMagicVoltage.withPosition(angle.getRotations());
          case DutyCyle -> wristMotionMagicDutyCycle.withPosition(angle.getRotations());
          case TorqueCurrentFOC -> wristMotionMagicTorqueCurrentFOC.withPosition(
              angle.getRotations());
        });
  }
}
