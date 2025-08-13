// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.gripper;

import static frc.robot.subsystems.gripper.GripperConstants.*;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;

/** Add your docs here. */
public class GripperIOTalon implements GripperIO {

  private final TalonFX motor = new TalonFX(AlgaeCanId);

  private final Debouncer motorConnectedDebounce = new Debouncer(.5);

  private final DigitalInput coralPresent = new DigitalInput(coralSensorPort);
  private final DigitalInput algaePresent = new DigitalInput(algaeSensorPort);

  private final StatusSignal<Angle> motorPosition;
  private final StatusSignal<AngularVelocity> motorVelocity;
  private final StatusSignal<Voltage> motorVolts;
  private final StatusSignal<Current> motorCurrent;
  private final StatusSignal<Temperature> motorTemp;

  public GripperIOTalon() {
    tryUntilOk(5, () -> motor.getConfigurator().apply(driveConfiguration, .25));

    motorPosition = motor.getPosition();
    motorVelocity = motor.getVelocity();
    motorVolts = motor.getMotorVoltage();
    motorCurrent = motor.getStatorCurrent();
    motorTemp = motor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50, motorPosition, motorVelocity, motorVolts, motorCurrent, motorTemp);
    motor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(GripperIOInputs inputs) {
    var status =
        BaseStatusSignal.refreshAll(
            motorPosition, motorVelocity, motorVolts, motorCurrent, motorTemp);

    inputs.algaeMotorPositionRad = Units.rotationsToRadians(motorPosition.getValueAsDouble());
    inputs.algaeMotorVelocityRadPerSec = Units.rotationsToRadians(motorVelocity.getValueAsDouble());
    inputs.algaeMotorAppliedVolts = motorVolts.getValueAsDouble();
    inputs.algaeMotorCurrentAmps = motorCurrent.getValueAsDouble();
    inputs.algaeMotorTemp = motorTemp.getValueAsDouble();

    inputs.algaeMotorConnected = motorConnectedDebounce.calculate(status.isOK());

    inputs.coralPresent = getCoralPresent();
    inputs.algaePresent = getAlgaePresent();
  }

  @Override
  public void setAlgaeMotorVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  @Override
  public void setCoralMotorVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  @Override
  public boolean getCoralPresent() {
    return !coralPresent.get();
  }

  @Override
  public boolean getAlgaePresent() {
    return !algaePresent.get();
  }
}
