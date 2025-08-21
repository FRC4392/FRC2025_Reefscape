// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.gripper;

import static frc.robot.subsystems.gripper.GripperConstants.GripperReduction;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Add your docs here. */
public class GripperIOSIm implements GripperIO {

  DCMotor gripperMotor = DCMotor.getKrakenX60Foc(1);
  DCMotorSim gripperMotorSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(gripperMotor, 0.004, GripperReduction), gripperMotor);

  double appliedVolts = 0.0;

  @Override
  public void updateInputs(GripperIOInputs inputs) {
    gripperMotorSim.setInputVoltage(appliedVolts);

    gripperMotorSim.update(0.02);

    inputs.motorConnected = true;
    inputs.motorAppliedVolts = appliedVolts;
    inputs.motorCurrentAmps = gripperMotorSim.getCurrentDrawAmps();
    inputs.motorPositionRad = gripperMotorSim.getAngularPositionRad();
    inputs.motorVelocityRadPerSec = gripperMotorSim.getAngularVelocityRadPerSec();
    inputs.motorTemp = 0.0;
  }

  @Override
  public void setGripperVoltage(double voltage) {
    appliedVolts = voltage;
  }
}
