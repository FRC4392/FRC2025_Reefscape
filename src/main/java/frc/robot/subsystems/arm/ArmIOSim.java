// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import static frc.robot.subsystems.arm.ArmConstants.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/** Add your docs here. */
public class ArmIOSim implements ArmIO {
    private final SingleJointedArmSim pivotSim;

    private double pivotSetVoltage = 0.0;

    public ArmIOSim() {
        pivotSim = new SingleJointedArmSim(pivotGearbox, pivotGearReduction, 1.0, 1.0, Units.degreesToRadians(-29.0),
                Units.degreesToRadians(120), true, Units.degreesToRadians(-29));
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        pivotSim.setInputVoltage(pivotSetVoltage);

        pivotSim.update(0.020);

        inputs.basePivotMotor1AppliedVolts = pivotSetVoltage;
        inputs.basePivotMotor1CurrentAmps = pivotSim.getCurrentDrawAmps();
        inputs.basePivotMotor1PositionRad = pivotSim.getAngleRads();
        inputs.basePivotMotor1VelocityRadPerSec = pivotSim.getVelocityRadPerSec();

        inputs.basePivotMotor2AppliedVolts = pivotSetVoltage;
        inputs.basePivotMotor2CurrentAmps = pivotSim.getCurrentDrawAmps();
        inputs.basePivotMotor2PositionRad = pivotSim.getAngleRads();
        inputs.basePivotMotor2VelocityRadPerSec = pivotSim.getVelocityRadPerSec();

        inputs.basePivotMotor3AppliedVolts = pivotSetVoltage;
        inputs.basePivotMotor3CurrentAmps = pivotSim.getCurrentDrawAmps();
        inputs.basePivotMotor3PositionRad = pivotSim.getAngleRads();
        inputs.basePivotMotor3VelocityRadPerSec = pivotSim.getVelocityRadPerSec();
    }

    @Override
    public void setAngle(Rotation2d angle) {
    }

    @Override
    public void setPivotVoltage(double volts) {
        pivotSetVoltage = volts;
    }

    @Override
    public void setLength(double length) {
    }

    @Override
    public void setExtensionVoltage(double volts) {
        
    }

    @Override
    public void setWrist(Rotation2d angle) {
    }

    @Override
    public void setWristVoltage(double volts) {
    }

}
