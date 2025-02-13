package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface ArmIO {
    @AutoLog
    public static class ArmIOInputs {
        public boolean basePivotMotor1Connected = false;
        public double basePivotMotor1PositionRad = 0.0;
        public double basePivotMotor1VelocityRadPerSec = 0.0;
        public double basePivotMotor1AppliedVolts = 0.0;
        public double basePivotMotor1CurrentAmps = 0.0;
        public double basePivotMotor1Temp = 0.0;

        public boolean basePivotMotor2Connected = false;
        public double basePivotMotor2PositionRad = 0.0;
        public double basePivotMotor2VelocityRadPerSec = 0.0;
        public double basePivotMotor2AppliedVolts = 0.0;
        public double basePivotMotor2CurrentAmps = 0.0;
        public double basePivotMotor2Temp = 0.0;

        public boolean basePivotMotor3Connected = false;
        public double basePivotMotor3PositionRad = 0.0;
        public double basePivotMotor3VelocityRadPerSec = 0.0;
        public double basePivotMotor3AppliedVolts = 0.0;
        public double basePivotMotor3CurrentAmps = 0.0;
        public double basePivotMotor3Temp = 0.0;

        public boolean extensionMotor1Connected = false;
        public double extensionMotor1PositionRad = 0.0;
        public double extensionMotor1VelocityRadPerSec = 0.0;
        public double extensionMotor1AppliedVolts = 0.0;
        public double extensionMotor1CurrentAmps = 0.0;
        public double extensionMotor1Temp = 0.0;

        public boolean extensionMotor2Connected = false;
        public double extensionMotor2PositionRad = 0.0;
        public double extensionMotor2VelocityRadPerSec = 0.0;
        public double extensionMotor2AppliedVolts = 0.0;
        public double extensionMotor2CurrentAmps = 0.0;
        public double extensionMotor2Temp = 0.0;

        public boolean wristMotorConnected = false;
        public double wristMotorPositionRad = 0.0;
        public double wristMotorVelocityRadPerSec = 0.0;
        public double wristMotorAppliedVolts = 0.0;
        public double wristMotorCurrentAmps = 0.0;
        public double wristMotorTemp = 0.0;
    }

    public default void updateInputs(ArmIOInputs inputs) {
    }

    public default void setAngle(Rotation2d angle) {
    }

    public default void setPivotVoltage(double volts){
    }

    public default void setLength(double length) {
    }

    public default void setExtensionVoltage(double volts){
    }

    public default void setWrist(Rotation2d angle) {
    }

    public default void setWristVoltage(double volts){
    }
}