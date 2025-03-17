// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import static frc.robot.subsystems.arm.ArmConstants.driveRadius;
import static frc.robot.subsystems.arm.ArmConstants.minAngle;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {

  public static enum ArmPosition {
    HOME,
    L1,
    L2,
    L3,
    L4,
    ALGAE1,
    ALGAE2,
    BARGE,
    CLIMB,
    INTAKE,
    PROCESSOR,
    UNKNOWN;

    public Rotation2d pivot() {
      switch (this) {
        case HOME:
          return minAngle;
        case L1:
          return Rotation2d.fromDegrees(90);
        case L2:
          return Rotation2d.fromDegrees(90);
        case L3:
          return Rotation2d.fromDegrees(90);
        case L4:
          return Rotation2d.fromDegrees(88);
        case ALGAE1:
          return Rotation2d.fromDegrees(100);
        case ALGAE2:
          return Rotation2d.fromDegrees(90);
        case BARGE:
          return Rotation2d.fromDegrees(70);
        case CLIMB:
          return Rotation2d.fromDegrees(90);
        case INTAKE:
          return Rotation2d.fromDegrees(25);
        case PROCESSOR:
          return minAngle.plus(Rotation2d.fromDegrees(14));
        default:
          return new Rotation2d();
      }
    }

    public double extension() {
      switch (this) {
        case HOME:
          return Units.inchesToMeters(0);
        case L1:
          return Units.inchesToMeters(0);
        case L2:
          return Units.inchesToMeters(0);
        case L3:
          return Units.inchesToMeters(3);
        case L4:
          return Units.inchesToMeters(15);
        case ALGAE1:
          return Units.inchesToMeters(0);
        case ALGAE2:
          return Units.inchesToMeters(7);
        case BARGE:
          return Units.inchesToMeters(17);
        case CLIMB:
          return Units.inchesToMeters(0);
        case INTAKE:
          return Units.inchesToMeters(0);
        case PROCESSOR:
          return Units.inchesToMeters(2);
        default:
          return Units.inchesToMeters(0);
      }
    }

    public Rotation2d wrist() {
      switch (this) {
        case HOME:
          return new Rotation2d();
        case L1:
          return Rotation2d.fromDegrees(0);
        case L2:
          return Rotation2d.fromDegrees(30);
        case L3:
          return Rotation2d.fromDegrees(30);
        case L4:
          return Rotation2d.fromDegrees(30);
        case ALGAE1:
          return Rotation2d.fromDegrees(5);
        case ALGAE2:
          return Rotation2d.fromDegrees(10);
        case BARGE:
          return Rotation2d.fromDegrees(115);
        case CLIMB:
          return Rotation2d.fromDegrees(90);
        case INTAKE:
          return Rotation2d.fromDegrees(90);
        case PROCESSOR:
          return Rotation2d.fromDegrees(180);
        default:
          return new Rotation2d();
      }
    }
  }

  public enum ArmState {
    TRAVELING,
    INPOSITION,
    UNKNOWN;
  }

  private final ArmIO armIO;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  private ArmPosition targetPosition = ArmPosition.HOME;
  private ArmPosition currentPosition = ArmPosition.HOME;
  private ArmState armState = ArmState.INPOSITION;

  private Rotation2d pivotSetpoint = minAngle;
  private double extensionSetPoint = 0.0;
  private Rotation2d wristSetpoint = new Rotation2d();

  private final Alert pivot1DisconnectedAlert =
      new Alert("Pivot Motor 1 Disconnected, arm may fail to work", AlertType.kError);
  private final Alert pivot2DisconnectedAlert =
      new Alert("Pivot Motor 2 Disconnected, arm may have reduced performance", AlertType.kError);
  private final Alert pivot3DisconnectedAlert =
      new Alert("Pivot Motor 3 Disconnected, arm may have reduced performance", AlertType.kError);

  private final Alert extension1DisconnectedAlert =
      new Alert("Extension Motor 1 Disconnected, extension may fail to work", AlertType.kError);
  private final Alert extension2DisconnectedAlert =
      new Alert(
          "Extension Motor 2 Disconnected, extension may have reduced performance",
          AlertType.kError);

  private final Alert wristDisconnectedAlert =
      new Alert("Wrist Motor Disconnected, wrist may fail to work", AlertType.kError);

  /** Creates a new Arm. */
  public Arm(ArmIO armIO) {
    this.armIO = armIO;
  }

  @Override
  public void periodic() {
    armIO.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);

    pivot1DisconnectedAlert.set(!inputs.basePivotMotor1Connected);
    pivot2DisconnectedAlert.set(!inputs.basePivotMotor2Connected);
    pivot3DisconnectedAlert.set(!inputs.basePivotMotor3Connected);

    extension1DisconnectedAlert.set(!inputs.extensionMotor1Connected);
    extension2DisconnectedAlert.set(!inputs.extensionMotor2Connected);

    wristDisconnectedAlert.set(!inputs.wristMotorConnected);
  }

  public void setPosition(Rotation2d pivotRotation, double length, Rotation2d wristAngle) {
    pivotSetpoint = pivotRotation;
    extensionSetPoint = length;
    wristSetpoint = wristAngle;
    armIO.setAngle(pivotRotation);
    armIO.setLength(length);
    armIO.setWrist(wristAngle);
  }

  public void setPosition(ArmPosition position) {
    currentPosition = position;
    armIO.setAngle(position.pivot());
    armIO.setLength(position.extension());
    armIO.setWrist(position.wrist());
  }

  public void setPivotPosition(Rotation2d pivotRotation) {
    pivotSetpoint = pivotRotation;
    armIO.setAngle(pivotRotation);
  }

  public void setExtensionDistance(double lengthMeters) {
    extensionSetPoint = lengthMeters;
    armIO.setLength(lengthMeters);
  }

  public void setWristAngle(Rotation2d wristAngle) {
    wristSetpoint = wristAngle;
    armIO.setWrist(wristAngle);
  }

  public void setPivotVoltage(double voltage) {
    armIO.setPivotVoltage(voltage);
  }

  public void setExtensionVoltage(double voltage) {
    armIO.setExtensionVoltage(voltage);
  }

  public void setWristVoltage(double voltage) {
    armIO.setWristVoltage(voltage);
  }

  @AutoLogOutput
  public boolean getPivotInPosition() {
    return Math.abs(inputs.basePivotMotor1PositionRad - pivotSetpoint.getRadians())
        < Units.degreesToRadians(2);
  }

  @AutoLogOutput
  public boolean getExtensionInPosition() {
    return Math.abs((inputs.extensionMotor2PositionRad * driveRadius) - extensionSetPoint) < 1;
  }

  @AutoLogOutput
  public boolean getWristInPosition() {
    return Math.abs(inputs.wristMotorPositionRad - wristSetpoint.getRadians())
        < Units.degreesToRadians(2);
  }

  @AutoLogOutput
  public Rotation2d getPivotSetpoint() {
    return pivotSetpoint;
  }

  @AutoLogOutput
  public double getExtensionSetpoitn() {
    return extensionSetPoint;
  }

  @AutoLogOutput
  public Rotation2d getWristSetpoint() {
    return wristSetpoint;
  }

  public boolean getArmInPosition() {
    return getExtensionInPosition() && getPivotInPosition() && getWristInPosition();
  }

  public ArmPosition getArmSetPosition() {
    return currentPosition;
  }

  public void setArmPostion(ArmPosition destinationPosition) {
    if (this.getCurrentCommand() != null) {
      this.getCurrentCommand().cancel();
    }
    switch (destinationPosition) {
      case HOME:
        if (currentPosition == ArmPosition.INTAKE || currentPosition == ArmPosition.PROCESSOR) {
          Commands.sequence(
                  this.run(
                          () ->
                              setPosition(
                                  currentPosition.pivot(),
                                  destinationPosition.extension(),
                                  destinationPosition.wrist()))
                      .until(() -> getArmInPosition()),
                  this.run(() -> setPosition(destinationPosition)))
              .schedule();
        } else if ((currentPosition != ArmPosition.HOME)) {
          Commands.sequence(
                  this.run(
                          () ->
                              setPosition(
                                  Rotation2d.fromDegrees(70),
                                  currentPosition.extension(),
                                  currentPosition.wrist()))
                      .until(() -> getArmInPosition()),
                  this.run(
                          () ->
                              setPosition(
                                  Rotation2d.fromDegrees(70),
                                  destinationPosition.extension(),
                                  destinationPosition.wrist()))
                      .until(() -> getArmInPosition()),
                  this.run(() -> setPosition(destinationPosition)))
              .schedule();
        }
        break;
      case L1:
        switch (currentPosition) {
          case L2:
          case L3:
          case L4:
          case ALGAE1:
          case ALGAE2:
          case HOME:
          case CLIMB:
          case BARGE:
            Commands.sequence(
                    this.run(
                            () ->
                                setPosition(
                                    Rotation2d.fromDegrees(70),
                                    currentPosition.extension(),
                                    currentPosition.wrist()))
                        .until(() -> getArmInPosition()),
                    this.run(
                            () ->
                                setPosition(
                                    Rotation2d.fromDegrees(70),
                                    destinationPosition.extension(),
                                    destinationPosition.wrist()))
                        .until(() -> getArmInPosition()),
                    this.run(() -> setPosition(destinationPosition)))
                .schedule();
            break;
          case INTAKE:
          case PROCESSOR:
            Commands.sequence(
                    this.run(
                            () ->
                                setPosition(
                                    Rotation2d.fromDegrees(70),
                                    currentPosition.extension(),
                                    destinationPosition.wrist()))
                        .until(() -> getArmInPosition()),
                    this.run(
                            () ->
                                setPosition(
                                    Rotation2d.fromDegrees(70),
                                    destinationPosition.extension(),
                                    destinationPosition.wrist()))
                        .until(() -> getArmInPosition()),
                    this.run(() -> setPosition(destinationPosition)))
                .schedule();
            break;
          case L1:
            this.run(() -> setPosition(destinationPosition));
            break;
          case UNKNOWN:
            break;
        }
        break;
      case L2:
        switch (currentPosition) {
          case L1:
          case L3:
          case L4:
          case ALGAE1:
          case ALGAE2:
          case HOME:
          case CLIMB:
          case BARGE:
            Commands.sequence(
                    this.run(
                            () ->
                                setPosition(
                                    Rotation2d.fromDegrees(70),
                                    currentPosition.extension(),
                                    currentPosition.wrist()))
                        .until(() -> getArmInPosition()),
                    this.run(
                            () ->
                                setPosition(
                                    Rotation2d.fromDegrees(70),
                                    destinationPosition.extension(),
                                    destinationPosition.wrist()))
                        .until(() -> getArmInPosition()),
                    this.run(() -> setPosition(destinationPosition)))
                .schedule();
            break;
          case INTAKE:
          case PROCESSOR:
            Commands.sequence(
                    this.run(
                            () ->
                                setPosition(
                                    Rotation2d.fromDegrees(70),
                                    currentPosition.extension(),
                                    destinationPosition.wrist()))
                        .until(() -> getArmInPosition()),
                    this.run(
                            () ->
                                setPosition(
                                    Rotation2d.fromDegrees(70),
                                    destinationPosition.extension(),
                                    destinationPosition.wrist()))
                        .until(() -> getArmInPosition()),
                    this.run(() -> setPosition(destinationPosition)))
                .schedule();
            break;
          case L2:
            this.run(() -> setPosition(destinationPosition));
            break;
          case UNKNOWN:
            break;
        }
        break;
      case L3:
        switch (currentPosition) {
          case L1:
          case L2:
          case L4:
          case ALGAE1:
          case ALGAE2:
          case HOME:
          case CLIMB:
          case BARGE:
            Commands.sequence(
                    this.run(
                            () ->
                                setPosition(
                                    Rotation2d.fromDegrees(70),
                                    currentPosition.extension(),
                                    currentPosition.wrist()))
                        .until(() -> getArmInPosition()),
                    this.run(
                            () ->
                                setPosition(
                                    Rotation2d.fromDegrees(70),
                                    destinationPosition.extension(),
                                    destinationPosition.wrist()))
                        .until(() -> getArmInPosition()),
                    this.run(() -> setPosition(destinationPosition)))
                .schedule();
            break;
          case INTAKE:
          case PROCESSOR:
            Commands.sequence(
                    this.run(
                            () ->
                                setPosition(
                                    Rotation2d.fromDegrees(70),
                                    currentPosition.extension(),
                                    destinationPosition.wrist()))
                        .until(() -> getArmInPosition()),
                    this.run(
                            () ->
                                setPosition(
                                    Rotation2d.fromDegrees(70),
                                    destinationPosition.extension(),
                                    destinationPosition.wrist()))
                        .until(() -> getArmInPosition()),
                    this.run(() -> setPosition(destinationPosition)))
                .schedule();
            break;
          case L3:
            this.run(() -> setPosition(destinationPosition));
            break;
          case UNKNOWN:
            break;
        }
        break;
      case L4:
        switch (currentPosition) {
          case L1:
          case L2:
          case L3:
          case ALGAE1:
          case ALGAE2:
          case HOME:
          case CLIMB:
          case BARGE:
            Commands.sequence(
                    this.run(
                            () ->
                                setPosition(
                                    Rotation2d.fromDegrees(70),
                                    currentPosition.extension(),
                                    currentPosition.wrist()))
                        .until(() -> getArmInPosition()),
                    this.run(
                            () ->
                                setPosition(
                                    Rotation2d.fromDegrees(70),
                                    destinationPosition.extension(),
                                    destinationPosition.wrist()))
                        .until(() -> getArmInPosition()),
                    this.run(() -> setPosition(destinationPosition)))
                .schedule();
            break;
          case INTAKE:
          case PROCESSOR:
            Commands.sequence(
                    this.run(
                            () ->
                                setPosition(
                                    Rotation2d.fromDegrees(70),
                                    currentPosition.extension(),
                                    destinationPosition.wrist()))
                        .until(() -> getArmInPosition()),
                    this.run(
                            () ->
                                setPosition(
                                    Rotation2d.fromDegrees(70),
                                    destinationPosition.extension(),
                                    destinationPosition.wrist()))
                        .until(() -> getArmInPosition()),
                    this.run(() -> setPosition(destinationPosition)))
                .schedule();
            break;
          case L4:
            this.run(() -> setPosition(destinationPosition));
            break;
          case UNKNOWN:
            break;
        }
        break;
      case ALGAE1:
        switch (currentPosition) {
          case L1:
          case L2:
          case L3:
          case L4:
          case ALGAE2:
          case HOME:
          case CLIMB:
          case BARGE:
            Commands.sequence(
                    this.run(
                            () ->
                                setPosition(
                                    Rotation2d.fromDegrees(70),
                                    currentPosition.extension(),
                                    currentPosition.wrist()))
                        .until(() -> getArmInPosition()),
                    this.run(
                            () ->
                                setPosition(
                                    Rotation2d.fromDegrees(70),
                                    destinationPosition.extension(),
                                    destinationPosition.wrist()))
                        .until(() -> getArmInPosition()),
                    this.run(() -> setPosition(destinationPosition)))
                .schedule();
            break;
          case INTAKE:
          case PROCESSOR:
            Commands.sequence(
                    this.run(
                            () ->
                                setPosition(
                                    Rotation2d.fromDegrees(70),
                                    currentPosition.extension(),
                                    destinationPosition.wrist()))
                        .until(() -> getArmInPosition()),
                    this.run(
                            () ->
                                setPosition(
                                    Rotation2d.fromDegrees(70),
                                    destinationPosition.extension(),
                                    destinationPosition.wrist()))
                        .until(() -> getArmInPosition()),
                    this.run(() -> setPosition(destinationPosition)))
                .schedule();
            break;
          case ALGAE1:
            this.run(() -> setPosition(destinationPosition));
            break;
          case UNKNOWN:
            break;
        }
        break;
      case ALGAE2:
        switch (currentPosition) {
          case L1:
          case L2:
          case L3:
          case L4:
          case ALGAE1:
          case HOME:
          case CLIMB:
          case BARGE:
            Commands.sequence(
                    this.run(
                            () ->
                                setPosition(
                                    Rotation2d.fromDegrees(70),
                                    currentPosition.extension(),
                                    currentPosition.wrist()))
                        .until(() -> getArmInPosition()),
                    this.run(
                            () ->
                                setPosition(
                                    Rotation2d.fromDegrees(70),
                                    destinationPosition.extension(),
                                    destinationPosition.wrist()))
                        .until(() -> getArmInPosition()),
                    this.run(() -> setPosition(destinationPosition)))
                .schedule();
            break;
          case INTAKE:
          case PROCESSOR:
            Commands.sequence(
                    this.run(
                            () ->
                                setPosition(
                                    Rotation2d.fromDegrees(70),
                                    currentPosition.extension(),
                                    destinationPosition.wrist()))
                        .until(() -> getArmInPosition()),
                    this.run(
                            () ->
                                setPosition(
                                    Rotation2d.fromDegrees(70),
                                    destinationPosition.extension(),
                                    destinationPosition.wrist()))
                        .until(() -> getArmInPosition()),
                    this.run(() -> setPosition(destinationPosition)))
                .schedule();
            break;
          case ALGAE2:
            this.run(() -> setPosition(destinationPosition));
            break;
          case UNKNOWN:
            break;
        }
        break;
      case BARGE:
        switch (currentPosition) {
          case L1:
          case L2:
          case L3:
          case L4:
          case ALGAE1:
          case ALGAE2:
          case CLIMB:
          case HOME:
            Commands.sequence(
                    this.run(
                            () ->
                                setPosition(
                                    Rotation2d.fromDegrees(70),
                                    currentPosition.extension(),
                                    currentPosition.wrist()))
                        .until(() -> getArmInPosition()),
                    this.run(
                            () ->
                                setPosition(
                                    Rotation2d.fromDegrees(70),
                                    destinationPosition.extension(),
                                    destinationPosition.wrist()))
                        .until(() -> getArmInPosition()),
                    this.run(() -> setPosition(destinationPosition)))
                .schedule();
            break;
          case INTAKE:
          case PROCESSOR:
            Commands.sequence(
                    this.run(
                            () ->
                                setPosition(
                                    Rotation2d.fromDegrees(70),
                                    currentPosition.extension(),
                                    destinationPosition.wrist()))
                        .until(() -> getArmInPosition()),
                    this.run(
                            () ->
                                setPosition(
                                    Rotation2d.fromDegrees(70),
                                    destinationPosition.extension(),
                                    destinationPosition.wrist()))
                        .until(() -> getArmInPosition()),
                    this.run(() -> setPosition(destinationPosition)))
                .schedule();
            break;
          case BARGE:
            this.run(() -> setPosition(destinationPosition));
            break;
          case UNKNOWN:
            break;
        }
        break;
      case CLIMB:
        switch (currentPosition) {
          case L1:
          case L2:
          case L3:
          case L4:
          case ALGAE1:
          case ALGAE2:
          case BARGE:
          case HOME:
            Commands.sequence(
                    this.run(
                            () ->
                                setPosition(
                                    Rotation2d.fromDegrees(70),
                                    currentPosition.extension(),
                                    currentPosition.wrist()))
                        .until(() -> getArmInPosition()),
                    this.run(
                            () ->
                                setPosition(
                                    Rotation2d.fromDegrees(70),
                                    destinationPosition.extension(),
                                    destinationPosition.wrist()))
                        .until(() -> getArmInPosition()),
                    this.run(() -> setPosition(destinationPosition)))
                .schedule();
            break;
          case INTAKE:
          case PROCESSOR:
            Commands.sequence(
                    this.run(
                            () ->
                                setPosition(
                                    Rotation2d.fromDegrees(70),
                                    currentPosition.extension(),
                                    destinationPosition.wrist()))
                        .until(() -> getArmInPosition()),
                    this.run(
                            () ->
                                setPosition(
                                    Rotation2d.fromDegrees(70),
                                    destinationPosition.extension(),
                                    destinationPosition.wrist()))
                        .until(() -> getArmInPosition()),
                    this.run(() -> setPosition(destinationPosition)))
                .schedule();
            break;
          case CLIMB:
            this.run(() -> setPosition(destinationPosition));
            break;
          case UNKNOWN:
            break;
        }
        break;
      case INTAKE:
        switch (currentPosition) {
          case L1:
          case L2:
          case L3:
          case L4:
          case ALGAE1:
          case ALGAE2:
          case BARGE:
          case CLIMB:
            Commands.sequence(
                    this.run(
                            () ->
                                setPosition(
                                    Rotation2d.fromDegrees(70),
                                    currentPosition.extension(),
                                    currentPosition.wrist()))
                        .until(() -> getArmInPosition()),
                    this.run(
                            () ->
                                setPosition(
                                    Rotation2d.fromDegrees(70),
                                    destinationPosition.extension(),
                                    destinationPosition.wrist()))
                        .until(() -> getArmInPosition()),
                    this.run(() -> setPosition(destinationPosition)))
                .schedule();
            break;
          case INTAKE:
          case PROCESSOR:
            this.run(() -> setPosition(destinationPosition)).schedule();
            break;
          case HOME:
            this.run(() -> setPosition(destinationPosition)).schedule();
            break;
          case UNKNOWN:
            break;
        }
        break;
      case PROCESSOR:
        switch (currentPosition) {
          case L1:
          case L2:
          case L3:
          case L4:
          case ALGAE1:
          case ALGAE2:
          case BARGE:
          case CLIMB:
            Commands.sequence(
                    this.run(
                            () ->
                                setPosition(
                                    Rotation2d.fromDegrees(70),
                                    currentPosition.extension(),
                                    currentPosition.wrist()))
                        .until(() -> getArmInPosition()),
                    this.run(
                            () ->
                                setPosition(
                                    Rotation2d.fromDegrees(70),
                                    destinationPosition.extension(),
                                    destinationPosition.wrist()))
                        .until(() -> getArmInPosition()),
                    this.run(() -> setPosition(destinationPosition)))
                .schedule();
            break;
          case INTAKE:
          case PROCESSOR:
            this.run(() -> setPosition(destinationPosition)).schedule();
            break;
          case HOME:
            this.run(() -> setPosition(destinationPosition)).schedule();
            break;
          case UNKNOWN:
            break;
        }
        break;
      default:
        break;
    }
  }

  public Command getDropOffLowCommand() {
    ArmPosition destinationPosition = ArmPosition.L2;
    return Commands.sequence(
        this.run(
                () ->
                    setPosition(
                        Rotation2d.fromDegrees(70),
                        currentPosition.extension(),
                        destinationPosition.wrist()))
            .until(() -> getArmInPosition()),
        this.run(
                () ->
                    setPosition(
                        Rotation2d.fromDegrees(70),
                        destinationPosition.extension(),
                        destinationPosition.wrist()))
            .until(() -> getArmInPosition()),
        this.run(() -> setPosition(destinationPosition)).until(() -> getArmInPosition()));
  }

  public Command getL3ArmCommand() {
    ArmPosition destinationPosition = ArmPosition.L3;
    return Commands.sequence(
        this.run(() -> setPosition(Rotation2d.fromDegrees(70), 0, new Rotation2d()))
            .until(() -> getArmInPosition()),
        this.run(
                () ->
                    setPosition(
                        Rotation2d.fromDegrees(70),
                        destinationPosition.extension(),
                        destinationPosition.wrist()))
            .until(() -> getArmInPosition()),
        this.run(() -> setPosition(destinationPosition)).until(() -> getArmInPosition()));
  }

  public Command getL4ArmCommand() {
    ArmPosition destinationPosition = ArmPosition.L4;
    return Commands.sequence(
        this.run(() -> setPosition(Rotation2d.fromDegrees(70), 0, new Rotation2d()))
            .until(() -> getArmInPosition()),
        this.run(
                () ->
                    setPosition(
                        Rotation2d.fromDegrees(70),
                        destinationPosition.extension(),
                        destinationPosition.wrist()))
            .until(() -> getArmInPosition()),
        this.run(() -> setPosition(destinationPosition)).until(() -> getArmInPosition()));
  }

  public Command getL4ScoreCommand() {
    ArmPosition destinationPosition = ArmPosition.L4;
    return Commands.sequence(
        this.run(
                () ->
                    setPosition(
                        destinationPosition.pivot(),
                        destinationPosition.extension(),
                        destinationPosition.wrist().plus(Rotation2d.fromDegrees(20))))
            .until(() -> getArmInPosition()),
        this.run(
                () ->
                    setPosition(
                        Rotation2d.fromDegrees(70),
                        currentPosition.extension(),
                        currentPosition.wrist()))
            .until(() -> getArmInPosition()),
        this.run(
                () ->
                    setPosition(
                        Rotation2d.fromDegrees(70),
                        ArmPosition.INTAKE.extension(),
                        ArmPosition.INTAKE.wrist()))
            .until(() -> getArmInPosition()),
        this.run(() -> setPosition(ArmPosition.INTAKE)));
  }
}
