// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

/** Add your docs here. */
public enum ArmState {
  START(new ArmStateData(
    Degrees.of(-22), 
    Inches.of(0), 
    Degrees.of(0))),
  HOME(new ArmStateData(
    Degrees.of(-22), 
    Inches.of(0), 
    Degrees.of(90))),
  L1(new ArmStateData(
    Degrees.of(100), 
    Inches.of(0), 
    Degrees.of(0))),
  L2(new ArmStateData(
    Degrees.of(90), 
    Inches.of(0), 
    Degrees.of(0))),
  L3(new ArmStateData(
    Degrees.of(80), 
    Inches.of(4.5), 
    Degrees.of(10))),
  L4(new ArmStateData(
    Degrees.of(80), 
    Inches.of(15), 
    Degrees.of(15))),
  ALGAE1(new ArmStateData(
    Degrees.of(100), 
    Inches.of(0), 
    Degrees.of(5))),
  ALGAE2(new ArmStateData(
    Degrees.of(90), 
    Inches.of(7), 
    Degrees.of(10))),
  BARGE(new ArmStateData(
    Degrees.of(70), 
    Inches.of(17), 
    Degrees.of(115))),
  CLIMB(new ArmStateData(
    Degrees.of(90), 
    Inches.of(0), 
    Degrees.of(90))),
  CoralIntake(new ArmStateData(
    Degrees.of(0), 
    Inches.of(0), 
    Degrees.of(0))),
  AlgaeIntake(new ArmStateData(
    Degrees.of(0), 
    Inches.of(0), 
    Degrees.of(-5))),
  PROCESSOR(new ArmStateData(
    Degrees.of(0), 
    Inches.of(0), 
    Degrees.of(-5))),
  UpTravel(new ArmStateData(
    Degrees.of(70), 
    Inches.of(0), 
    Degrees.of(90))),
  DownTravel(new ArmStateData(
    Degrees.of(0), 
    Inches.of(0), 
    Degrees.of(90)));

  private ArmState(ArmStateData data) {
    this.data = data;
  }

  public ArmStateData getStateData() {
    return data;
  }

  private final ArmStateData data;
}
