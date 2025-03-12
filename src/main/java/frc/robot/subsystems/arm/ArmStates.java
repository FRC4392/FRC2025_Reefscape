// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

/** Add your docs here. */
public enum ArmStates {
    START(new ArmStateData()),
    HOME(new ArmStateData()),
    L1(new ArmStateData()),
    L2(new ArmStateData()),
    L3(new ArmStateData()),
    L4(new ArmStateData()),
    ALGAE1(new ArmStateData()),
    ALGAE2(new ArmStateData()),
    BARGE(new ArmStateData()),
    CLIMB(new ArmStateData()),
    CoralIntake(new ArmStateData()),
    AlgaeIntake(new ArmStateData()),
    PROCESSOR(new ArmStateData()),
    UpTravel(new ArmStateData()),
    DownTravel(new ArmStateData());

    private ArmStates(ArmStateData data){
        this.data = data;
    }

    public ArmStateData getStateData(){
        return data;
    }

    private final ArmStateData data;
}
