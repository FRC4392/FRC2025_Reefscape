// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Inch;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

/** Add your docs here. */
public class ArmStateData {
    private Angle pivotAngle;
    private Angle wristAngle;
    private Distance extensionDistance;

    public ArmStateData(){
        pivotAngle = Degree.of(0);
        extensionDistance = Inch.of(0);
        wristAngle = Degree.of(0);
    }

    public ArmStateData(Angle pivot, Distance extension, Angle wrist){
        pivotAngle = pivot;
        extensionDistance = extension;
        wristAngle = wrist;
    }

    public Angle getPivot(){
        return pivotAngle;
    }

    public Distance getExtension(){
        return extensionDistance;
    }

    public Angle getWrist(){
        return wristAngle;
    }
}
