// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
  /** Inputs from vision system */
  @AutoLog
  public static class VisionIOInputs {
    public boolean connected = false;
    public TargetObservation latestTargetObservation =
        new TargetObservation(new Rotation2d(), new Rotation2d(), 0);
    public PoseObservation[] poseObservations = new PoseObservation[0];
    public targetPoseObservation lastTargetPoseObservation =
        new targetPoseObservation(new Pose3d(), 0);
    public int[] tagIds = new int[0];
  }

  /** Represents the angle to a simple target, not used for pose estimation. */
  public static record TargetObservation(Rotation2d tx, Rotation2d ty, double tid) {}

  /** Represents the targets pose in robot space, not used for pose estimation */
  public static record targetPoseObservation(Pose3d targetPose, int targetID) {}

  /** Represents a robot pose sample used for pose estimation. */
  public static record PoseObservation(
      double timestamp,
      Pose3d pose,
      double ambiguity,
      int tagCount,
      double averageTagDistance,
      PoseObservationType type) {}

  /** Possible pose observation types */
  public static enum PoseObservationType {
    MEGATAG_1,
    MEGATAG_2,
    PHOTONVISION
  }

  /**
   * Updates the set of loggable inputs.
   *
   * @param inputs Instance of the vision inputs
   */
  public default void updateInputs(VisionIOInputs inputs) {}
}
