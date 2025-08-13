package frc.robot.subsystems.swerve;

/** Represents the different control states of a swerve */
public enum SwerveState {
  /** Driving under manual joystick control */
  joystickDrive,
  /** Stopped with wheels in an x pattern */
  stopWithX,
  /** Automatically driving to setpoint of following path */
  autoDriveInProgress,
  /** Finished automatically driving */
  autoDriveDone,
  /** Automatic drive failed */
  autoDriveFail,
  /** Unknown state */
  other;
}
