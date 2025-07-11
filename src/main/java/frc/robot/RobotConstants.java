package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/** Constants used for robot settings */
public final class RobotConstants {
  /**
   * What sim mode should be used?
   *
   * <p>Mode.SIM = Simulator
   *
   * <p>Mode.Replay = AdvantageKit Replay
   */
  public static final Mode simMode = Mode.SIM;

  /** What is the current mode the robot is in? */
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  /** The mode the robot is in */
  public static enum Mode {
    /** Real Robot */
    REAL,
    /** Simulation */
    SIM,
    /** AdvantageKit Replay */
    REPLAY
  }
}
