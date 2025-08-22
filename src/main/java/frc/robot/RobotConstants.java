package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/** Constants used for robot settings */
public final class RobotConstants {
  private RobotConstants() {}
  /**
   * What sim mode should be used?
   *
   * <p>Mode.SIM = Simulator
   *
   * <p>Mode.REPLAY = AdvantageKit Replay
   */
  private static final Mode simMode = Mode.SIM;

  /**
   * What sim mode should be used?
   *
   * <p>Mode.REAL = Real robot as would be used on the field
   *
   * <p>Mode.COMMISIONING = Enable extra diagnostic and testing functions that aren't helpful during
   * competition
   */
  private static final Mode realMode = Mode.REAL;

  /** What is the current mode the robot is in? */
  public static final Mode currentMode = RobotBase.isReal() ? realMode : simMode;

  /** The mode the robot is in */
  public static enum Mode {
    /** Commisioning Mode for extra diagnostics and test functions */
    COMMISIONING,
    /** Real Robot */
    REAL,
    /** Simulation */
    SIM,
    /** AdvantageKit Replay */
    REPLAY
  }
}
