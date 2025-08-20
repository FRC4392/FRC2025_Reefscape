package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import java.util.Optional;
import org.littletonrobotics.junction.AutoLogOutput;

/** Used to track various robot states and status */
public class DeceiverRobotState {

  // Standard robot state data
  private boolean wasEnabled = false;
  private boolean wasAuto = false;
  private boolean wasTeleop = false;
  private boolean wasTest = false;
  private boolean isAuto = false;
  private boolean isTeleop = false;
  private boolean isTest = false;
  private boolean isDisabled = false;

  private double disabledStartTime = 0.0;
  private double teleopStartTime = 0.0;
  private double autoStartTime = 0.0;
  private double testStartTime = 0.0;

  /**
   * Get a value if the robot has been enabled since last boot
   *
   * @return True if the tobot has been enabled, false if not
   */
  @AutoLogOutput(key = "RobotState/wasEnabled")
  public boolean getWasEnabled() {
    return wasEnabled;
  }

  /**
   * Get a value if the robot has entered auto mode since last boot
   *
   * @return True if auto has been entered, false if not
   */
  @AutoLogOutput(key = "RobotState/wasAuto")
  public boolean getWasAuto() {
    return wasAuto;
  }

  /**
   * Get a value if the robot has entered teleop mode since last boot
   *
   * @return True if teleop has been entered, false if not
   */
  @AutoLogOutput(key = "RobotState/wasTeleop")
  public boolean getWasTeleop() {
    return wasTeleop;
  }

  /**
   * Get a value if the robot has entered test mode since last boot
   *
   * @return True if test mode has been entered, false if not
   */
  @AutoLogOutput(key = "RobotState/wasTest")
  public boolean isWasTest() {
    return wasTest;
  }

  /**
   * Get a value indicating if the robot is running test or not
   *
   * @return True when the robot is running test, false when it is not
   */
  @AutoLogOutput(key = "RobotState/isTest")
  public boolean isTest() {
    return isTest;
  }

  /**
   * Set if the robot is running in test mode
   *
   * <p>Should only be set to true in testInit and false in testExit
   *
   * @param isAuto true when in test, false otherwise
   */
  protected void setTest(boolean isTest) {

    if (isTest == true && this.isTest == false) {
      testStartTime = Timer.getFPGATimestamp();
    } else if (isTest == false) {
      testStartTime = 0.0;
    }

    this.isTest = isTest;

    if (isTest == true) {
      wasTest = true;
    }
  }

  /**
   * Get a value indicating if the robot is running auto or not
   *
   * @return True when the robot is running auto, false when it is not
   */
  @AutoLogOutput(key = "RobotState/isAuto")
  public boolean isAuto() {
    return isAuto;
  }

  /**
   * Set if the robot is running in auto mode
   *
   * <p>Should only be set to true in autoInit and false in autoExit
   *
   * @param isAuto true when in auto, false otherwise
   */
  protected void setAuto(boolean isAuto) {

    if (isAuto == true && this.isAuto == false) {
      autoStartTime = Timer.getFPGATimestamp();
    } else if (isTest == false) {
      autoStartTime = 0.0;
    }

    this.isAuto = isAuto;

    if (isAuto == true) {
      wasAuto = true;
    }
  }

  /**
   * Get a value indicating if the robot is running teleop or not
   *
   * @return True when the robot is running teleop, false when it is not
   */
  @AutoLogOutput(key = "RobotState/isTeleop")
  public boolean isTeleop() {
    return isTeleop;
  }

  /**
   * Set if the robot is running in teleop mode
   *
   * <p>Should only be set to true in teleopInit and false in telopExit
   *
   * @param isTeleop true when in teleop, false otherwise
   */
  protected void setTeleop(boolean isTeleop) {

    if (isTeleop == true && this.isTeleop == false) {
      teleopStartTime = Timer.getFPGATimestamp();
    } else if (isTest == false) {
      teleopStartTime = 0.0;
    }

    this.isTeleop = isTeleop;

    if (isTeleop == true) {
      wasTeleop = true;
    }
  }

  /**
   * Get a value indicating if the robot is disabled or not
   *
   * @return True when the robot is disabled, false when it is not
   */
  @AutoLogOutput(key = "RobotState/isDisabled")
  public boolean isDisabled() {
    return isDisabled;
  }

  /**
   * Get a value indicating if the robot is enabled or not
   *
   * @return True when the robot is enabled, false when it is not
   */
  @AutoLogOutput(key = "RobotState/isEnabled")
  public boolean isEnabled() {
    return !isDisabled;
  }

  /**
   * Sets if the robot is disabled or not
   *
   * <p>Should be only set to true in disabledInit and false in disabledExit
   *
   * @param isDisabled Current disabled state
   */
  protected void setDisabled(boolean isDisabled) {

    if (isDisabled == true && this.isDisabled == false) {
      disabledStartTime = Timer.getFPGATimestamp();
    } else if (isDisabled == false) {
      disabledStartTime = 0.0;
    }

    this.isDisabled = isDisabled;

    if (!isDisabled) {
      this.wasEnabled = true;
    }
  }

  /**
   * Get the amount of time the robot has been in test mode
   *
   * @return time in seconds since the robot has been running in test mode, 0 if not in test mode
   */
  @AutoLogOutput(key = "RobotState/testTime")
  public double getTestTime() {
    if (isTest) {
      return Timer.getFPGATimestamp() - testStartTime;
    } else {
      return 0.0;
    }
  }

  /**
   * Get the amount of time the robot has been in auto mode
   *
   * @return time in seconds since the robot has been running in auto mode, 0 if not in auto mode
   */
  @AutoLogOutput(key = "RobotState/autoTime")
  public double getAutoTime() {
    if (isAuto) {
      return Timer.getFPGATimestamp() - autoStartTime;
    } else {
      return 0.0;
    }
  }

  /**
   * Get the amount of time the robot has been in teleop mode
   *
   * @return time in seconds since the robot has been running in teleop mode, 0 if not in teleop
   *     mode
   */
  @AutoLogOutput(key = "RobotState/teleopTime")
  public double getTeleopTime() {
    if (isTeleop) {
      return Timer.getFPGATimestamp() - teleopStartTime;
    } else {
      return 0.0;
    }
  }

  /**
   * Get the amount of time the robot has been in disabled mode
   *
   * @return time in seconds since the robot has been running in disabled mode, 0 if not in disabled
   *     mode
   */
  @AutoLogOutput(key = "RobotState/disabledTime")
  public double getDisabledTime() {
    if (isDisabled) {
      return Timer.getFPGATimestamp() - disabledStartTime;
    } else {
      return 0.0;
    }
  }

  /**
   * Get the current alliance from the FMS.
   *
   * <p>If the FMS is not connected, it is set from the team alliance setting on the driver station.
   *
   * @return The alliance (red or blue) or an empty optional if the alliance is invalid
   */
  public Optional<Alliance> getAlliance() {
    return DriverStation.getAlliance();
  }

  /**
   * Gets a value indicating whether the Robot is e-stopped.
   *
   * @return True if the robot is e-stopped, false otherwise.
   */
  @AutoLogOutput(key = "RobotState/isEstopped")
  public boolean getIsEstopped() {
    return DriverStation.isEStopped();
  }

  /**
   * Check if the system is browned out.
   *
   * @return True if the system is browned out
   */
  @AutoLogOutput(key = "RobotState/isBrownedOut")
  public boolean getIsBrownedOut() {
    return RobotController.isBrownedOut();
  }

  /**
   * Get a value indicating if the robot is on the Red alliance
   *
   * @return true if on red, false if not
   */
  @AutoLogOutput(key = "RobotState/isRed")
  private boolean isRedAlliance() {
    if (getAlliance().isPresent()) {
      return getAlliance().get() == Alliance.Red;
    } else {
      return false;
    }
  }

  /**
   * Get a value indicating if the robot is on the Blue alliance
   *
   * @return true if on blue, false if not
   */
  @AutoLogOutput(key = "RobotState/isBlue")
  private boolean isBlueAlliance() {
    if (getAlliance().isPresent()) {
      return getAlliance().get() == Alliance.Blue;
    } else {
      return false;
    }
  }

  /** Resets the robot state to initial startup. Only works if the robot is currently disabled. */
  public void resetState() {
    if (isDisabled) {
      wasEnabled = false;
      wasAuto = false;
      wasTeleop = false;
      wasTest = false;
      isAuto = false;
      isTeleop = false;
      isTest = false;
      disabledStartTime = 0.0;
      teleopStartTime = 0.0;
      autoStartTime = 0.0;
      testStartTime = 0.0;
    }
  }
}
