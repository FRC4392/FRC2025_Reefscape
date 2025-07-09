package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import java.util.Optional;

/** Add your docs here. */
public class RobotState {
  // Standard robot state data

  private boolean wasEnabled = false;
  private boolean wasAuto = false;
  private boolean wasTeleop = false;
  private boolean wasTest = false;
  private boolean isAuto = false;
  private boolean isTeleop = false;
  private boolean isTest = false;
  private boolean isDisabled = false;

  /**
   * Get a value if the robot has been enabled since last boot
   *
   * @return True if the tobot has been enabled, false if not
   */
  public boolean getWasEnabled() {
    return wasEnabled;
  }

  /**
   * Get a value if the robot has entered auto mode since last boot
   *
   * @return True if auto has been entered, false if not
   */
  public boolean getWasAuto() {
    return wasAuto;
  }

  /**
   * Get a value if the robot has entered teleop mode since last boot
   *
   * @return True if teleop has been entered, false if not
   */
  public boolean getWasTeleop() {
    return wasTeleop;
  }

  /**
   * Get a value if the robot has entered test mode since last boot
   *
   * @return True if test mode has been entered, false if not
   */
  public boolean isWasTest() {
    return wasTest;
  }

  /**
   * Get a value indicating if the robot is running test or not
   *
   * @return True when the robot is running test, false when it is not
   */
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
  public boolean isDisabled() {
    return isDisabled;
  }

  /**
   * Get a value indicating if the robot is enabled or not
   *
   * @return True when the robot is enabled, false when it is not
   */
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
    this.isDisabled = isDisabled;

    if (!isDisabled) {
      this.wasEnabled = true;
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
  public boolean getIsEstopped() {
    return DriverStation.isEStopped();
  }

  /**
   * Check if the system is browned out.
   *
   * @return True if the system is browned out
   */
  public boolean getIsBrownedOut() {
    return RobotController.isBrownedOut();
  }
}
