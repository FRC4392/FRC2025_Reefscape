package frc.robot.operatorinterface;

import static frc.robot.operatorinterface.OperatorInterfaceConstants.*;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.swerve.SwerveControlSignal;

public class OperatorInterface {
  private final CommandXboxController driverController;
  private final CommandXboxController operatorController;

  Alert driverControllerAlert = new Alert("Driver Controller Disconnected 🎮", AlertType.kError);
  Alert operatorControllerAlert =
      new Alert("Operator Controller Disconnected 🎮", AlertType.kError);

  public OperatorInterface() {
    driverController = new CommandXboxController(DriverControllerPort);
    operatorController = new CommandXboxController(OperatorControllerPort);
  }

  // Update alerts
  public void updateAlerts() {
    driverControllerAlert.set(!driverController.isConnected());
    operatorControllerAlert.set(!operatorController.isConnected());
  }

  /**
   * Starts the joystick rumbling and then stops it at the completion of the command.
   *
   * @return Command to rumble the joystick
   */
  public Command joystickRumbleCommand() {
    return Commands.startEnd(
        () -> {
          driverController.setRumble(RumbleType.kBothRumble, 1.0);
          operatorController.setRumble(RumbleType.kBothRumble, 1.0);
        },
        () -> {
          driverController.setRumble(RumbleType.kBothRumble, 0.0);
          operatorController.setRumble(RumbleType.kBothRumble, 0.0);
        });
  }

  // Serve controls

  /**
   * Get trigger to stop with wheels in an x shape
   *
   * @return Trigger to activate stopping with x
   */
  public Trigger stopWithXTrigger() {
    return driverController.x();
  }

  /**
   * Get trigger to stop with wheels in an x shape
   *
   * @return Trigger to activate stopping with x
   */
  public Trigger restGyroTrigger() {
    return driverController.start();
  }

  /**
   * Get SwerveControlSignal that represents all data need to drive swerve
   *
   * @return
   */
  public SwerveControlSignal getSwerveControlSignal() {
    return new SwerveControlSignal(
        () -> driverController.getLeftX(),
        () -> driverController.getLeftY(),
        () -> driverController.getLeftTriggerAxis() - driverController.getRightTriggerAxis(),
        () -> driverController.getHID().getAButton());
  }

  // Game Controls (vary by year)

  public Trigger intakeTrigger() {
    return driverController.leftStick();
  }

  public Trigger outtakeTrigger() {
    return driverController.rightStick();
  }

  public Trigger autoAlignLeftTrigger() {
    return driverController.leftBumper();
  }

  public Trigger autoAlignRightTrigger() {
    return driverController.rightBumper();
  }

  public boolean L1PositionTrigger() {
    return operatorController.getHID().getBButton();
  }

  public boolean L2PositionTrigger() {
    return operatorController.getHID().getAButton();
  }

  public boolean L3PositionTrigger() {
    return operatorController.getHID().getXButton();
  }

  public boolean L4PositionTrigger() {
    return operatorController.getHID().getRightStickButton();
  }

  public boolean homePositionTrigger() {
    return operatorController.getHID().getStartButton();
  }

  public boolean intakePositionTrigger() {
    return operatorController.getHID().getRightBumperButton()
        || operatorController.getHID().getLeftBumperButton();
  }

  public boolean algae1PositionTrigger() {
    return operatorController.getHID().getPOV() == 180;
  }

  public boolean algae2PositionTrigger() {
    return operatorController.getHID().getPOV() == 0;
  }

  public boolean bargePositionTrigger() {
    return operatorController.getHID().getPOV() == 90;
  }

  public boolean processorPositionTrigger() {
    return operatorController.getHID().getPOV() == 270;
  }
}
