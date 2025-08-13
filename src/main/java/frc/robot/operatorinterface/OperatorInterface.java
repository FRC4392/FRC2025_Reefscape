package frc.robot.operatorinterface;

import static frc.robot.operatorinterface.OperatorInterfaceConstants.*;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class OperatorInterface {
    private final CommandXboxController driverController;
    private final CommandXboxController operatorController;

    public OperatorInterface(){
        driverController = new CommandXboxController(DriverControllerPort);
        operatorController = new CommandXboxController(OperatorControllerPort);
    }
}
