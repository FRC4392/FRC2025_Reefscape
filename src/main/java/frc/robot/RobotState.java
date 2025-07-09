package frc.robot;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

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

    public boolean getWasEnabled() {
        return wasEnabled;
    }

    public boolean getWasAuto() {
        return wasAuto;
    }

    public boolean getWasTeleop() {
        return wasTeleop;
    }

    public boolean isWasTest() {
        return wasTest;
    }

    public boolean isTest() {
        return isTest;
    }

    protected void setTest(boolean isTest) {
        this.isTest = isTest;
        if (isTest == true){
            wasTest = true;
        }
    }

    public boolean isAuto() {
        return isAuto;
    }

    protected void setAuto(boolean isAuto) {
        this.isAuto = isAuto;
        if (isAuto == true){
            wasAuto = true;
        }
    }

    public boolean isTeleop() {
        return isTeleop;
    }

    protected void setTeleop(boolean isTeleop) {
        this.isTeleop = isTeleop;
        if (isTeleop == true){
            wasTeleop = true;
        }
    }

    public boolean isDisabled() {
        return isDisabled;
    }

    protected void setDisabled(boolean isDisabled) {
        this.isDisabled = isDisabled;
    }
    
  /**
   * Get the current alliance from the FMS.
   *
   * <p>If the FMS is not connected, it is set from the team alliance setting on the driver station.
   *
   * @return The alliance (red or blue) or an empty optional if the alliance is invalid
   */
    public Optional<Alliance> getAlliance(){
        return DriverStation.getAlliance();
    }



}
