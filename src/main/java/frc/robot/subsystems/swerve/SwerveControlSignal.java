package frc.robot.subsystems.swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/** Add your docs here. */
public class SwerveControlSignal {
  private DoubleSupplier xSignal;
  private DoubleSupplier ySignal;
  private DoubleSupplier omegaSignal;
  private BooleanSupplier allowFullSpeedSignal;

  public DoubleSupplier getxSignal() {
    return xSignal;
  }

  public void setxSignal(DoubleSupplier xSignal) {
    this.xSignal = xSignal;
  }

  public DoubleSupplier getySignal() {
    return ySignal;
  }

  public void setySignal(DoubleSupplier ySignal) {
    this.ySignal = ySignal;
  }

  public DoubleSupplier getOmegaSignal() {
    return omegaSignal;
  }

  public void setOmegaSignal(DoubleSupplier omegaSignal) {
    this.omegaSignal = omegaSignal;
  }

  public BooleanSupplier getAllowFullSpeedSignal() {
    return allowFullSpeedSignal;
  }

  public void setAllowFullSpeedSignal(BooleanSupplier allowFullSpeedSignal) {
    this.allowFullSpeedSignal = allowFullSpeedSignal;
  }

  public SwerveControlSignal(
      DoubleSupplier x, DoubleSupplier y, DoubleSupplier omega, BooleanSupplier fullSpeed) {
    xSignal = x;
    ySignal = y;
    omegaSignal = omega;
    allowFullSpeedSignal = fullSpeed;
  }
}
