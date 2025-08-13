// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLED.ColorOrder;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.gripper.Gripper.GripperState;
import frc.robot.subsystems.swerve.SwerveState;
import java.util.List;
import java.util.function.Supplier;

@SuppressWarnings("unused")
public class Leds extends SubsystemBase {
  // LED data
  private final AddressableLED leds;
  private final AddressableLEDBuffer buffer;
  private static final int length = 50;

  // Pattern Constants
  private static final double strobeFastDuration = 0.1;
  private static final double strobeSlowDuration = 0.25;
  private static final double breathDuration = 1.0;
  private static final double rainbowCycleLength = 25.0;
  private static final double rainbowDuration = 1.0;
  private static final double waveExponent = 0.4;
  private static final double waveFastCycleLength = 25.0;
  private static final double waveFastDuration = 0.25;
  private static final double waveSlowCycleLength = 25.0;
  private static final double waveSlowDuration = 3.0;
  private static final double waveAllianceCycleLength = 15.0;
  private static final double waveAllianceDuration = 2.0;
  private static final double autoFadeTime = 2.5;
  private static final double autoFadeMaxTime = 5.0;
  private static final int stripeLength = 3;
  private static final double stripeDuration = 1.0;

  // Startup notifier
  private final Notifier loadingNotifier;

  // Subsystem states
  private GripperState gripperState = GripperState.OFF;
  private Supplier<GripperState> gripperSupplier;
  private SwerveState swerveState = SwerveState.other;
  private Supplier<SwerveState> swerveSupplier;

  /** Creates a new Leds. */
  public Leds() {
    // Configure LED strip
    leds = new AddressableLED(0);
    leds.setColorOrder(ColorOrder.kRGB);
    buffer = new AddressableLEDBuffer(length);
    leds.setLength(length);
    leds.setData(buffer);
    leds.start();

    // Start pattern while robot is booting
    loadingNotifier =
        new Notifier(
            () -> {
              synchronized (this) {
                breath(
                    Section.FULL,
                    Color.kBlue,
                    Color.kWhite,
                    strobeSlowDuration,
                    System.currentTimeMillis() / 1000.0);
                leds.setData(buffer);
              }
            });

    loadingNotifier.startPeriodic(0.02);
  }

  @Override
  public void periodic() {
    gripperState = gripperSupplier.get();
    swerveState = swerveSupplier.get();

    // Stop loading pattern after it has booted
    loadingNotifier.stop();

    if (!DriverStation.isDSAttached()) {
      // No driverstation attached
      strobe(Section.FULL, Color.kRed, strobeSlowDuration);
    } else if (DriverStation.isDisabled()) {
      // Disabled
      stripes(Section.FULL, List.of(Color.kWhite, Color.kBlue), stripeLength, stripeDuration);
    } else if (DriverStation.isAutonomous()) {
      // In Autonomous
      rainbow(Section.FULL, rainbowCycleLength, rainbowDuration);
    } else {
      // In teleop or any other mode
      if ((swerveState == SwerveState.joystickDrive) || (swerveState == SwerveState.other)) {
        switch (gripperState) {
          case OFF:
            wave(Section.FULL, Color.kBlue, Color.kWhite, waveSlowCycleLength, waveSlowDuration);
            break;
          case IntakeOuttakeWithNone:
            strobe(Section.FULL, Color.kBlue, strobeSlowDuration);
            break;
          case IntakeOuttakeWithBoth:
            break;
          case IntakeOuttakeWithAlgae:
            strobe(Section.FULL, Color.kTeal, breathDuration);
            break;
          case IntakeOuttakeWithCoral:
            strobe(Section.FULL, Color.kWhite, breathDuration);
            break;
          case HasAlgae:
            solid(Section.FULL, Color.kAquamarine);
            break;
          case HasCoral:
            solid(Section.FULL, Color.kOrange);
            break;
          case HasBoth:
            stripes(Section.FULL, List.of(Color.kTeal, Color.kWhite), stripeLength, stripeDuration);
            break;
        }
      } else {
        switch (swerveState) {
          case other:
          case joystickDrive:
            strobe(Section.FULL, Color.kDarkRed, strobeFastDuration);
            break;
          case autoDriveDone:
            strobe(Section.FULL, Color.kGreen, strobeFastDuration);
            break;
          case autoDriveFail:
            strobe(Section.FULL, Color.kRed, strobeFastDuration);
            break;
          case autoDriveInProgress:
            solid(Section.FULL, Color.kYellow);
            break;
          case stopWithX:
            solid(Section.FULL, Color.kRed);
            break;
        }
      }
    }

    leds.setData(buffer);
  }

  // Display a solid color
  private void solid(Section section, Color color) {
    if (color != null) {
      for (int i = section.start(); i < section.end(); i++) {
        buffer.setLED(i, color);
      }
    }
  }

  // Strobe color on and off
  private void strobe(Section section, Color color, double duration) {
    boolean on = ((Timer.getFPGATimestamp() % duration) / duration) > 0.5;
    solid(section, on ? color : Color.kBlack);
  }

  // Breath between two colors (fad in and out)
  private void breath(Section section, Color c1, Color c2, double duration) {
    breath(section, c1, c2, duration, Timer.getFPGATimestamp());
  }

  // Breath between two colors (fad in and out)
  private void breath(Section section, Color c1, Color c2, double duration, double timestamp) {
    double x = ((timestamp % breathDuration) / breathDuration) * 2.0 * Math.PI;
    double ratio = (Math.sin(x) + 1.0) / 2.0;
    double red = (c1.red * (1 - ratio)) + (c2.red * ratio);
    double green = (c1.green * (1 - ratio)) + (c2.green * ratio);
    double blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio);
    solid(section, new Color(red, green, blue));
  }

  // Display a moving rainbow
  private void rainbow(Section section, double cycleLength, double duration) {
    double x = (1 - ((Timer.getFPGATimestamp() / duration) % 1.0)) * 180.0;
    double xDiffPerLed = 180.0 / cycleLength;
    for (int i = 0; i < section.end(); i++) {
      x += xDiffPerLed;
      x %= 180.0;
      if (i >= section.start()) {
        buffer.setHSV(i, (int) x, 255, 255);
      }
    }
  }

  // Display a moving wave between two colors (gradient between two colors that moves)
  private void wave(Section section, Color c1, Color c2, double cycleLength, double duration) {
    double x = (1 - ((Timer.getFPGATimestamp() % duration) / duration)) * 2.0 * Math.PI;
    double xDiffPerLed = (2.0 * Math.PI) / cycleLength;
    for (int i = 0; i < section.end(); i++) {
      x += xDiffPerLed;
      if (i >= section.start()) {
        double ratio = (Math.pow(Math.sin(x), waveExponent) + 1.0) / 2.0;
        if (Double.isNaN(ratio)) {
          ratio = (-Math.pow(Math.sin(x + Math.PI), waveExponent) + 1.0) / 2.0;
        }
        if (Double.isNaN(ratio)) {
          ratio = 0.5;
        }
        double red = (c1.red * (1 - ratio)) + (c2.red * ratio);
        double green = (c1.green * (1 - ratio)) + (c2.green * ratio);
        double blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio);
        buffer.setLED(i, new Color(red, green, blue));
      }
    }
  }

  // Display stipes of multiple colors
  private void stripes(Section section, List<Color> colors, int length, double duration) {
    int offset = (int) (Timer.getFPGATimestamp() % duration / duration * length * colors.size());
    for (int i = section.start(); i < section.end(); i++) {
      int colorIndex =
          (int) (Math.floor((double) (i - offset) / length) + colors.size()) % colors.size();
      colorIndex = colors.size() - 1 - colorIndex;
      buffer.setLED(i, colors.get(colorIndex));
    }
  }

  private static enum Section {
    FULL;

    private int start() {
      switch (this) {
        case FULL:
          return 0;
        default:
          return 0;
      }
    }

    private int end() {
      switch (this) {
        case FULL:
          return length;
        default:
          return length;
      }
    }
  }

  public void setGripperSupplier(Supplier<GripperState> newSupplier) {
    gripperSupplier = newSupplier;
  }

  public void setSwerveSupplier(Supplier<SwerveState> newSupplier) {
    swerveSupplier = newSupplier;
  }
}
