package frc.robot.subsystems.leds;

import static frc.robot.subsystems.leds.DeceiverLEDConstants.*;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.List;

/** Subsytem that manages the LEDs of the robot */
public class DeceiverLED extends SubsystemBase {

  // LED data
  private final AddressableLED leds;
  private final AddressableLEDBuffer buffer;

  // Startup notifier
  private final Notifier loadingNotifier;

  //Robot data
  private final RobotState robotState;

  /** Constructor */
  public DeceiverLED(RobotState state) {
    robotState = state;
    
    leds = new AddressableLED(ledPort);
    leds.setColorOrder(stripColorOrder);
    buffer = new AddressableLEDBuffer(numLEDs);
    leds.setLength(numLEDs);
    leds.setData(buffer);
    leds.start();

    // Start pattern while robot is booting
    loadingNotifier = new Notifier(this::startupAnimation);

    loadingNotifier.startPeriodic(0.02);
  }

  /** Update loop for LEDs, called every loop cycle */
  @SuppressWarnings("unused")
  @Override
  public void periodic() {

    // Stop the loading animation
    loadingNotifier.stop();

    // For testing patterns, enable testing constant. Don't run testing patterns when on real field
    if (isLedTestingMode && !DriverStation.isFMSAttached()) {
      switch (testingMode) {
        case STARTUP:
          startupAnimation();
          break;
        case SOLID:
          solid(Section.FULL, Color.kAliceBlue);
          break;
        case STROBE:
          strobe(Section.FULL, Color.kAliceBlue, strobeSlowDuration);
          break;
        case BREATH:
          breath(Section.FULL, Color.kAliceBlue, Color.kAntiqueWhite, breathDuration);
          break;
        case RAINBOW:
          rainbow(Section.FULL, rainbowCycleLength, rainbowDuration);
          break;
        case WAVE:
          wave(
              Section.FULL,
              Color.kAliceBlue,
              Color.kAntiqueWhite,
              waveFastCycleLength,
              waveFastDuration);
          break;
        case STRIPES:
          stripes(
              Section.FULL,
              List.of(Color.kAliceBlue, Color.kAntiqueWhite),
              stripeLength,
              stripeDuration);
        default:
          break;
      }
    } else {
      // Run user LED code here
    }

    leds.setData(buffer);
  }

  /** Animation that runs while the robot is booting up */
  private synchronized void startupAnimation() {
    breath(Section.FULL, Color.kBlue, Color.kWhite, strobeSlowDuration);
    leds.setData(buffer);
  }

  /**
   * Display solid color
   *
   * @param section The subsection of the LED strip to display on
   * @param color The color to display
   */
  private void solid(Section section, Color color) {
    if (color != null) {
      for (int i = section.start(); i < section.end(); i++) {
        buffer.setLED(i, color);
      }
    }
  }

  /**
   * Strobe LEDs on and off
   *
   * @param section The subsection of the LED strip to display on
   * @param color The color to display
   * @param duration The length of time in seconds for the animation to complete
   */
  private void strobe(Section section, Color color, double duration) {
    boolean on = ((Timer.getFPGATimestamp() % duration) / duration) > 0.5;
    solid(section, on ? color : Color.kBlack);
  }

  /**
   * Breath between two colors (fad in and out)
   *
   * @param section The subsection of the LED strip to display on
   * @param c1 The first color to display
   * @param c2 The second color to display
   * @param duration The length of time in seconds for the animation to complete
   */
  private void breath(Section section, Color c1, Color c2, double duration) {
    double timestamp = Timer.getFPGATimestamp();
    double x = ((timestamp % breathDuration) / breathDuration) * 2.0 * Math.PI;
    double ratio = (Math.sin(x) + 1.0) / 2.0;
    double red = (c1.red * (1 - ratio)) + (c2.red * ratio);
    double green = (c1.green * (1 - ratio)) + (c2.green * ratio);
    double blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio);
    solid(section, new Color(red, green, blue));
  }

  /**
   * Display a moving rainbow
   *
   * @param section The subsection of the LED strip to display on
   * @param cycleLength The length of the rainbow before repeat
   * @param duration The length of time in seconds for the animation to complete
   */
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

  /**
   * Display a moving wave between two colors (gradient between two colors that moves)
   *
   * @param section The subsection of the LED strip to display on
   * @param c1 The first color to display
   * @param c2 The second color to display
   * @param cycleLength The length of the wave before restarting
   * @param duration The length of time in seconds for the animation to complete
   */
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

  /**
   * Display stipes of multiple colors
   *
   * @param section The subsection of the LED strip to display on
   * @param colors List of colors to display
   * @param length The length of one set of colors
   * @param duration The length of time in seconds for the animation to complete
   */
  private void stripes(Section section, List<Color> colors, int length, double duration) {
    int offset = (int) (Timer.getFPGATimestamp() % duration / duration * length * colors.size());
    for (int i = section.start(); i < section.end(); i++) {
      int colorIndex =
          (int) (Math.floor((double) (i - offset) / length) + colors.size()) % colors.size();
      colorIndex = colors.size() - 1 - colorIndex;
      buffer.setLED(i, colors.get(colorIndex));
    }
  }

  /**
   * Subsection of LED strip
   *
   * <p>define subsections of the led strip with different start and end points
   */
  private static enum Section {
    /** Entire led strip */
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
          return numLEDs;
        default:
          return numLEDs;
      }
    }
  }
}
