// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED.ColorOrder;

/** Add your docs here. */
public class DeceiverLEDConstants {

  /** Port on the roboRIO the led strip is connected to */
  public static final int ledPort = 0;

  /** Modes that allow for testing animations without overriding logic */
  public static enum LedTestingMode {
    STARTUP,
    SOLID,
    STROBE,
    BREATH,
    RAINBOW,
    WAVE,
    STRIPES,
  }

  /** Should the led strips be in testing mode */
  public static final boolean isLedTestingMode = false;
  /** What mode should be tested */
  public static LedTestingMode testingMode = LedTestingMode.SOLID;

  /** How many leds are in the complete strip */
  public static final int numLEDs = 50;
  // ** What is the color order for the led strip */
  public static final ColorOrder stripColorOrder = ColorOrder.kRGB;

  // Pattern Constants
  public static final double strobeFastDuration = 0.1;
  public static final double strobeSlowDuration = 0.25;
  public static final double breathDuration = 1.0;
  public static final double rainbowCycleLength = 25.0;
  public static final double rainbowDuration = 1.0;
  public static final double waveExponent = 0.4;
  public static final double waveFastCycleLength = 25.0;
  public static final double waveFastDuration = 0.25;
  public static final double waveSlowCycleLength = 25.0;
  public static final double waveSlowDuration = 3.0;
  public static final double waveAllianceCycleLength = 15.0;
  public static final double waveAllianceDuration = 2.0;
  public static final double autoFadeTime = 2.5;
  public static final double autoFadeMaxTime = 5.0;
  public static final int stripeLength = 3;
  public static final double stripeDuration = 1.0;
}
