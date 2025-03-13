// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
package frc.robot.subsystems.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO.targetPoseObservation;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class SwerveCommands {
  private static final double DEADBAND = 0.01;
  private static final double ANGLE_KP = 5.0;
  private static final double ANGLE_KD = 0.4;
  private static final double ANGLE_MAX_VELOCITY = 8.0;
  private static final double ANGLE_MAX_ACCELERATION = 20.0;
  private static final double FF_START_DELAY = 2.0; // Secs
  private static final double FF_RAMP_RATE = 0.1; // Volts/Sec
  private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
  private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2

  private static final double ReefOffsetRight = Units.inchesToMeters(6); // Meters
  private static final double ReefOffsetLeft = Units.inchesToMeters(6); // Meters
  private static final double ReffOffsetForward = .5; // Meters

  private static int targetID = 0;
  private static boolean invertOffset = false;
  private static double alignmentAngle = 0;

  private static PIDController strafeController = new PIDController(0.08, 0, 0);
  private static PIDController forwardController = new PIDController(0.08, 0, 0);

  private static ProfiledPIDController angleController =
      new ProfiledPIDController(
          ANGLE_KP,
          0.0,
          ANGLE_KD,
          new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));

  public static enum ReefSide {
    left,
    right;
  }

  private SwerveCommands() {}

  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(new Translation2d(), linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
        .getTranslation();
  }

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive(
      Swerve drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      BooleanSupplier fastMode) {
    return Commands.run(
        () -> {
          double multiplier = fastMode.getAsBoolean() ? 1 : .75;
          // Get linear velocity
          Translation2d linearVelocity =
              getLinearVelocityFromJoysticks(
                  xSupplier.getAsDouble() * multiplier, ySupplier.getAsDouble() * multiplier);

          // Apply rotation deadband
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

          omega = omega * multiplier;

          // Square rotation value for more precise control
          omega = Math.copySign(omega * omega, omega);

          // Convert to field relative speeds & send command
          ChassisSpeeds speeds =
              new ChassisSpeeds(
                  linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                  linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                  omega * drive.getMaxAngularSpeedRadPerSec());
          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;
          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  speeds,
                  isFlipped
                      ? drive.getRotation().plus(new Rotation2d(Math.PI))
                      : drive.getRotation()));
        },
        drive);
  }

  /**
   * Field relative drive command using joystick for linear control and PID for angular control.
   * Possible use cases include snapping to an angle, aiming at a vision target, or controlling
   * absolute rotation with a joystick.
   */
  public static Command joystickDriveAtAngle(
      Swerve drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Rotation2d> rotationSupplier) {

    // Create PID controller
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            ANGLE_KP,
            0.0,
            ANGLE_KD,
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    // Construct command
    return Commands.run(
            () -> {
              // Get linear velocity
              Translation2d linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

              // Calculate angular speed
              double omega =
                  angleController.calculate(
                      drive.getRotation().getRadians(), rotationSupplier.get().getRadians());

              // Convert to field relative speeds & send command
              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                      linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                      omega);
              boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      isFlipped
                          ? drive.getRotation().plus(new Rotation2d(Math.PI))
                          : drive.getRotation()));
            },
            drive)

        // Reset PID controller when command starts
        .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()));
  }

  /**
   * Measures the velocity feedforward constants for the drive motors.
   *
   * <p>This command should only be used in voltage control mode.
   */
  public static Command feedforwardCharacterization(Swerve drive) {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();

    return Commands.sequence(
        // Reset data
        Commands.runOnce(
            () -> {
              velocitySamples.clear();
              voltageSamples.clear();
            }),

        // Allow modules to orient
        Commands.run(
                () -> {
                  drive.runCharacterization(0.0);
                },
                drive)
            .withTimeout(FF_START_DELAY),

        // Start timer
        Commands.runOnce(timer::restart),

        // Accelerate and gather data
        Commands.run(
                () -> {
                  double voltage = timer.get() * FF_RAMP_RATE;
                  drive.runCharacterization(voltage);
                  velocitySamples.add(drive.getFFCharacterizationVelocity());
                  voltageSamples.add(voltage);
                },
                drive)

            // When cancelled, calculate and print results
            .finallyDo(
                () -> {
                  int n = velocitySamples.size();
                  double sumX = 0.0;
                  double sumY = 0.0;
                  double sumXY = 0.0;
                  double sumX2 = 0.0;
                  for (int i = 0; i < n; i++) {
                    sumX += velocitySamples.get(i);
                    sumY += voltageSamples.get(i);
                    sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                    sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                  }
                  double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                  double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                  NumberFormat formatter = new DecimalFormat("#0.00000");
                  System.out.println("********** Drive FF Characterization Results **********");
                  System.out.println("\tkS: " + formatter.format(kS));
                  System.out.println("\tkV: " + formatter.format(kV));
                }));
  }

  /** Measures the robot's wheel radius by spinning in a circle. */
  public static Command wheelRadiusCharacterization(Swerve drive) {
    SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

    return Commands.parallel(
        // Drive control sequence
        Commands.sequence(
            // Reset acceleration limiter
            Commands.runOnce(
                () -> {
                  limiter.reset(0.0);
                }),

            // Turn in place, accelerating up to full speed
            Commands.run(
                () -> {
                  double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                  drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                },
                drive)),

        // Measurement sequence
        Commands.sequence(
            // Wait for modules to fully orient before starting measurement
            Commands.waitSeconds(1.0),

            // Record starting measurement
            Commands.runOnce(
                () -> {
                  state.positions = drive.getWheelRadiusCharacterizationPositions();
                  state.lastAngle = drive.getRotation();
                  state.gyroDelta = 0.0;
                }),

            // Update gyro delta
            Commands.run(
                    () -> {
                      var rotation = drive.getRotation();
                      state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                      state.lastAngle = rotation;
                    })

                // When cancelled, calculate and print results
                .finallyDo(
                    () -> {
                      double[] positions = drive.getWheelRadiusCharacterizationPositions();
                      double wheelDelta = 0.0;
                      for (int i = 0; i < 4; i++) {
                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                      }
                      double wheelRadius =
                          (state.gyroDelta * SwerveConstants.driveBaseRadius) / wheelDelta;

                      NumberFormat formatter = new DecimalFormat("#0.000");
                      System.out.println(
                          "********** Wheel Radius Characterization Results **********");
                      System.out.println(
                          "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                      System.out.println(
                          "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                      System.out.println(
                          "\tWheel Radius: "
                              + formatter.format(wheelRadius)
                              + " meters, "
                              + formatter.format(Units.metersToInches(wheelRadius))
                              + " inches");
                    })));
  }

  public static Command autoAlignCommand2d(Swerve swerve, Vision vision, ReefSide side) {
    angleController.enableContinuousInput(-Math.PI, Math.PI);
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              targetID = vision.getTargetId(2);

              // determine offset
              if ((targetID >= 20 && targetID <= 22) || (targetID >= 9 && targetID <= 11)) {
                invertOffset = true;
              } else {
                invertOffset = false;
              }

              switch (targetID) {
                case 6:
                  alignmentAngle = -60;
                  break;
                case 7:
                  alignmentAngle = 0;
                  break;
                case 8:
                  alignmentAngle = 60;
                  break;
                case 9:
                  alignmentAngle = 120;
                  break;
                case 10:
                  alignmentAngle = 180;
                  break;
                case 11:
                  alignmentAngle = -120;
                  break;
                case 17:
                  alignmentAngle = -120;
                  break;
                case 18:
                  alignmentAngle = 180;
                  break;
                case 19:
                  alignmentAngle = 120;
                  break;
                case 20:
                  alignmentAngle = 60;
                  break;
                case 21:
                  alignmentAngle = 0;
                  break;
                case 22:
                  alignmentAngle = -60;
                  break;

                default:
                  alignmentAngle = 99999;
                  break;
              }
            }),
        Commands.run(
            () -> {
              double rotateSpeed = 0;
              if (alignmentAngle <= 180) {
                rotateSpeed =
                    angleController.calculate(
                        swerve.getRotation().getRadians(),
                        Rotation2d.fromDegrees(alignmentAngle).getRadians());
              }

              double strafeVelocity = 0;
              double forwardVelocity = 0;

              if (vision.getTargetId(2) == targetID) {
                double strafeAngle = vision.getTargetX(2).getDegrees();

                strafeVelocity = -strafeController.calculate(strafeAngle, 11.5);

                double forwardAngle = vision.getTargetY(2).getDegrees();

                forwardVelocity = -forwardController.calculate(forwardAngle, -3);
              }
              ChassisSpeeds chassisSpeeds =
                  new ChassisSpeeds(forwardVelocity, strafeVelocity, rotateSpeed);

              swerve.runVelocity(chassisSpeeds);
            },
            swerve));
  }

  public static Command autoAlignCommand3D(Swerve swerve, Vision vision, ReefSide side) {
    angleController.enableContinuousInput(-Math.PI, Math.PI);
    return Commands.run(
        () -> {
          ReefSide targetSide = side;
          // Get all the targets we are looking at
          List<targetPoseObservation> targets = new LinkedList<>();

          for (int i = 0; i < 3; i++) {
            targets.add(vision.getLastTargetPoseObservation(i));
          }

          // Find closest target
          int closestTag = -1;
          double closest = -1;
          for (int i = 0; i < targets.size(); i++) {
            double distance =
                targets.get(i).targetPose().getTranslation().getDistance(new Translation3d());

            if ((distance < closest) || ((closest == -1) && (distance > 0))) {
              closest = distance;
              closestTag = i;
            }
          }

          if (closestTag == -1) {
            return;
          }
          
          int closestTagID = targets.get(closestTag).targetID();
          Pose3d closestTagPose = targets.get(closestTag).targetPose();

          // Determine which way to offset
          boolean invertSide = false;
          if ((targetID >= 20 && targetID <= 22) || (targetID >= 9 && targetID <= 11)) {
            invertSide = true;
          } else {
            invertSide = false;
          }

          // invert if on the other side of the field
          if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
            invertSide = !invertSide;
          }

          if (invertSide && side == ReefSide.left) {
            targetSide = ReefSide.right;
          } else if (invertSide && side == ReefSide.right) {
            targetSide = ReefSide.left;
          }

          // Determine offset Position
          double positionOffset = 0;
          if (targetSide == ReefSide.left) {
            positionOffset = ReefOffsetLeft;
          } else if (targetSide == ReefSide.right) {
            positionOffset = ReefOffsetRight;
          }

          // Determine Rotation
          Rotation2d rotationTarget =
              swerve
                  .getRotation()
                  .plus(closestTagPose.getRotation().toRotation2d().minus(Rotation2d.k180deg));

          // Position to offset position
          double strafeSpeed = strafeController.calculate(closestTagPose.getX(), positionOffset);
          double forwadSpeed =
              forwardController.calculate(closestTagPose.getY(), ReffOffsetForward);
          double rotation =
              -angleController.calculate(
                  swerve.getRotation().getRadians(), rotationTarget.getRadians());
          // double rotation = 0;

          ChassisSpeeds speeds = new ChassisSpeeds(forwadSpeed, strafeSpeed, rotation);
          swerve.runVelocity(speeds);
        });
  }

  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = new Rotation2d();
    double gyroDelta = 0.0;
  }
}
