package frc.robot.subsystems.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO.targetPoseObservation;

import static frc.robot.subsystems.swerve.SwerveConstants.pathConstraints;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;

public class SwerveCommands {
  private static final double DEADBAND = 0.01;
  private static final double ANGLE_KP = 4.0;
  private static final double ANGLE_KD = 0.4;
  private static final double ANGLE_MAX_VELOCITY = 8.0;
  private static final double ANGLE_MAX_ACCELERATION = 20.0;
  private static final double FF_START_DELAY = 2.0; // Secs
  private static final double FF_RAMP_RATE = 0.1; // Volts/Sec
  private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
  private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2
  private static final double SLOW_SPEED_PERCENTAGE =
      0.75; // Percentage of full speed when in slow mode

  private static final double ReefOffsetRight = Units.inchesToMeters(-6.5); // Meters
  private static final double ReefOffsetLeft = Units.inchesToMeters(6.5); // Meters
  private static final double ReffOffsetForward = -.55; // Meters

  public static enum ReefSide {
    left,
    right;
  }

  private SwerveCommands() {}

  /**
   * Helper function to get linear velocity of the joysticks.
   *
   * <p>Takes raw inputs from a joysticks axis and converts them to a linear movement. Deadband is
   * applied to the linear distance and then the the value is squared to give the driver finer
   * control.
   *
   * @param x Position of the x axis of the joystick in range -1 to 1
   * @param y Position of the x axis of the joystick in range -1 to 1
   * @return Translation2D that represents the linear velocity from the joysticks
   */
  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Translation2d(linearMagnitude, linearDirection);
  }

  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = new Rotation2d();
    double gyroDelta = 0.0;
  }

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   *
   * @param swerve Swerve Drive dependancy
   * @param signal Signal composing x and y speeds along with rotation speed and signal to specify
   *     full speeds
   * @return
   */
  public static Command joystickDrive(Swerve swerve, SwerveControlSignal signal) {
    return joystickDrive(
        swerve,
        signal.getxSignal(),
        signal.getySignal(),
        signal.getOmegaSignal(),
        signal.getAllowFullSpeedSignal());
  }

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   *
   * @param drive Swerve Drive dependancy
   * @param xSupplier DoubleSupplier that supplies the x position of the joystick
   * @param ySupplier DoubleSupplier that supplies the y position of the joystick
   * @param omegaSupplier DoubleSupplier that supplies the rotation position of the joystick
   * @param fastMode BooleanSupplier that indicates if the robot should travel full speed or not
   * @return Command that is used for joystick drive
   */
  public static Command joystickDrive(
      Swerve drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      BooleanSupplier fastMode) {
    return Commands.run(
        () -> {
          drive.setSwerveState(SwerveState.joystickDrive);
          double multiplier = fastMode.getAsBoolean() ? 1 : SLOW_SPEED_PERCENTAGE;

          // Get linear velocity multiplied by speed scalar
          Translation2d linearVelocity =
              getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble())
                  .times(multiplier);

          // Apply rotation deadband
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

          // Apply speed scalar
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
   *
   * @param drive Swerve Drive dependancy
   * @param xSupplier DoubleSupplier that supplies the x position of the joystick
   * @param ySupplier DoubleSupplier that supplies the y position of the joystick
   * @param rotationSupplier Rotation2D Supplied that supplies the specified angle of the drivetrain
   * @param fastMode BooleanSupplier that indicates if the robot should travel full speed or not
   * @return Command for driving with joystick at a specified angle
   */
  public static Command joystickDriveAtAngle(
      Swerve drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Rotation2d> rotationSupplier,
      BooleanSupplier fastMode) {

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
              drive.setSwerveState(SwerveState.joystickDrive);
              double multiplier = fastMode.getAsBoolean() ? 1 : SLOW_SPEED_PERCENTAGE;
              // Get linear velocity
              Translation2d linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble())
                      .times(multiplier);

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
   *
   * @param drive Swerve Drive dependency
   * @return Command to measure feedforward
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
              drive.setSwerveState(SwerveState.other);
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

  /**
   * Measures the robot's wheel radius by spinning in a circle and comparing distance traveled to
   * angle moved
   *
   * @param drive Swerve Drive dependancy
   * @return Command to measure wheel radius
   */
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
                  drive.setSwerveState(SwerveState.other);
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

  /**
   * Drive straight to specified pose
   * 
   * The origin for the pose is on the blue side of the field with x pointing away from the driverstation and z pointing up
   * @param swerve swerve drive dependancy
   * @param pose pose to drive to
   * @return command to drive to the pose
   */
  public Command driveToPose(Swerve swerve, Pose2d pose){
    return Commands.run(() -> {
      //TODO: write pose drive command
    }, swerve);
  }

  /**
   * Drive straight to specified pose
   * 
   * If red alliance is specified the path will be flipped so the same pose goes to the same point regardless of alliace color
   * @param swerve swerve drive dependancy
   * @param pose pose to drive to
   * @param alliance current alliance color
   * @return command to drive to that pose
   */
  public Command driveToPose(Swerve swerve, Pose2d pose, Alliance alliance){
    if (alliance == Alliance.Red){
      pose = FlippingUtil.flipFieldPose(pose);
    }
    return driveToPose(swerve, pose);
  }

  /**
   * Use Pathplanner to find a path to a specific pose.
   * 
   * If red alliance is specified the path will be flipped so the same pose goes to the same point regardless of alliace color
   * @param pose The pose to pathfind to
   * @param endVelocity The end velocity to end the path with
   * @param alliance The current alliance color
   * @return Command to pathfind to that pose
   */
  public Command pathfindToPose(Pose2d pose, LinearVelocity endVelocity, Alliance alliance){
    if (alliance == Alliance.Red) {
      return AutoBuilder.pathfindToPoseFlipped(pose, pathConstraints, endVelocity);
    } else {
      return AutoBuilder.pathfindToPose(pose, pathConstraints, endVelocity);
    }
  }

  /**
   * Use Pathplanner to find a path to a specific pose.
   * 
   * The origin for the pose is on the blue side of the field with x pointing away from the driverstation and z pointing up
   * @param pose The pose to pathfind to
   * @param endVelocity The end velocity to end the path with
   * @param alliance The current alliance color
   * @return Command to pathfind to that pose
   */
  public Command pathfindToPose(Pose2d pose, LinearVelocity endVelocity){
    return AutoBuilder.pathfindToPose(pose, pathConstraints, endVelocity);
  }

  /**
   * Use Pathplanner to find a path to the start point of a path, then follow that path
   * @param path path to pathfind to and then follow
   * @return command to pathfind and then follow a path
   */
  public Command pathfindToPathThenFollow(PathPlannerPath path){
    return AutoBuilder.pathfindThenFollowPath(path, pathConstraints);
  }

  /**
   * Stop the swerve drive and position the wheels in an X pattern
   *
   * @param swerve Swerve Drive dependancy
   * @return Command to stop the drivetrain
   */
  public static Command stopWithX(Swerve swerve) {
    return Commands.run(
        () -> {
          swerve.stopWithX();
          swerve.setSwerveState(SwerveState.stopWithX);
        },
        swerve);
  }

    // Year Specific commands

    //Bad
    public static Command autoAlignCommand3D(Swerve swerve, Vision vision, ReefSide side) {
      @SuppressWarnings("resource")
      PIDController strafeController = new PIDController(4, 0, 0); // 0.08
      @SuppressWarnings("resource")
      PIDController forwardController = new PIDController(4, 0, 0); // 0.08
  
      strafeController.setTolerance(Units.inchesToMeters(2));
      forwardController.setTolerance(Units.inchesToMeters(2));
  
      ProfiledPIDController angleController =
          new ProfiledPIDController(
              ANGLE_KP,
              0.0,
              ANGLE_KD,
              new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
  
      angleController.enableContinuousInput(-Math.PI, Math.PI);
      angleController.setTolerance(Units.degreesToRadians(3));
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
  
              if ((distance < closest && distance > 0) || ((closest == -1) && (distance > 0))) {
                closest = distance;
                closestTag = i;
              }
            }
  
            if (closestTag == -1) {
              swerve.runVelocity(new ChassisSpeeds());
              swerve.setSwerveState(SwerveState.autoDriveFail);
              return;
            }
  
            int closestTagID = targets.get(closestTag).targetID();
            Pose3d closestTagPose = targets.get(closestTag).targetPose();
  
            SmartDashboard.putNumber("closestTagID", closestTagID);
            SmartDashboard.putNumber("ClosestTagPose", closest);
            SmartDashboard.putNumber("closestTagIndex", closestTag);
  
            // Determine which way to offset
            boolean invertSide = false;
            if ((closestTagID >= 20 && closestTagID <= 22)
                || (closestTagID >= 9 && closestTagID <= 11)) {
              invertSide = true;
            } else {
              invertSide = false;
            }
  
            // invert if on the other side of the field
            if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
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
                -forwardController.calculate(closestTagPose.getZ(), ReffOffsetForward);
            @SuppressWarnings("unused")
            double rotation =
                angleController.calculate(
                    swerve.getRotation().getRadians(), rotationTarget.getRadians());
            //  double rotation = 0;
  
            if (strafeController.atSetpoint()
                && forwardController.atSetpoint()
                && angleController.atSetpoint()) {
              swerve.setSwerveState(SwerveState.autoDriveDone);
            } else {
              swerve.setSwerveState(SwerveState.autoDriveInProgress);
            }
  
            ChassisSpeeds speeds = new ChassisSpeeds(forwadSpeed, strafeSpeed, 0);
            swerve.runVelocity(speeds);
          });
    }
}
