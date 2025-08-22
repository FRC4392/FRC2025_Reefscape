// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.swerve.SwerveConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.DeceiverRobotState;
import frc.robot.RobotConstants;
import frc.robot.RobotConstants.Mode;
import frc.robot.subsystems.swerve.SwerveConstants.ReefSide;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO.targetPoseObservation;
import frc.robot.util.LocalADStarAK;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Swerve extends SubsystemBase {
  private final DeceiverRobotState robotState;
  public static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final SwerveModule[] modules = new SwerveModule[4]; // FL, FR, BL, BR
  private final SysIdRoutine sysId;
  private final Alert gyroDisconnectedAlert =
      new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);
  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(moduleTranslations);
  private Rotation2d rawGyroRotation = new Rotation2d();
  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());

  private SwerveState state = SwerveState.other;

  /**
   * Create a new swerve subsystem
   *
   * @param gyroIO IO interface of the gyroscope
   * @param flModuleIO IO interface of the front left module
   * @param frModuleIO IO interface of the front right module
   * @param blModuleIO IO interface of the back left module
   * @param brModuleIO IO interface of the back right module
   * @param robotState Instance of the robot state
   */
  public Swerve(
      GyroIO gyroIO,
      SwerveModuleIO flModuleIO,
      SwerveModuleIO frModuleIO,
      SwerveModuleIO blModuleIO,
      SwerveModuleIO brModuleIO,
      DeceiverRobotState robotState) {
    this.gyroIO = gyroIO;
    modules[0] = new SwerveModule(flModuleIO, 0);
    modules[1] = new SwerveModule(frModuleIO, 1);
    modules[2] = new SwerveModule(blModuleIO, 2);
    modules[3] = new SwerveModule(brModuleIO, 3);

    this.robotState = robotState;

    // Start odometry thread
    SwerveOdometryThread.getInstance().start();

    // Configure AutoBuilder for PathPlanner
    AutoBuilder.configure(
        this::getPose,
        this::setPose,
        this::getChassisSpeeds,
        this::autoRunVelocity,
        new PPHolonomicDriveController(
            new PIDConstants(4, 0.0, 0.0), new PIDConstants(4, 0.0, 0.0)),
        ppConfig,
        () -> robotState.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        this);
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });

    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runCharacterization(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }
    odometryLock.unlock();

    // Stop moving when disabled
    if (robotState.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }

      // Log empty setpoint states when disabled
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Update odometry
    double[] sampleTimestamps =
        modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distanceMeters
                    - lastModulePositions[moduleIndex].distanceMeters,
                modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      // Update gyro angle
      if (gyroInputs.isConnected) {
        // Use the real gyro angle
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      // Apply update
      poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
    }

    // Update gyro alert
    gyroDisconnectedAlert.set(!gyroInputs.isConnected && RobotConstants.currentMode != Mode.SIM);
  }

  private void autoRunVelocity(ChassisSpeeds speeds) {
    setSwerveState(SwerveState.autoDriveInProgress);
    runVelocity(speeds);
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  private void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, maxSpeedMetersPerSec);

    // Log unoptimized setpoints
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

    // Send setpoints to modules
    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i]);
    }

    // Log optimized setpoints (runSetpoint mutates each state)
    Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
  }

  /** Runs the drive in a straight line with the specified drive output. */
  private void runCharacterization(double output) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(output);
    }
  }

  /** Stops the drive. */
  private void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /** Returns the module states (azimuth angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the current state of the swerve base on commands running the swerve */
  @AutoLogOutput(key = "Drive/State")
  public SwerveState getSwerveState() {
    return state;
  }

  /**
   * set the new state of the swerve drive
   *
   * @param newState new state
   */
  private void setSwerveState(SwerveState newState) {
    state = newState;
  }

  /** Returns the module positions ((azimuth angles and drive positions)) for all of the modules. */
  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /** Returns the measured chassis speeds of the robot. */
  @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  /** Returns the position of each module in radians. */
  private double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = modules[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }

  /** Returns the average velocity of the modules in rad/sec. */
  private double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += modules[i].getFFCharacterizationVelocity() / 4.0;
    }
    return output;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  // TODO: private?
  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  // TODO: private?
  public void resetGyro() {
    var pose = poseEstimator.getEstimatedPosition();
    var rotation =
        robotState.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
            ? new Rotation2d()
            : Rotation2d.k180deg;
    Pose2d newPose = new Pose2d(pose.getTranslation(), rotation);
    poseEstimator.resetPose(newPose);
  }

  /** Adds a new timestamped vision measurement. */
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    poseEstimator.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return maxSpeedMetersPerSec;
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return maxSpeedMetersPerSec / driveBaseRadius;
  }

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /// Commands
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(sysId.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
  }

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
  public Command joystickDrive(SwerveControlSignal signal) {
    return joystickDrive(
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
  public Command joystickDrive(
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      BooleanSupplier fastMode) {
    return run(
        () -> {
          setSwerveState(SwerveState.joystickDrive);
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
                  linearVelocity.getX() * getMaxLinearSpeedMetersPerSec(),
                  linearVelocity.getY() * getMaxLinearSpeedMetersPerSec(),
                  omega * getMaxAngularSpeedRadPerSec());

          boolean isFlipped =
              robotState.getAlliance().isPresent()
                  && robotState.getAlliance().get() == Alliance.Red;

          runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  speeds, isFlipped ? getRotation().plus(new Rotation2d(Math.PI)) : getRotation()));
        });
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
  public Command joystickDriveAtAngle(
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
    return run(() -> {
          setSwerveState(SwerveState.joystickDrive);
          double multiplier = fastMode.getAsBoolean() ? 1 : SLOW_SPEED_PERCENTAGE;
          // Get linear velocity
          Translation2d linearVelocity =
              getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble())
                  .times(multiplier);

          // Calculate angular speed
          double omega =
              angleController.calculate(
                  getRotation().getRadians(), rotationSupplier.get().getRadians());

          // Convert to field relative speeds & send command
          ChassisSpeeds speeds =
              new ChassisSpeeds(
                  linearVelocity.getX() * getMaxLinearSpeedMetersPerSec(),
                  linearVelocity.getY() * getMaxLinearSpeedMetersPerSec(),
                  omega);
          boolean isFlipped =
              robotState.getAlliance().isPresent()
                  && robotState.getAlliance().get() == Alliance.Red;
          runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  speeds, isFlipped ? getRotation().plus(new Rotation2d(Math.PI)) : getRotation()));
        })

        // Reset PID controller when command starts
        .beforeStarting(() -> angleController.reset(getRotation().getRadians()));
  }

  /**
   * Measures the velocity feedforward constants for the drive motors.
   *
   * <p>This command should only be used in voltage control mode.
   *
   * @param drive Swerve Drive dependency
   * @return Command to measure feedforward
   */
  public Command feedforwardCharacterization() {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();

    return Commands.sequence(
        // Reset data
        Commands.runOnce(
            () -> {
              velocitySamples.clear();
              voltageSamples.clear();
              setSwerveState(SwerveState.other);
            }),

        // Allow modules to orient
        run(() -> {
              runCharacterization(0.0);
            })
            .withTimeout(FF_START_DELAY),

        // Start timer
        Commands.runOnce(timer::restart),

        // Accelerate and gather data
        run(() -> {
              double voltage = timer.get() * FF_RAMP_RATE;
              runCharacterization(voltage);
              velocitySamples.add(getFFCharacterizationVelocity());
              voltageSamples.add(voltage);
            })

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
  public Command wheelRadiusCharacterization() {
    SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

    return Commands.parallel(
        // Drive control sequence
        Commands.sequence(
            // Reset acceleration limiter
            Commands.runOnce(
                () -> {
                  limiter.reset(0.0);
                  setSwerveState(SwerveState.other);
                }),

            // Turn in place, accelerating up to full speed
            run(
                () -> {
                  double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                  runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                })),

        // Measurement sequence
        Commands.sequence(
            // Wait for modules to fully orient before starting measurement
            Commands.waitSeconds(1.0),

            // Record starting measurement
            Commands.runOnce(
                () -> {
                  state.positions = getWheelRadiusCharacterizationPositions();
                  state.lastAngle = getRotation();
                  state.gyroDelta = 0.0;
                }),

            // Update gyro delta
            Commands.run(
                    () -> {
                      var rotation = getRotation();
                      state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                      state.lastAngle = rotation;
                    })

                // When cancelled, calculate and print results
                .finallyDo(
                    () -> {
                      double[] positions = getWheelRadiusCharacterizationPositions();
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
   * <p>The origin for the pose is on the blue side of the field with x pointing away from the
   * driverstation and z pointing up
   *
   * @param swerve swerve drive dependancy
   * @param pose pose to drive to
   * @return command to drive to the pose
   */
  public Command driveToPose(Pose2d pose) {
    return driveToPose(pose, () -> Alliance.Blue);
  }

  /**
   * Drive straight to specified pose
   *
   * <p>If red alliance is specified the path will be flipped so the same pose goes to the same
   * point regardless of alliace color
   *
   * @param swerve swerve drive dependancy
   * @param pose pose to drive to
   * @param alliance current alliance color
   * @return command to drive to that pose
   */
  public Command driveToPose(Pose2d pose, Supplier<Alliance> alliance) {
    return run(
        () -> {
          // TODO write pose flipping code
        });
  }

  /**
   * Use Pathplanner to find a path to a specific pose.
   *
   * <p>If red alliance is specified the path will be flipped so the same pose goes to the same
   * point regardless of alliace color
   *
   * @param pose The pose to pathfind to
   * @param endVelocity The end velocity to end the path with
   * @param alliance The current alliance color
   * @return Command to pathfind to that pose
   */
  public Command pathfindToPose(
      Pose2d pose, LinearVelocity endVelocity, Supplier<Alliance> allianceSupplier) {
    return Commands.either(
        AutoBuilder.pathfindToPoseFlipped(pose, pathConstraints, endVelocity),
        AutoBuilder.pathfindToPose(pose, pathConstraints, endVelocity),
        () -> allianceSupplier.get() == Alliance.Red);
  }

  /**
   * Use Pathplanner to find a path to a specific pose.
   *
   * <p>The origin for the pose is on the blue side of the field with x pointing away from the
   * driverstation and z pointing up
   *
   * @param pose The pose to pathfind to
   * @param endVelocity The end velocity to end the path with
   * @return Command to pathfind to that pose
   */
  public Command pathfindToPose(Pose2d pose, LinearVelocity endVelocity) {
    return AutoBuilder.pathfindToPose(pose, pathConstraints, endVelocity);
  }

  /**
   * Use Pathplanner to find a path to the start point of a path, then follow that path
   *
   * @param path path to pathfind to and then follow
   * @return command to pathfind and then follow a path
   */
  public Command pathfindToPathThenFollow(PathPlannerPath path) {
    return AutoBuilder.pathfindThenFollowPath(path, pathConstraints);
  }

  /**
   * Stop the swerve drive and position the wheels in an X pattern
   *
   * @param swerve Swerve Drive dependancy
   * @return Command to stop the drivetrain
   */
  public Command stopWithX() {
    return run(
        () -> {
          Rotation2d[] headings = new Rotation2d[4];
          for (int i = 0; i < 4; i++) {
            headings[i] = moduleTranslations[i].getAngle();
          }
          kinematics.resetHeadings(headings);
          stop();
          setSwerveState(SwerveState.stopWithX);
        });
  }

  // Year Specific commands

  // Bad
  public Command autoAlignCommand3D(Vision vision, ReefSide side) {
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
    return run(
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
            runVelocity(new ChassisSpeeds());
            setSwerveState(SwerveState.autoDriveFail);
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
          if (robotState.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
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
              getRotation()
                  .plus(closestTagPose.getRotation().toRotation2d().minus(Rotation2d.k180deg));

          // Position to offset position
          double strafeSpeed = strafeController.calculate(closestTagPose.getX(), positionOffset);
          double forwadSpeed =
              -forwardController.calculate(closestTagPose.getZ(), ReffOffsetForward);
          @SuppressWarnings("unused")
          double rotation =
              angleController.calculate(getRotation().getRadians(), rotationTarget.getRadians());
          //  double rotation = 0;

          if (strafeController.atSetpoint()
              && forwardController.atSetpoint()
              && angleController.atSetpoint()) {
            setSwerveState(SwerveState.autoDriveDone);
          } else {
            setSwerveState(SwerveState.autoDriveInProgress);
          }

          ChassisSpeeds speeds = new ChassisSpeeds(forwadSpeed, strafeSpeed, 0);
          runVelocity(speeds);
        });
  }
}
