// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystems.drive;


import org.wpilib.epilogue.Logged;
import org.wpilib.epilogue.Logged.Importance;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.controller.ProfiledPIDController;
import org.wpilib.math.estimator.SwerveDrivePoseEstimator;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Pose3dJsonAdapter;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N3;
import org.wpilib.math.trajectory.TrapezoidProfile.Constraints;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.smartdashboard.Field2d;
import org.wpilib.telemetry.Telemetry;
import java.lang.reflect.Array;
import java.util.Arrays;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.frc6423.lib.util.Tracer;
import org.frc6423.lib.util.TunableNumber;
import org.frc6423.robot.Constants.Flags;
import org.frc6423.robot.Robot;
import org.frc6423.robot.subsystems.drive.component.GyroIO;
import org.frc6423.robot.subsystems.drive.component.GyroIONone;
import org.frc6423.robot.subsystems.drive.component.GyroIOPigeon2;
import org.frc6423.robot.subsystems.drive.component.SwerveModuleIO;
import org.frc6423.robot.subsystems.drive.component.SwerveModuleIOComp;
import org.frc6423.robot.subsystems.drive.component.SwerveModuleIOSim;
import org.frc6423.robot.subsystems.drive.constant.SwerveConstants;

/** {@link Mechanism} manager class for the swerve drivetrain. */
public class DriveSubsystem implements Mechanism {
  @Override
  public String getName() {
    return "Drive";
  }

  /**
   * Static Factory for automatically configuring and creating a {@link DriveSubsystem}
   *
   * @return {@link DriveSubsystem}
   */
  public static DriveSubsystem create() {
    return (Robot.isReal())
        ? new DriveSubsystem(
            new GyroIOPigeon2(kConstants.getGyroConfig()),
            new SwerveModuleIOComp(kConstants.getFrontLeftModuleConfig()),
            new SwerveModuleIOComp(kConstants.getFrontRightModuleConfig()),
            new SwerveModuleIOComp(kConstants.getBackLeftModuleConfig()),
            new SwerveModuleIOComp(kConstants.getBackRightModuleConfig()))
        : new DriveSubsystem(
            new GyroIONone(),
            new SwerveModuleIOSim(kConstants.getFrontLeftModuleConfig()),
            new SwerveModuleIOSim(kConstants.getFrontRightModuleConfig()),
            new SwerveModuleIOSim(kConstants.getBackLeftModuleConfig()),
            new SwerveModuleIOSim(kConstants.getBackRightModuleConfig()));
  }

  // * ~~~~~~~~ CONSTANTS ~~~~~~~~

  /** {@link SwerveConstants} Physical/hardware constants to initialize subsystem with */
  public static final SwerveConstants kConstants = Flags.kDrivetrainContants;

  /** {@link Lock} Mutex for high frequency odometry */
  public static Lock kLock = new ReentrantLock();

  /** {@link String} The directory to store tunables in NT */
  public static final String kTunablesPrefix = "/Drive";

  // * ~~~~~~~~ TUNNABLES ~~~~~~~~

  private static final TunableNumber kTranslationalFeedbackKp =
      new TunableNumber(kTunablesPrefix + "/Trans kP");
  private static final TunableNumber kTranslationalFeedbackKi =
      new TunableNumber(kTunablesPrefix + "/Trans kI");
  private static final TunableNumber kTranslationalFeedbackKd =
      new TunableNumber(kTunablesPrefix + "/Trans kD");
  private static final TunableNumber kTranslationalFeedbackTolerance =
      new TunableNumber(kTunablesPrefix + "/tolerance (centimeters)");

  private static final TunableNumber kAngularFeedbackKp =
      new TunableNumber(kTunablesPrefix + "/Angular kP");
  private static final TunableNumber kAngularFeedbackKi =
      new TunableNumber(kTunablesPrefix + "/Angular kI");
  private static final TunableNumber kAngularFeedbackKd =
      new TunableNumber(kTunablesPrefix + "/Angular kD");
  private static final TunableNumber kAngularFeebackToleranceDeg =
      new TunableNumber(kTunablesPrefix + "/tolerance (degrees)");

  static {
    if (Robot.isReal()) {
      // kTranslationalFeedbackKp.initDefault(10.0);
      // kTranslationalFeedbackKi.initDefault(0.01);
      // kTranslationalFeedbackKd.initDefault(0.02);
      // kTranslationalFeedbackTolerance.initDefault(2.54);
      kTranslationalFeedbackKp.initDefault(8.0);
      kTranslationalFeedbackKi.initDefault(0.0);
      kTranslationalFeedbackKd.initDefault(0.0);
      kTranslationalFeedbackTolerance.initDefault(2.54);

      // kAngularFeedbackKp.initDefault(5.0);
      // kAngularFeedbackKi.initDefault(0.0);
      // kAngularFeedbackKd.initDefault(0.08);
      kAngularFeedbackKp.initDefault(4.0);
      kAngularFeedbackKi.initDefault(0.0);
      kAngularFeedbackKd.initDefault(0.0);
      kAngularFeebackToleranceDeg.initDefault(2.0);
    } else {
      kTranslationalFeedbackKp.initDefault(10.0);
      kTranslationalFeedbackKi.initDefault(0.01);
      kTranslationalFeedbackKd.initDefault(0.02);
      kTranslationalFeedbackTolerance.initDefault(2.54);

      kAngularFeedbackKp.initDefault(5.0);
      kAngularFeedbackKi.initDefault(0.0);
      kAngularFeedbackKd.initDefault(0.08);
      kAngularFeebackToleranceDeg.initDefault(2.0);
    }
  }

  // * ~~~~~~~~ MEMBERS ~~~~~~~~

  /** {@link GyroIO} Hardware Interface for the gyro */
  private final GyroIO mGyro;

  /** {@link Rotation2d} Simulated robot rotation for sim */
  private Rotation2d mSimRotation = Rotation2d.ZERO;

  /** {@link SwerveModuleIO} Hardware Interface for the front right swerve module */
  @Logged(name = "Front Right")
  private final SwerveModuleIO mFrontRightModule;

  /** {@link SwerveModuleIO} Hardware Interface for the back right swerve module */
  @Logged(name = "Back Right")
  private final SwerveModuleIO mBackRightModule;

  /** {@link SwerveModuleIO} Hardware Interface for the front left swerve module */
  @Logged(name = "Front Left")
  private final SwerveModuleIO mFrontLeftModule;

  /** {@link SwerveModuleIO} Hardware Interface for the back left swerve module */
  @Logged(name = "Back Left")
  private final SwerveModuleIO mBackLeftModule;

  /**
   * {@link Array} of {@link SwerveModuleIO} Array storing all {@link SwerveModuleIO} hardware for
   * easier manipulation
   *
   * <p>Order is Front Right, Back Right, Front Left, Back Left
   */
  private final SwerveModuleIO[] mModules;

  /**
   * {@link Array} of {@link SwerveModuleState} Array for storing setpoint {@link SwerveModuleIO}
   * states for logging
   *
   * <p>Order is Front Right, Back Right, Front Left, Back Left
   */
  private SwerveModuleVelocity[] mSetpointWheelStates =
      new SwerveModuleVelocity[] {
        new SwerveModuleVelocity(),
        new SwerveModuleVelocity(),
        new SwerveModuleVelocity(),
        new SwerveModuleVelocity()
      };

  /**
   * {@link SwerveDrivePoseEstimator} Odometry class for estimating the position of robot using
   * vision/drivetrain measurements
   */
  private final SwerveDrivePoseEstimator mPoseEstimator;

  /** {@link Field2d} Member for visualizing relevant positions on Elastic */
  private final Field2d mF2d;

  /**
   * {@link ProfiledPIDController} Feedback controller for translational assists in the x direction
   */
  @Logged(name = "Translational X")
  private final ProfiledPIDController mTranslationalXController =
      new ProfiledPIDController(
          kTranslationalFeedbackKp.get(),
          kTranslationalFeedbackKi.get(),
          kTranslationalFeedbackKd.get(),
          new Constraints(
              kConstants.getMaxLinearVelocityMetersPerSecond(),
              kConstants.getMaxLinearAccelerationMetersPerSecondPerSecond()));

  /**
   * {@link ProfiledPIDController} Feedback controller for translational assists in the y direction
   */
  @Logged(name = "Translational Y")
  private final ProfiledPIDController mTranslationalYController =
      new ProfiledPIDController(
          kTranslationalFeedbackKp.get(),
          kTranslationalFeedbackKi.get(),
          kTranslationalFeedbackKd.get(),
          new Constraints(
              kConstants.getMaxLinearVelocityMetersPerSecond(),
              kConstants.getMaxLinearAccelerationMetersPerSecondPerSecond()));

  /** {@link ProfiledPIDController} Feedback controller for angular assists */
  @Logged(name = "Angular")
  private final PIDController mAngularController =
      new PIDController(
          kAngularFeedbackKp.get(), kAngularFeedbackKi.get(), kAngularFeedbackKd.get());

  // private final ProfiledPIDController mAngularController =
  //     new ProfiledPIDController(
  //         kAngularFeedbackKp.get(),
  //         kAngularFeedbackKi.get(),
  //         kAngularFeedbackKd.get(),
  //         new Constraints(8.989, 90.090));

  // new Constraints(
  //     kConstants.getMaxAngularVelocityRadsPerSec(),
  //     kConstants.getAngularAccelerationRadsPerSecPerSec()));

  protected DriveSubsystem(
      GyroIO gyro,
      SwerveModuleIO frontRight,
      SwerveModuleIO backRight,
      SwerveModuleIO frontLeft,
      SwerveModuleIO backLeft) {
    mGyro = gyro;

    mFrontRightModule = frontRight;
    mBackRightModule = backRight;
    mFrontLeftModule = frontLeft;
    mBackLeftModule = backLeft;

    mModules =
        new SwerveModuleIO[] {
          mFrontRightModule, mBackRightModule, mFrontLeftModule, mBackLeftModule
        };

    mPoseEstimator =
        new SwerveDrivePoseEstimator(
            kConstants.getKinematics(),
            new Rotation2d(),
            new SwerveModulePosition[] {
              new SwerveModulePosition(),
              new SwerveModulePosition(),
              new SwerveModulePosition(),
              new SwerveModulePosition()
            },
            new Pose2d());

    mAngularController.enableContinuousInput(0.0, 2 * Math.PI);

    mF2d = new Field2d();
  }

  public void periodic() {
    for (var module : mModules) {
      module.periodic();
    }

    Tracer.traceFunc(
        "Update Odometry",
        () -> {
          mPoseEstimator.update(getGyroRotation2d(), getWheelPositions());
        });

    mF2d.setRobotPose(getPose2d());
    Telemetry.log("Field", mF2d);

    // Update tunables
    if (kTranslationalFeedbackKp.hasChanged(hashCode())
        || kTranslationalFeedbackKi.hasChanged(hashCode())
        || kTranslationalFeedbackKd.hasChanged(hashCode())
        || kAngularFeedbackKp.hasChanged(hashCode())
        || kAngularFeedbackKi.hasChanged(hashCode())
        || kAngularFeedbackKd.hasChanged(hashCode())) {
      mTranslationalXController.setPID(
          kTranslationalFeedbackKp.get(),
          kTranslationalFeedbackKi.get(),
          kTranslationalFeedbackKd.get());
      mTranslationalYController.setPID(
          kTranslationalFeedbackKp.get(),
          kTranslationalFeedbackKi.get(),
          kTranslationalFeedbackKd.get());

      mAngularController.setPID(
          kAngularFeedbackKp.get(), kAngularFeedbackKi.get(), kAngularFeedbackKd.get());

      resetFeedbackControllers();
    }
  }

  public void simulationPeriodic() {
    mSimRotation =
        mSimRotation.rotateBy(
            Rotation2d.fromRadians(
                    !Double.isNaN(getChassisVelocities().omega)
                    ? getChassisVelocities().omega * 0.02
                    : 0));
  }

  // * ~~~~~~~~ GETTERS ~~~~~~~~

  /**
   * Check if robot has approximately reached current setpoint translational position
   *
   * @return {@link Boolean}
   */
  @Logged(name = "Within Translational Tolerance (bool)", importance = Importance.INFO)
  public boolean isAtTranslationalTarget() {
    return mTranslationalXController.atSetpoint() && mTranslationalYController.atSetpoint();
  }

  /**
   * Check if robot has approximately reached current setpoint angular position
   *
   * @return {@link Boolean}
   */
  @Logged(name = "Within Angular Tolerance (bool)", importance = Importance.INFO)
  public boolean isFacingAngularTarget() {
    return mAngularController.atSetpoint();
  }

  /**
   * Get estimated yaw rotation of robot
   *
   * @return {@link Rotation2dd}
   */
  public Rotation2d getGyroRotation2d() {
    return (Robot.isReal()) ? mGyro.getYawRotation2d() : mSimRotation;
  }

  public Rotation2d getRotation2d() {
    return getPose2d().getRotation();
  }

  /**
   * Get estimated position of robot in 2-Dimensional space (x, y)
   *
   * @return {@link Pose2d}
   */
  @Logged(name = "Field Position (Pose2d)", importance = Importance.INFO)
  public Pose2d getPose2d() {
    return mPoseEstimator.getEstimatedPosition();
  }

  /**
   * Get measured speed (meters per second)
   *
   * @return {@link Double}
   */
  @Logged(name = "Speed (meters per second)", importance = Importance.INFO)
  public double getSpeedMetersPerSecond() {
    return Math.hypot(getChassisVelocities().vx, getChassisVelocities().vy);
  }

  /**
   * Get velocity components of robot WRT field (x, y, omega rate)
   *
   * @return {@link ChassisVelocities}
   */
  public ChassisVelocities getChassisVelocitiesWrtField() {
    return getChassisVelocities().toFieldRelative(getRotation2d());
  }

  /**
   * Get velocity components of robot (dx, dy, omega rate)
   *
   * @return {@link ChassisVelocities}
   */
  @Logged(name = "Velocity Components (ChassisVelocities)", importance = Importance.INFO)
  public ChassisVelocities getChassisVelocities() {
    return kConstants.getKinematics().toChassisVelocities(getWheelStates());
  }

  /**
   * Get array measured wheel positions
   *
   * <p>Order is Front Right, Back Right, Front Left, Back Left
   *
   * @return {@link Array} of {@link SwerveModulePosition}
   */
  @Logged(name = "Wheel Positions (SwerveModulePosition)", importance = Importance.INFO)
  public SwerveModulePosition[] getWheelPositions() {
    return Arrays.stream(mModules)
        .map(SwerveModuleIO::getWheelPosition)
        .toArray(SwerveModulePosition[]::new);
  }

  /**
   * Get array measured wheel states (velocity vectors)
   *
   * <p>Order is Front Right, Back Right, Front Left, Back Left
   *
   * @return {@link Array} of {@link SwerveModuleState}
   */
  @Logged(name = "Swerve Module Velocity States (SwerveModuleState)", importance = Importance.INFO)
  public SwerveModuleVelocity[] getWheelStates() {
    return Arrays.stream(mModules)
        .map(SwerveModuleIO::getWheelState)
        .toArray(SwerveModuleVelocity[]::new);
  }

  /**
   * Get array setpoint wheel states (velocity vectors)
   *
   * <p>Order is Front Right, Back Right, Front Left, Back Left
   *
   * @return {@link Array} of {@link SwerveModuleState}
   */
  @Logged(
      name = "Setpoint Swerve Module Velocity States (SwerveModuleState)",
      importance = Importance.INFO)
  public SwerveModuleVelocity[] getWheelSetpointStates() {
    return mSetpointWheelStates;
  }

  // * ~~~~~~~~ SETTERS ~~~~~~~~

  public void addVisionMeasurement(Pose2d estimate, double timestamp, Matrix<N3, N1> stdevs) {
    mPoseEstimator.addVisionMeasurement(estimate, timestamp, stdevs);
  }

  public void resetFeedbackControllers() {
    mTranslationalXController.reset(
        getPose2d().getX(), getChassisVelocitiesWrtField().vx);
    mTranslationalYController.reset(
        getPose2d().getY(), getChassisVelocitiesWrtField().vy);
    mAngularController.reset();
    // mAngularController.reset(
    //     getRotation2d().getRadians(), getChassisVelocitiesWrtField().omega);
  }

  /**
   * Set setpoint {@link ChassisVelocities} for drivetrain to follow
   *
   * @param setpoint {@link ChassisVelocities} Desired drivetrain speeds
   */
  public void setChassisVelocitiesSetpoint(ChassisVelocities setpoint) {
    // Generate a time-specific setpoint from continous-time speeds
    setpoint = setpoint.discretize(0.02);

    // Convert to SwerveModuleState setpoints & clamp their velocities
    var states = kConstants.getKinematics().toSwerveModuleVelocities(setpoint);
    states =
        SwerveDriveKinematics.desaturateWheelVelocities(
            states, kConstants.getMaxLinearVelocityMetersPerSecond());

    // Auto FOC Toggle calculations;
    // if drivetrain is under 90% of max speed, FOC should be enabled
    // to maximize acceleration
    var focEnabled =
        getSpeedMetersPerSecond()
            < (kConstants.getMaxLinearVelocityMetersPerSecond()
                * kConstants.getFocAutoToggleMagnitude());

    // Send setpoints
    setSetpointWheelStates(states, focEnabled);
  }

  public void setSetpointWheelStates(SwerveModuleVelocity[] desiredStates, double[] wheelTorquesNm) {
    mSetpointWheelStates = desiredStates;

    for (int i = 0; i < mModules.length; i++) {
      desiredStates[i] = mModules[i].setSetpoint(desiredStates[i], wheelTorquesNm[i]);
    }
  }

  public void setSetpointWheelStates(SwerveModuleVelocity[] desiredStates, boolean focEnabled) {
    mSetpointWheelStates = desiredStates;

    for (int i = 0; i < mModules.length; i++) {
      desiredStates[i] = mModules[i].setSetpoint(desiredStates[i], focEnabled);
    }
  }

  public void reset(Pose2d pose) {
    mPoseEstimator.resetPosition(getGyroRotation2d(), getWheelPositions(), pose);
  }

  /** Stop drivetrain completely */
  public void stop() {
    mSetpointWheelStates =
        new SwerveModuleVelocity[] {
          new SwerveModuleVelocity(),
          new SwerveModuleVelocity(),
          new SwerveModuleVelocity(),
          new SwerveModuleVelocity()
        };

    for (int i = 0; i < mModules.length; i++) {
      mModules[i].neutral();
    }
  }

  // * ~~~~~~~~ COMMANDS ~~~~~~~~

  public Command driveTeleoperatedFacingTarget(
      DoubleSupplier vx, DoubleSupplier vy, Supplier<Translation2d> target, boolean flipped) {
    return driveTeleoperatedWithAngularAsisst(
        vx,
        vy,
        () ->
            target
                .get()
                .minus(getPose2d().getTranslation())
                .getAngle()
                .orElse(getRotation2d())
                .plus(flipped ? Rotation2d.k180deg : Rotation2d.ZERO));
  }

  public Command driveTeleoperatedWithAngularAsisst(
      DoubleSupplier vx, DoubleSupplier vy, Supplier<Rotation2d> angle) {
    return run(
            coroutine -> {
              resetFeedbackControllers();
              while (true) {
                setChassisVelocitiesSetpoint(
                    new ChassisVelocities(
                            vx.getAsDouble(),
                            vy.getAsDouble(),
                            mAngularController.calculate(
                                getRotation2d().getRadians(), angle.get().getRadians()))
                        .toRobotRelative(getRotation2d().plus(Flags.getAllianceRotation())));
                coroutine.yield();
              }
            })
        .whenCanceled(this::stop)
        .named("Drive Teleoperated Facing Angle");
  }

  public Command driveSetTranslationalAndAngularSetpoint(Pose2d setpoint) {
    return driveSetTranslationalAndAngularSetpoint(() -> setpoint);
  }

  /**
   * Drives to a pose selected when this command is scheduled.
   *
   * <p>The supplier is intentionally evaluated once per command run, so a changing selection source
   * cannot switch targets partway through an alignment.
   */
  public Command driveSetTranslationalAndAngularSetpoint(Supplier<Pose2d> setpointSupplier) {
    return run(
            coroutine -> {
              Pose2d setpoint = setpointSupplier.get();
              if (setpoint == null) {
                return;
              }
              resetFeedbackControllers();
              while (true) {
                setChassisVelocitiesSetpoint(
                    new ChassisVelocities(
                            mTranslationalXController.calculate(
                                getPose2d().getX(), setpoint.getX()),
                            mTranslationalYController.calculate(
                                getPose2d().getY(), setpoint.getY()),
                            mAngularController.calculate(
                                getRotation2d().getRadians(),
                                setpoint.getRotation().getRadians()))
                        .toRobotRelative(getRotation2d()));
                coroutine.yield();
              }
            })
        .whenCanceled(this::stop)
        .named("Drive Translational and Angular Setpoint");
  }

  public Command driveTeleoperated(DoubleSupplier vx, DoubleSupplier vy, DoubleSupplier omega) {
    return runRepeatedly(
            () ->
                setChassisVelocitiesSetpoint(
                    new ChassisVelocities(vx.getAsDouble(), vy.getAsDouble(), omega.getAsDouble())
                        .toRobotRelative(getRotation2d().plus(Flags.getAllianceRotation()))))
        .whenCanceled(this::stop)
        .named("Drive Teleoperated");
  }
}
