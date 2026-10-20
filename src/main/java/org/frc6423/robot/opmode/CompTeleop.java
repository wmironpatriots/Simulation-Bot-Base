// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc6423.robot.opmode;

import org.wpilib.command3.Trigger;
import org.wpilib.command3.button.CommandNiDsPS5Controller;
import org.wpilib.command3.button.CommandNiDsXboxController;
import org.wpilib.command3.button.RobotModeTriggers;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.opmode.PeriodicOpMode;
import org.wpilib.opmode.Teleop;
import org.frc6423.lib.util.InputStream;
import org.frc6423.lib.util.NodeSelector;
import org.frc6423.robot.Constants;
import org.frc6423.robot.Constants.Flags;
import org.frc6423.robot.Robot;
import org.frc6423.robot.subsystems.drive.DriveSubsystem;

@Teleop
public class CompTeleop extends PeriodicOpMode {
  private final Robot robot;
  private final DriveSubsystem mDrive;
  private final CommandNiDsPS5Controller mDriverController = new CommandNiDsPS5Controller(0);

  /** The Robot instance is passed into the opmode via the constructor. */
  public CompTeleop(Robot robot) {
    this.robot = robot;
    this.mDrive = robot.getDriveSubsystem();
    configureDriverBindings();
  }

  private void configureDriverBindings() {
    // ~~~ Drive Controls ~~~

    InputStream rawX =
        InputStream.of(mDriverController::getLeftY)
            .negate()
            .log("Telemetry/Operator Triggers/Raw Vx Vec");
    InputStream rawY =
        InputStream.of(mDriverController::getLeftX)
            .negate()
            .log("Telemetry/Operator Triggers/Raw Vy Vec");

    InputStream r =
        InputStream.hypot(rawX, rawY)
            .clamp(1.0)
            .deadband(0.02, 1.0)
            .signedPow(2.0)
            .scale(() -> Flags.kDrivetrainContants.getMaxLinearVelocityMetersPerSecond())
            .log("Telemetry/Operator Triggers/V Vec");

    InputStream theta = InputStream.atan(rawX, rawY);

    InputStream x = r.scale(theta.map(Math::cos)).log("Telemetry/Operator Triggers/Vx Vec");
    InputStream y = r.scale(theta.map(Math::sin)).log("Telemetry/Operator Triggers/Vy Vec");

    InputStream omega =
        InputStream.of(mDriverController::getRightX)
            .negate()
            .clamp(1.0)
            .deadband(0.02, 1.0)
            .signedPow(2.0)
            .scale(0.85)
            .scale(() -> Flags.kDrivetrainContants.getMaxAngularVelocityRadsPerSec())
            .log("Telemetry/Operator Triggers/Omega");
    double stickDeadband = 0.02;

    Trigger sticksIdle =
        new Trigger(
            () ->
                Math.hypot(
                        mDriverController.getLeftX(), mDriverController.getLeftY())
                    < stickDeadband
                    && Math.hypot(
                            mDriverController.getRightX(), mDriverController.getRightY())
                        < stickDeadband);

    Trigger sticksMoving = sticksIdle.negate();
    RobotModeTriggers.teleop().and(sticksMoving).whileTrue(mDrive.driveTeleoperated(x, y, omega));
    RobotModeTriggers.teleop()
        .and(sticksIdle)
        .onTrue(
            mDrive.driveSetTranslationalAndAngularSetpoint(
                () ->
                    NodeSelector.selectClosestNode(
                        Constants.Field.kReefScoringPoses, mDrive.getPose2d(), 0.8)));

  }

  @Override
  public void disabledPeriodic() {
    /* Called periodically (on every DS packet) while the robot is disabled. */
  }

  @Override
  public void start() {
    /* Called once when the robot is enabled. */
  }

  @Override
  public void periodic() {
    /* Called periodically (set time interval) while the robot is enabled. */
  }

  @Override
  public void end() {
    /* Called when the robot is disabled (after previously being enabled). */
  }

  @Override
  public void close() {
    /* Called when the opmode is de-selected / no additional methods will be called. */
  }
}
