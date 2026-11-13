
package org.frc6423.robot.opmode;

import org.wpilib.command3.Scheduler;
import org.wpilib.command3.Command;
import org.wpilib.opmode.Autonomous;
import org.wpilib.opmode.PeriodicOpMode;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;

import org.frc6423.robot.Robot;
import org.frc6423.robot.subsystems.drive.DriveSubsystem;

@Autonomous(name = "My Auto", group = "Group 1")
public class MyAuto extends PeriodicOpMode {

    private final DriveSubsystem mDrive;
    private final Command[] autoCommands;

    private int currentCommand = 0;

    public MyAuto(Robot robot) {
        mDrive = robot.getDriveSubsystem();

        // Movement 1: Drive forward 2 meters.
        Command driveForward =
            mDrive.driveSetTranslationalAndAngularSetpoint(
                new Pose2d(2.0, 0.0, Rotation2d.ZERO)
            );

        // Movement 2: Turn left 90 degrees.
        Command turnLeft =
            mDrive.driveSetTranslationalAndAngularSetpoint(
                new Pose2d(2.0, 0.0, Rotation2d.fromRadians(Math.PI / 2))
            );

        // Movement 3: Drive forward another meter.
        Command driveAgain =
            mDrive.driveSetTranslationalAndAngularSetpoint(
                new Pose2d(2.0, 1.0, Rotation2d.fromRadians(Math.PI / 2))
            );

        autoCommands = new Command[] {
            driveForward,
            turnLeft,
            driveAgain
        };
    }

    @Override
    public void periodic() {

        // Check whether the current movement has finished.
        if (currentCommand < autoCommands.length
                && mDrive.isAtTranslationalTarget()
                && mDrive.isFacingAngularTarget()) {

            // Stop the current movement.
            Scheduler.getDefault().cancel(
                autoCommands[currentCommand]
            );

            currentCommand++;

            // Start the next movement.
            if (currentCommand < autoCommands.length) {
                Scheduler.getDefault().schedule(
                    autoCommands[currentCommand]
                );
            } else {
                mDrive.stop();
            }
        }
    }

    @Override
    public void start() {
        currentCommand = 0;

        // Reset estimated position.
        mDrive.reset(new Pose2d());
        mDrive.resetFeedbackControllers();

        // Start the first movement.
        Scheduler.getDefault().schedule(autoCommands[0]);
    }

    @Override
    public void end() {
        // Cancel all autonomous movements.
        for (Command command : autoCommands) {
            Scheduler.getDefault().cancel(command);
        }

        mDrive.stop();
    }
}
