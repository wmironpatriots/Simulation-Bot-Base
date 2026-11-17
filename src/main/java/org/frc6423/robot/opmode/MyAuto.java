
package org.frc6423.robot.opmode;

import org.wpilib.command3.Scheduler;
import org.wpilib.opmode.Autonomous;
import org.wpilib.opmode.PeriodicOpMode;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;

import org.frc6423.robot.Robot;
import org.frc6423.robot.subsystems.drive.DriveSubsystem;

@Autonomous(name = "My Auto", group = "Group 1")
public class MyAuto extends PeriodicOpMode {
  private final Robot robot;
  private final DriveSubsystem mDrive;
  private final Command autoCommand;

  /** The Robot instance is passed into the opmode via the constructor. */
  public MyAuto(Robot robot) {
    this.robot = robot;
    this.mDrive = robot.getDriveSubsystem();
    this.autoCommand = null; 
  }

  /*
   * This method runs periodically, using the same period as the Robot instance.
   *
   * Additional periodic methods may be configured with addPeriodic(),
   * which can have periods that differ from the main Robot instance.
   */
  @Override
  public void periodic() {
    
  }

  @Override
  public void start() {
    // This method is called once when the opmode is started.
    Scheduler.getDefault().schedule(autoCommand);
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
