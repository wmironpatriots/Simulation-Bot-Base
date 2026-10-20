// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc6423.robot;

import org.frc6423.robot.Constants.Flags;
import org.frc6423.robot.subsystems.drive.DriveSubsystem;
import org.wpilib.epilogue.Epilogue;
import org.wpilib.epilogue.EpilogueConfiguration;
import org.wpilib.epilogue.Logged;
import org.wpilib.epilogue.Logged.Importance;
import org.wpilib.epilogue.logging.errors.ErrorHandler;
import static org.wpilib.units.Units.Seconds;
import org.wpilib.framework.OpModeRobot;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.system.DataLogManager;
import org.wpilib.command3.Scheduler;

/**
 * The methods in this class are called automatically as described in the OpModeRobot documentation.
 * OpMode classes anywhere in the package (or sub-packages) where this class is located are
 * automatically registered to display in the Driver Station. If you change the name of this class
 * or the package after creating this project, you must also update the Main.java file in the
 * project.
 */
public class Robot extends OpModeRobot {
  // * ~~~~~~~~ SUBSYSTEMS ~~~~~~~~

  @Logged(name = "Drive Subsystem")
  private final DriveSubsystem mDrive = DriveSubsystem.create();

  public DriveSubsystem getDriveSubsystem() {
    return mDrive;
  }

  /** Field-relative REEF scoring targets for the AdvantageScope field view. */
  @Logged(name = "Reef Scoring Poses", importance = Importance.INFO)
  public Pose2d[] getReefScoringPoses() {
    return Constants.Field.kReefScoringPoses;
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    // Configure Epilogue
    Epilogue.configure(
        (EpilogueConfiguration config) -> {
          // RobotBase publishes telemetry to /Telemetry and suppresses duplicates.
          config.root = "";

          // Start logging
          if (Robot.isReal()) {
            DataLogManager.start();
          }

          // Set error handling
          if (Robot.isSimulation()) {
            config.errorHandler = ErrorHandler.crashOnError();
          } else {
            config.errorHandler = ErrorHandler.printErrorMessages();
          }

          // Set lowest importance level to be logged
          config.minimumImportance = Flags.kLowestLoggingLevel;

          // OpModeRobot does not get Epilogue.bind(); schedule the generated
          // root logger explicitly from the robot loop instead.
          config.loggingPeriod = Seconds.of(0.02);
        });

    var epilogueConfig = Epilogue.getConfig();
    addPeriodic(
        () ->
            Epilogue.org_frc6423_robot_RobotLogger.tryUpdate(
                epilogueConfig.table.getTable(epilogueConfig.root),
                this,
                epilogueConfig.errorHandler),
        epilogueConfig.loggingPeriod.in(Seconds));
  }

  /** This function is called exactly once when the DS first connects. */
  @Override
  public void driverStationConnected() {}

  @Override
  public void simulationPeriodic() {
    mDrive.simulationPeriodic();
  }
  @Override
  public void robotPeriodic() {
    Scheduler.getDefault().run();
    mDrive.periodic();
  }
  /**
   * This function is called periodically anytime when no opmode is selected, including when the
   * Driver Station is disconnected.
   */
  @Override
  public void nonePeriodic() {}
}
