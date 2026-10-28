// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc6423.robot.opmode;

import org.wpilib.command3.Scheduler;
import edu.wpi.first.wpilibj2.command.Command;
import org.wpilib.opmode.Autonomous;
import org.wpilib.opmode.PeriodicOpMode;
import org.frc6423.robot.Robot;
import org.frc6423.robot.subsystems.drive.DriveSubsystem;

@Autonomous(name = "My Auto", group = "Group 1")
public class MyAuto extends PeriodicOpMode {
  private final Robot robot;
  private final DriveSubsystem mDrive;
  private final Command autoCommand;
  private final org.wpilib.command.CommandScheduler commands;

  /** The Robot instance is passed into the opmode via the constructor. */
  public MyAuto(Robot robot) {
    this.robot = robot;
    this.commands = new org.wpilib.command.CommandScheduler();
    this.autoCommand = edu.wpi.first.wpilibj2.command.Commands.run(() -> mDrive.drive(0.5, 0), mDrive);
    this.autoCommand = commands.run(() -> robot.getDriveSubsystem().stop(), robot.getDriveSubsystem());
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
    autoCommand.schedule();
  }

  @Override
  public void end() {
    // This method is called once when the opmode ends.
  }
}
