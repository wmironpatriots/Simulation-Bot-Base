// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot;

import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Meters;

import com.ctre.phoenix6.CANBus;
import org.wpilib.epilogue.Logged.Importance;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.shape.Rectangle2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.units.measure.Distance;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchState;
import org.frc6423.robot.subsystems.drive.constant.RebuiltL2;
import org.frc6423.robot.subsystems.drive.constant.SwerveConstants;

public final class Constants {
  public static final class Flags {
    public static final boolean kInitializeTunables = true;

    public static final boolean kInitializeSimulatedFuelPools = true;

    public static final SwerveConstants kDrivetrainContants = new RebuiltL2();

    public static final Importance kLowestLoggingLevel = Importance.DEBUG;

    public static Alliance getRobotAlliance() {
      return MatchState.getAlliance().orElse(Alliance.BLUE);
    }

    public static Rotation2d getAllianceRotation() {
      return Rotation2d.fromRotations(getRobotAlliance() == Alliance.BLUE ? 0.0 : 0.5);
    }
  }

  public static final class Field {
    public static final Distance kFieldLength = Inches.of(651.22);

    public static final Distance kFieldWidth = Inches.of(317.69);

    /**
     * Common robot-alignment adjustment for every REEF scoring pose.
     *
     * <p>The translation is in each pose's local coordinate frame: positive X is outward from the
     * REEF and positive Y is to the pose's left. The rotation is added to each pose's heading.
     */
    public static final Transform2d kReefScoringPoseAdjustment =
        new Transform2d(new Translation2d(0.75, 0.0), Rotation2d.ZERO);

    // The REEF poses below use the official 2025 welded-field coordinate system. Keep these
    // dimensions separate from the current season's field dimensions used elsewhere in this file.
    private static final Distance kReefscapeFieldLength = Meters.of(17.548);
    private static final Distance kReefscapeFieldWidth = Meters.of(8.052);
    private static final Translation2d kBlueReefCenter =
        new Translation2d(Inches.of(176.746), kReefscapeFieldWidth.div(2.0));

    /**
     * Nominal poses of the 12 coral branches on each REEF, in field coordinates.
     *
     * <p>The first 12 entries are the blue-side REEF, ordered clockwise by face with the two
     * branches on each face adjacent. The final 12 entries are the red-side REEF, mirrored across
     * the center of the 2025 field. The rotation is the outward-facing normal of each reef face.
     * These are branch-center poses; robot alignment offsets should be applied by the drivetrain.
     */
    private static final Pose2d[] kNominalReefScoringPoses = createReefScoringPoses();

    private static Pose2d[] createReefScoringPoses() {
      Pose2d[] poses = new Pose2d[24];

      // Official 2025 REEF geometry: branch centers are 30.738 inches from the REEF center and
      // 6.469 inches to either side of each face center. Faces start toward the blue driver station
      // and proceed clockwise.
      for (int face = 0; face < 6; face++) {
        Rotation2d heading = Rotation2d.fromDegrees(180.0 - 60.0 * face);
        Pose2d faceFrame = new Pose2d(kBlueReefCenter, heading);

        poses[face * 2] =
            faceFrame.transformBy(
                new Transform2d(
                    new Translation2d(Inches.of(30.738), Inches.of(6.469)), Rotation2d.ZERO));
        poses[face * 2 + 1] =
            faceFrame.transformBy(
                new Transform2d(
                    new Translation2d(Inches.of(30.738), Inches.of(-6.469)), Rotation2d.ZERO));
      }

      // Generate the red REEF from the blue REEF using the official 2025 field center.
      Translation2d fieldCenter =
          new Translation2d(kReefscapeFieldLength.div(2.0), kReefscapeFieldWidth.div(2.0));
      for (int i = 0; i < 12; i++) {
        poses[i + 12] =
            new Pose2d(
                poses[i].getTranslation().rotateAround(fieldCenter, Rotation2d.k180deg),
                poses[i].getRotation().plus(Rotation2d.k180deg));
      }

      return poses;
    }

    /**
     * REEF scoring poses with {@link #kReefScoringPoseAdjustment} applied in each pose's local
     * frame.
     */
    public static final Pose2d[] kReefScoringPoses =
        java.util.Arrays.stream(kNominalReefScoringPoses)
            .map(pose -> pose.transformBy(kReefScoringPoseAdjustment))
            .toArray(Pose2d[]::new);

    public static final Distance kAllianceZoneLength = Inches.of(182.11);

    public static final Pose2d kMidPose =
        new Pose2d(kFieldLength.div(2), kFieldWidth.div(2), Rotation2d.ZERO);

    public static Pose2d kBlueAllianceZonePose2d =
        new Pose2d(kAllianceZoneLength.div(2), kFieldWidth.div(2), Rotation2d.ZERO);

    public static Pose2d kBlueAllianceHubPose2d =
        new Pose2d(Inches.of(182.11), Inches.of(158.84), Rotation2d.ZERO);

    public static Pose2d getHubPose2d() {
      return getRobotAlliancePose2d(kBlueAllianceHubPose2d);
    }

    public static Rectangle2d getAllianceZone() {
      return new Rectangle2d(
          getRobotAlliancePose2d(kBlueAllianceZonePose2d), kAllianceZoneLength, kFieldWidth);
    }

    public static Pose2d getRobotAlliancePose2d(Pose2d pose) {
      return (Flags.getRobotAlliance() == Alliance.BLUE)
          ? pose
          : new Pose2d(
              pose.getTranslation().rotateAround(kMidPose.getTranslation(), Rotation2d.k180deg),
              pose.getRotation().plus(Rotation2d.k180deg));
    }
  }

  public static final class Matrix {
    public static final CANBus kDriveCanBus = new CANBus("DRIVE");

    public static final int kDriveFrPivotId = 1;
    public static final int kDriveFrEncoderId = 9;
    public static final int kDriveFrDriveId = 2;

    public static final int kDriveBrPivotId = 7;
    public static final int kDriveBrEncoderId = 12;
    public static final int kDriveBrDriveId = 8;

    public static final int kDriveFlPivotId = 3;
    public static final int kDriveFlEncoderId = 10;
    public static final int kDriveFlDriveId = 4;

    public static final int kDriveBlPivotId = 5;
    public static final int kDriveBlEncoderId = 11;
    public static final int kDriveBlDriveId = 6;

    public static final int kDriveGyroId = 13;

    public static final CANBus kSubsystemCanBus = new CANBus("SOUP");
  }
}
