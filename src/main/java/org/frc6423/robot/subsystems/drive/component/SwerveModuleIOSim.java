// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystems.drive.component;

import org.wpilib.math.controller.PIDController;
import org.wpilib.math.controller.SimpleMotorFeedforward;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.Models;
import org.wpilib.math.util.Units;
import org.wpilib.simulation.DCMotorSim;
import org.frc6423.robot.subsystems.drive.constant.SwerveConstants.ModuleConfig;

public class SwerveModuleIOSim extends SwerveModuleIO {
  private static final DCMotor kDriveModel = DCMotor.getKrakenX60Foc(1);
  private static final DCMotor kPivotModel = DCMotor.getKrakenX60Foc(1);

  private final DCMotorSim mPivotSim, mDriveSim;
  private final PIDController mPivotController = new PIDController(0.0, 0.0, 0.0);
  private final PIDController mDriveController = new PIDController(0.01, 0.0, 0.0);
  private final SimpleMotorFeedforward mDriveFeedforward = new SimpleMotorFeedforward(0.0, 0.0);

  private double mPivotAppliedVolts, mDriveAppliedVolts, mDriveFeedforwardVolts;

  public SwerveModuleIOSim(ModuleConfig config) {
    super(config);

    mPivotSim =
        new DCMotorSim(
            Models.singleJointedArmFromPhysicalConstants(
                kPivotModel,
                0.004,
                mConfig.pivotConfig().Feedback.SensorToMechanismRatio
                    * mConfig.pivotConfig().Feedback.RotorToSensorRatio),
            kPivotModel);

    mDriveSim =
        new DCMotorSim(
            Models.singleJointedArmFromPhysicalConstants(
                kDriveModel, 0.025, mConfig.driveConfig().Feedback.SensorToMechanismRatio),
            kDriveModel);

    mPivotController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void periodic() {
    super.periodic();
  }

  @Override
  public double getPivotAppliedVolts() {
    return mPivotSim.getInputVoltage();
  }

  @Override
  public double getPivotSupplyCurrentAmps() {
    return mPivotSim.getCurrentDraw();
  }

  @Override
  public double getPivotStatorCurrentAmps() {
    return 0.0;
  }

  @Override
  public double getPivotSupplyTorqueAmps() {
    return 0.0;
  }

  @Override
  public double getPivotTemperatureCelsius() {
    return 0.0;
  }

  @Override
  public double getPivotAngleRevs() {
    return Units.radiansToRotations(mPivotSim.getAngularPosition());
  }

  @Override
  public void runPivotCharacterizationVoltage(double volts) {}

  @Override
  public void runPivotCharacterizationCurrent(double amps) {}

  @Override
  protected void setPivotAngleSetpoint(double angleRevs) {
    mPivotAppliedVolts =
        Math.clamp(
            mPivotController.calculate(
                mPivotSim.getAngularPosition(), Units.rotationsToRadians(angleRevs)),
            -12.0,
            12.0);

    mPivotSim.setInputVoltage(mPivotAppliedVolts);
    mPivotSim.update(0.02);
  }

  @Override
  public double getDriveAppliedVolts() {
    return mDriveSim.getInputVoltage();
  }

  @Override
  public double getDriveSupplyCurrentAmps() {
    return mDriveSim.getCurrentDraw();
  }

  @Override
  public double getDriveStatorCurrentAmps() {
    return 0.0;
  }

  @Override
  public double getDriveSupplyTorqueAmps() {
    return 0.0;
  }

  @Override
  public double getDriveTemperatureCelsius() {
    return 0.0;
  }

  @Override
  public double getDriveAngleRevs() {
    return Units.radiansToRotations(mDriveSim.getAngularPosition());
  }

  @Override
  public double getDriveAngularSpeedRevsPerSec() {
    return Units.radiansToRotations(mDriveSim.getAngularVelocity());
  }

  @Override
  public void runDriveCharacterizationVoltage(double volts) {}

  @Override
  public void runDriveCharacterizationCurrent(double amps) {}

  @Override
  protected void setDriveSpeedSetpoint(double speedRevsPerSec, boolean focEnabled) {
    mDriveAppliedVolts =
        Math.clamp(
            // mDriveController.calculate(mDriveSim.getAngularVelocityRadPerSec())
            0.0 + mDriveFeedforward.calculate(Units.rotationsToRadians(speedRevsPerSec)),
            -12.0,
            12.0);
    mDriveSim.setInputVoltage(mDriveAppliedVolts);
    mDriveSim.update(0.02);
  }

  @Override
  protected void setDriveSpeedSetpoint(double speedRevsPerSec, double torqueNm) {
    setDriveSpeedSetpoint(speedRevsPerSec, true);
  }

  @Override
  public void neutral() {
    mPivotAppliedVolts = 0.0;
    mDriveAppliedVolts = 0.0;
  }

  @Override
  protected void setPivotGains(double kS, double kV, double kA, double kP, double kD) {
    mPivotController.setPID(kP, 0.0, kD);
  }

  @Override
  protected void setDriveTorqueGains(double kS, double kV, double kA, double kP, double kD) {
    mDriveController.setPID(kP, 0.0, kD);
    mDriveFeedforward.setKs(kS);
    mDriveFeedforward.setKv(kV);
    mDriveFeedforward.setKa(kA);
  }

  @Override
  protected void setDriveVoltGains(double kS, double kV, double kA, double kP, double kD) {}
}
