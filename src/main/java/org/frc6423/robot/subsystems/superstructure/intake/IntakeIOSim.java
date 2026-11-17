package org.frc6423.robot.subsystems.superstructure.intake;
import java.util.function.BooleanSupplier;

import org.wpilib.simulation.SingleJointedArmSim;

public class IntakeIOSim extends IntakeIO {
    private static final double kPeriodSeconds = 0.02;
    private static final double kMaxVoltage = 12.0;

    private final SingleJointedArmSim mIntakeSim =
            new SingleJointedArmSim(
                    org.wpilib.math.system.DCMotor.getKrakenX60Foc(1),
                    kIntakeGearRatio,
                    SingleJointedArmSim.estimateMOI(kIntakeLengthMeters, kIntakeMassKg),
                    kIntakeLengthMeters,
                    Math.toRadians(kIntakeMinAngleDegrees),
                    Math.toRadians(kIntakeMaxAngleDegrees),
                    true,
                    Math.toRadians(0.0)
            );

    private final org.wpilib.math.controller.PIDController mPIDController = new org.wpilib.math.controller.PIDController(10.0, 0.0, 0.4);
    private final org.wpilib.math.controller.ArmFeedforward mFeedforward = new org.wpilib.math.controller.ArmFeedforward(0.0, 0.29, 0.0, 0.0);

    private double mAngleSetpointRadians = Math.toRadians(180.0);
    private double mAppliedVoltage = 0.0;
    private boolean mHasCoral = false;

    @Override
    public void periodic() {
        double pidOutput = mPIDController.calculate(mIntakeSim.getAngle(), mAngleSetpointRadians);
        double feedforwardOutput = mFeedforward.calculate(mIntakeSim.getAngle(), mIntakeSim.getVelocity());
        
        mAppliedVoltage = Math.clamp(pidOutput + feedforwardOutput, -kMaxVoltage, kMaxVoltage);
        
        mIntakeSim.setInputVoltage(mAppliedVoltage);
        mIntakeSim.update(kPeriodSeconds);
    }

    @Override
    public void setAngleSetpoint(double angleDegrees) {
        mAngleSetpointRadians = Math.toRadians(Math.clamp(angleDegrees, kIntakeMinAngleDegrees, kIntakeMaxAngleDegrees));
    }

    @Override
    public double getAngleSetpoint() {
        return Math.toDegrees(mAngleSetpointRadians);
    }

    @Override
    public double getAngleDegrees() {
        return Math.toDegrees(mIntakeSim.getAngle());
    }

    @Override
    public double getVelocity() {
        return Math.toDegrees(mIntakeSim.getVelocity());
    }

    @Override
    public double getAppliedVoltage() {
        return mAppliedVoltage;
    }

    @Override
    public BooleanSupplier hasCoral() {
        return () -> mHasCoral;
    }

    public void setHasCoral(boolean hasCoral) {
        mHasCoral = hasCoral;
    }

    @Override
    public void setBrakeMode(boolean brake) {
        // No-op for simulation
    }

    @Override
    public void zero() {
        mIntakeSim.setState(Math.toRadians(180), 0.0);
        mIntakeSim.setInputVoltage(0.0);
        mAngleSetpointRadians = 180.0;
        mAppliedVoltage = 0.0;
    }

    @Override
    public void stop() {
        mAppliedVoltage = 0.0;
        mIntakeSim.setInputVoltage(0.0);
    }

    @Override
    public void setGains(double kS, double kG, double kV, double kA, double kP, double kI, double kD) {
        mFeedforward.setKs(kS);
        mFeedforward.setKg(kG);
        mFeedforward.setKv(kV);
        mFeedforward.setKa(kA);
        mPIDController.setPID(kP, kI, kD);
    }
}
