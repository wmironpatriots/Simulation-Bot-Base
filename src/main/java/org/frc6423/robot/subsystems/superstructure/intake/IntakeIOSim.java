package org.frc6423.robot.subsystems.superstructure.intake;

import org.wpilib.simulation.SingleJointedArmSim;

import com.ctre.phoenix6.ffi.StatusSignalNative.return_values_t;

import java.nio.charset.MalformedInputException;

import org.wpilib.driverstation.RobotState;

import org.wpilib.simulation.SingleJointedArmSim;

public class IntakeIOSim extends IntakeIO {
    private static final double kPeriodSeconds = 0.02;
    private static final double kMaxVoltage = 12.0;

    private final SingleJointedArmSim mIntakeSim = 
        new SingleJointedArmSim(
            org.wpilib.math.system.DCMotor.getKrakenX60Foc(1),
            kIntakeGearRatio,
            SingleJointedArmSim.estimateMOI(
                kIntakeLengthMeters,
                kIntakeMassKg
            ),
            kIntakeLengthMeters,
            Math.toRadians(kIntakeMinAngleDegrees),
            Math.toRadians(kIntakeMaxAngleDegrees),
            true,
            Math.toRadians(POSE_STOW)
        );
    
    private final org.wpilib.math.controller.PIDController mPIDController =
        new org.wpilib.math.controller.PIDController(20.0, 0.0, 0.0);
    
    private final org.wpilib.math.controller.ArmFeedforward mFeedforward = 
        new org.wpilib.math.controller.ArmFeedforward(0.0, 1.0, 0.0, 0.0);
    
    private double mAngleSetpointRadians = Math.toRadians(POSE_STOW);
    private double mAppliedVoltage = 0.0;

    @Override
    public void periodic() {
        if (RobotState.isEnabled()) {
            double pidOutput = mPIDController.calculate(
                mIntakeSim.getAngle(),
                mAngleSetpointRadians
            );

            double feedforwardOutput = mFeedforward.calculate(
                mIntakeSim.getAngle(),
                0.0
            );

            mAppliedVoltage = Math.clamp(
                pidOutput + feedforwardOutput,
                -kMaxVoltage,
                kMaxVoltage
            );

        } else {
            mPIDController.reset();
            mAppliedVoltage = 0.0;
        }

        mIntakeSim.setInputVoltage(mAppliedVoltage);
        mIntakeSim.update(kPeriodSeconds);
    }

    @Override
    public void setAngleSetpoint(double angleDegrees) {
        mAngleSetpointRadians = Math.toRadians(
            Math.clamp(
                angleDegrees,
                kIntakeMinAngleDegrees,
                kIntakeMaxAngleDegrees
            )
        );
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
    public void setBrakeMode(boolean brake) {

    }

    @Override
    public void zero() {
        mIntakeSim.setState(0.0, 0.0);
        mIntakeSim.setInputVoltage(0.0);

        mAngleSetpointRadians = 0.0;
        mAppliedVoltage = 0.0;

        mPIDController.reset();
    }

    @Override 
    public void stop() {
        mPIDController.reset();
        mAppliedVoltage = 0.0;
        mIntakeSim.setInputVoltage(0.0);
    }

    @Override
    public void setGains(
        double kS,
        double kG,
        double kV,
        double kA,
        double kP,
        double kI,
        double kD
    ) {
        mFeedforward.setKs(kS);
        mFeedforward.setKg(kG);
        mFeedforward.setKv(kV);
        mFeedforward.setKa(kA);

        mPIDController.setPID(kP, kI, kD);
    }
}
