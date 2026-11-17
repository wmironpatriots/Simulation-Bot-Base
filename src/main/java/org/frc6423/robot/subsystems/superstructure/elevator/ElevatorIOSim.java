package org.frc6423.robot.subsystems.superstructure.elevator;

import org.wpilib.simulation.ElevatorSim;
import org.wpilib.driverstation.RobotState;

public class ElevatorIOSim extends ElevatorIO {
    private static final double kPeriodSeconds = 0.02;
    private static final double kMaxVoltage = 12.0;

    private final ElevatorSim mElevatorSim =
        new ElevatorSim(
            org.wpilib.math.system.DCMotor.getKrakenX60Foc(1),
            kElevatorGearRatio,
            kElevatorCarriageMassKg,
            kElevatorDrumRadiusMeters,
            kElevatorMinHeightMeters,
            kElevatorMaxHeightMeters,
            true,
            POSE_READY
        );

    private final org.wpilib.math.controller.PIDController mPIDController =
        new org.wpilib.math.controller.PIDController(20.0, 0.0, 0.0);
    
    private final org.wpilib.math.controller.ElevatorFeedforward mFeedforward = 
        new org.wpilib.math.controller.ElevatorFeedforward(0.0, 0.63, 0.0, 0.0);
    
    private double mHeightSetpointMeters = POSE_READY;
    private double mAppliedVoltage = 0.0;

    @Override
    public void periodic() {
        if (RobotState.isEnabled()) {
            double pidOutput = mPIDController.calculate(
                mElevatorSim.getPosition(),
                mHeightSetpointMeters
            );

            double feedforwardOutput = 
                mFeedforward.calculate(0.0);

            mAppliedVoltage = Math.clamp(
                pidOutput + feedforwardOutput,
                -kMaxVoltage,
                kMaxVoltage
            );
        } else {
            mPIDController.reset();
            mAppliedVoltage = 0.0;
        }

        mElevatorSim.setInputVoltage(mAppliedVoltage);
        mElevatorSim.update(kPeriodSeconds);
    }

    @Override
    public void setHeightSetpoint(double heightMeters) {
        mHeightSetpointMeters = Math.clamp(
            heightMeters,
            kElevatorMinHeightMeters,
            kElevatorMaxHeightMeters
        );
    }

    @Override
    public double getHeightSetpoint() {
        return mHeightSetpointMeters;
    } 

    @Override
    public double getHeightMeters() {
        return mElevatorSim.getPosition();
    }

    @Override
    public double getVelocityMetersPerSecond() {
        return mElevatorSim.getVelocity();
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
        mElevatorSim.setState(kElevatorMinHeightMeters, 0.0);
        mElevatorSim.setInputVoltage(0.0);

        mHeightSetpointMeters = kElevatorMinHeightMeters;
        mAppliedVoltage = 0.0;

        mPIDController.reset();
    }

    @Override
    public void stop() {
        mPIDController.reset();
        mAppliedVoltage = 0.0;
        mElevatorSim.setInputVoltage(0.0);
    }

    public void setGains(
        double kS, double kG, double kV, double kA,
        double kP, double kI, double kD
    ) {
        mFeedforward.setKs(kS);
        mFeedforward.setKg(kG);
        mFeedforward.setKv(kV);
        mFeedforward.setKa(kA);

        mPIDController.setPID(kP, kI, kD);
    }
}
