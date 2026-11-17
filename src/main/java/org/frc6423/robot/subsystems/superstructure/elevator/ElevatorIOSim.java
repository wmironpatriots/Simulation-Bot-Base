package org.frc6423.robot.subsystems.superstructure.elevator;
import org.frc6423.robot.subsystems.superstructure.arm.ArmIO;
import org.wpilib.simulation.ElevatorSim;
import org.wpilib.math.controller.ElevatorFeedforward;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.system.DCMotor;

public class ElevatorIOSim extends ElevatorIO {
    private static final double kPeriodSeconds = 0.02;
    private static final double kMaxVoltage = 12.0;

    private static final double kMovingMassKg = kElevatorCarriageMassKg + ArmIO.kArmMassKg;

    private final ElevatorSim mElevatorSim =
            new ElevatorSim(
                    DCMotor.getKrakenX60Foc(2),
                    kElevatorGearRatio, 
                    kMovingMassKg,
                    kElevatorDrumRadiusMeters, 
                    kElevatorMinHeightMeters, 
                    kElevatorMaxHeightMeters,
                    true,
                    0.1
                );
    
    private final PIDController mPIDController = new PIDController(50.0, 0.0, 0.0);
    private final ElevatorFeedforward mFeedforward = new ElevatorFeedforward(0.0, 0.37, 0.0, 0.0);
    
    private double mHeightSetpoint = 0.2;
    private double mAppliedVoltage = 0.0;
    
    @Override
    public void periodic() {
        double pidOutput = mPIDController.calculate(mElevatorSim.getPosition(), mHeightSetpoint);
        double feedforwardOutput = mFeedforward.calculate(mElevatorSim.getVelocity());

        mAppliedVoltage = Math.clamp(pidOutput + feedforwardOutput, -kMaxVoltage, kMaxVoltage);
        mElevatorSim.setInputVoltage(mAppliedVoltage);
        mElevatorSim.update(kPeriodSeconds);
    }

    @Override
    public void setHeightSetpoint(double heightMeters) {
        mHeightSetpoint = Math.clamp(heightMeters, kElevatorMinHeightMeters, kElevatorMaxHeightMeters);
    }

    @Override
    public double getHeightSetpoint() {
        return mHeightSetpoint;
    }

    @Override
    public double getHeightMeters() {
        return mElevatorSim.getPosition();
    }

    @Override
    public double getVelocity() {
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
        mElevatorSim.setState(0.0, 0.0);
        mHeightSetpoint = 0.0;
        mAppliedVoltage = 0.0;
    }

    @Override
    public void stop() {
        mElevatorSim.setInputVoltage(0.0);
        mAppliedVoltage = 0.0;
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
