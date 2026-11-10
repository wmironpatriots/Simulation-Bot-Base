package org.frc6423.robot.subsystems.superstructure.arm;
import org.wpilib.simulation.SingleJointedArmSim;
import org.wpilib.driverstation.RobotState;

public class ArmIOSim extends ArmIO {
    private static final double kPeriodSeconds = 0.02;
    private static final double kMaxVoltage = 12.0;

    private final SingleJointedArmSim mArmSim =
            new SingleJointedArmSim(
                    org.wpilib.math.system.DCMotor.getKrakenX60Foc(1),
                    kArmGearRatio,
                    SingleJointedArmSim.estimateMOI(kArmLengthMeters, kArmMassKg),
                    kArmLengthMeters,
                    Math.toRadians(kArmMinAngleDegrees),
                    Math.toRadians(kArmMaxAngleDegrees),
                    true,
                    Math.toRadians(POSE_READY)
            );

    private final org.wpilib.math.controller.PIDController mPIDController = new org.wpilib.math.controller.PIDController(20.0, 0.0, 0.0);
    private final org.wpilib.math.controller.ArmFeedforward mFeedforward = new org.wpilib.math.controller.ArmFeedforward(0.0, 0.29, 0.0, 0.0);

    private double mAngleSetpointRadians = Math.toRadians(POSE_READY);
    private double mAppliedVoltage = 0.0;

    @Override
    public void periodic() {
        if (RobotState.isEnabled()) {
            double pidOutput = mPIDController.calculate(mArmSim.getAngle(), mAngleSetpointRadians);
            double feedforwardOutput = mFeedforward.calculate(mArmSim.getAngle(), 0.0);
            mAppliedVoltage = Math.clamp(pidOutput + feedforwardOutput, -kMaxVoltage, kMaxVoltage);
        } else {
            mPIDController.reset();
            mAppliedVoltage = 0.0;
        }
        
        mArmSim.setInputVoltage(mAppliedVoltage);
        mArmSim.update(kPeriodSeconds);
    }

    @Override
    public void setAngleSetpoint(double angleDegrees) {
        mAngleSetpointRadians = Math.toRadians(Math.clamp(angleDegrees, kArmMinAngleDegrees, kArmMaxAngleDegrees));
    }

    @Override
    public double getAngleSetpoint() {
        return Math.toDegrees(mAngleSetpointRadians);
    }

    @Override
    public double getAngleDegrees() {
        return Math.toDegrees(mArmSim.getAngle());
    }

    @Override
    public double getVelocity() {
        return Math.toDegrees(mArmSim.getVelocity());
    }

    @Override
    public double getAppliedVoltage() {
        return mAppliedVoltage;
    }

    @Override
    public void setBrakeMode(boolean brake) {
        // No-op for simulation
    }

    @Override
    public void zero() {
        mArmSim.setState(0.0, 0.0);
        mArmSim.setInputVoltage(0.0);
        mAngleSetpointRadians = 0.0;
        mAppliedVoltage = 0.0;
        mPIDController.reset();
    }

    @Override
    public void stop() {
        mPIDController.reset();
        mAppliedVoltage = 0.0;
        mArmSim.setInputVoltage(0.0);
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
