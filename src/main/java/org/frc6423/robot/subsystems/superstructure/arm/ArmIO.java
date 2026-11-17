package org.frc6423.robot.subsystems.superstructure.arm;

import org.wpilib.epilogue.Logged;
import org.wpilib.epilogue.Logged.Importance;

public abstract class ArmIO {
    public static final double kArmMinAngleDegrees = -350.0;
    public static final double kArmMaxAngleDegrees = 350.0;
    public static final double kArmLengthMeters = 0.6;
    public static final double kArmMassKg = 5.0;
    public static final double kArmGearRatio = 64.5;

    public static final double POSE_HANDOFF = 270.0;
    public static final double POSE_READY = 90.0;
    public static final double POSE_SCORE_L4 = 65.0;
    public static final double POSE_AFTER_L4 = 60.0;
    public static final double POSE_SCORE_L3 = 50.0;
    public static final double POSE_AFTER_L3 = 45.0;
    public static final double POSE_SCORE_L2 = 50.0;
    public static final double POSE_AFTER_L2 = 45.0;

    public abstract void setAngleSetpoint(double angleDegrees);

    @Logged(name = "Arm Angle Setpoint", importance = Importance.INFO)
    public abstract double getAngleSetpoint();

    @Logged(name = "Arm Angle", importance = Importance.INFO)
    public abstract double getAngleDegrees();

    @Logged(name = "Arm Velocity", importance = Importance.INFO)
    public abstract double getVelocity();

    @Logged(name = "Arm Applied Voltage", importance = Importance.INFO)
    public abstract double getAppliedVoltage();

    public abstract void setBrakeMode(boolean brake);

    public abstract void zero();

    public abstract void periodic();

    public abstract void stop();

    public abstract void setGains(double kS, double kG, double kV, double kA, double kP, double kI, double kD);
}
