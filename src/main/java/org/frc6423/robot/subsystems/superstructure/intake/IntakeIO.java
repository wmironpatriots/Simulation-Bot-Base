package org.frc6423.robot.subsystems.superstructure.intake;

import org.wpilib.epilogue.Logged;
import org.wpilib.epilogue.Logged.Importance;

public abstract class IntakeIO {
    public static final double kIntakeMinAngleDegrees = -45.0;
    public static final double kIntakeMaxAngleDegrees = 270.0;
    public static final double kIntakeLengthMeters = 0.5;
    public static final double kIntakeMassKg = 5.0;
    public static final double kIntakeGearRatio = 20.0;

    public static final double POSE_HANDOFF = 140.0;
    public static final double POSE_STOW = 0.0;
    public static final double POSE_INTAKE = 0.0;
    
    public abstract void setAngleSetpoint(double angleDegrees);

    @Logged(name = "Intake Angle Setpoint (deg)", importance = Importance.INFO)
    public abstract double getAngleSetpoint();

    @Logged(name = "Intake Angle (deg)", importance = Importance.INFO)
    public abstract double getAngleDegrees();

    @Logged(name = "Intake Velocity (deg per sec)", importance = Importance.INFO)
    public abstract double getVelocity();

    @Logged(name = "Intake Applied Voltage (V)", importance = Importance.INFO)
    public abstract double getAppliedVoltage();

    @Logged(name = "Intake Angle Error (deg)", importance = Importance.INFO)
    public double getAngleErrorDegrees() {
        return getAngleSetpoint() - getAngleDegrees();
    }

    public abstract void setBrakeMode(boolean brake);

    public abstract void zero();

    public abstract void periodic();

    public abstract void stop();

    public abstract void setGains(
            double kS,
            double kG,
            double kV,
            double kA,
            double kP,
            double kI,
            double kD
    );
}
