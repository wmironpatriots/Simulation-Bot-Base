package org.frc6423.robot.subsystems.superstructure.elevator;

import org.wpilib.epilogue.Logged;
import org.wpilib.epilogue.Logged.Importance;

public abstract class ElevatorIO {
    public static final double kElevatorMinHeightMeters = 0.2;
    public static final double kElevatorMaxHeightMeters = 1.6;
    public static final double kElevatorDrumRadiusMeters = 0.01905;
    public static final double kElevatorCarriageMassKg = 8.0;
    public static final double kElevatorGearRatio = 4.0;

    public static final double POSE_HANDOFF = 1.1;
    public static final double POSE_READY = 0.6;
    public static final double POSE_SCORE_L4 = 1.3;
    public static final double POSE_AFTER_L4 = 1.2;
    public static final double POSE_SCORE_L3 = 0.7;
    public static final double POSE_AFTER_L3 = 0.6;
    public static final double POSE_SCORE_L2 = 0.5;
    public static final double POSE_AFTER_L2 = 0.4;

    public abstract void setHeightSetpoint(double heightMeters);

    @Logged(name = "Elevator Height Setpoint (m)", importance = Importance.INFO)
    public abstract double getHeightSetpoint();
    
    @Logged(name = "Elevator Height (m)", importance = Importance.INFO)
    public abstract double getHeightMeters();
    
    @Logged(name = "Elevator Velocity (m per sec)", importance = Importance.INFO)
    public abstract double getVelocityMetersPerSecond();

    @Logged(name = "Elevator Applied Voltage (V)", importance = Importance.INFO)
    public abstract double getAppliedVoltage();

    @Logged(name = "Elevator Height Error (m)", importance = Importance.INFO)
    public double getHeightErrorMeters() {
        return getHeightSetpoint() - getHeightMeters();
    }

    public abstract void setBrakeMode(boolean brake);
    
    public abstract void periodic();

    public abstract void stop();

    public abstract void zero();

    public abstract void setGains(
        double kS, double kG, double kV, double kA,
        double kP, double kI, double kD
    );
}
