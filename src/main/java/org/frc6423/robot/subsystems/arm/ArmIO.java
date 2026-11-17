package org.frc6423.robot.subsystems.arm;

public interface ArmIO {

    class ArmIOInputs {
        public double positionRadians = 0.0;
        public double velocityRadiansPerSecond = 0.0;
        public boolean motorConnected = false;
    }

    default void updateInputs(ArmIOInputs inputs) {}

    default void setPosition(double positionRadians) {}

    default void stop() {}
} 