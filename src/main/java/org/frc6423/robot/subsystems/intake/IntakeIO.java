
package org.frc6423.robot.subsystems.intake;

public interface IntakeIO {

    class IntakeIOInputs {
        public double velocity = 0.0;
        public double positionRadians = 0.0;
        public double velocityRadiansPerSecond = 0.0;
        public boolean motorConnected = false;
    }

    default void updateInputs(IntakeIOInputs inputs) {}

    default void setVoltage(double voltage) {}

    default void setPosition(double positionRadians) {}

    default void stop() {}
}
