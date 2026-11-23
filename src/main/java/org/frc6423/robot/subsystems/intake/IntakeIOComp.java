package org.frc6423.robot.subsystems.intake;

public class IntakeIOComp implements IntakeIO {

    public IntakeIOComp() {
        // TalonFX configuration will go here.
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        // Read motor velocity here.
    }

    @Override
    public void setVoltage(double voltage) {
        // Send voltage request to TalonFX here.
    }

    @Override
    public void stop() {
        // Stop the intake motor here.
    }
} 