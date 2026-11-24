package org.frc6423.robot.subsystems.elevator;

import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;

public class Elevator implements Mechanism {

    private final ElevatorIO io;
    private final ElevatorIO.ElevatorIOInputs inputs =
        new ElevatorIO.ElevatorIOInputs();

    public Elevator(ElevatorIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }

    public Command goToHeight(double heightMeters) {
        double target = Math.max(
            ElevatorConstants.kElevatorMinHeightMeters,
            Math.min(ElevatorConstants.kElevatorMaxHeightMeters, heightMeters)
        );

        return runOnce(() -> io.setPosition(target))
            .named("Elevator Go To Height");
    }

    public Command handoff() {
        return goToHeight(ElevatorConstants.POSE_HANDOFF);
    }

    public Command ready() {
        return goToHeight(ElevatorConstants.POSE_READY);
    }

    public Command scoreL4() {
        return goToHeight(ElevatorConstants.POSE_SCORE_L4);
    }

    public Command scoreL3() {
        return goToHeight(ElevatorConstants.POSE_SCORE_L3);
    }

    public Command scoreL2() {
        return goToHeight(ElevatorConstants.POSE_SCORE_L2);
    }
}