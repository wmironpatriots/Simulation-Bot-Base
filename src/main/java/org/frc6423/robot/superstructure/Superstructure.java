
package org.frc6423.robot.superstructure;

import org.frc6423.robot.subsystems.arm.Arm;
import org.frc6423.robot.subsystems.elevator.Elevator;
import org.frc6423.robot.subsystems.intake.Intake;
import org.wpilib.command3.Command;
import org.wpilib.command3.StateMachine;
import org.wpilib.command3.Trigger;

public class Superstructure {
    
    private StateMachine.State stowedState;
    private StateMachine.State intakingState;
    private StateMachine.State scoringState;
    private final Arm arm;
    private final Elevator elevator;
    private final Intake intake;
    private final StateMachine stateMachine =
    new StateMachine("Superstructure");

    private static final double ARM_HANDOFF = 0.0;
    private static final double ARM_READY = 0.0;
    private static final double ARM_SCORE_L2 = 0.0;
    private static final double ARM_SCORE_L3 = 0.0;
    private static final double ARM_SCORE_L4 = 0.0;

    public Superstructure(Arm arm, Elevator elevator, Intake intake) {
        this.arm = arm;
        this.elevator = elevator;
        this.intake = intake;

         stowedState = stateMachine.addState(
        ready()
            .alongWith(stowIntake())
            .andThen(arm.run(() -> {}))
    );

    stateMachine.setInitialState(stowedState);
    intakingState = stateMachine.addState(
    elevator.handoff()
        .alongWith(arm.goToPosition(ARM_HANDOFF))
        .alongWith(intakePosition())
        .andThen(arm.run(() -> {}))
);
scoringState = stateMachine.addState(
    scoreL4()
        .andThen(arm.run(() -> {}))
);
    }

    public Arm getArm() {
        return arm;
    }

    public Elevator getElevator() {
        return elevator;
    }

    public Intake getIntake() {
        return intake;
    }

    public Command handoff() {
        return elevator.handoff()
            .alongWith(arm.goToPosition(ARM_HANDOFF))
            .alongWith(intake.handoff())
            .named("Superstructure Handoff");
    }

    public Command ready() {
        return elevator.ready()
            .alongWith(arm.goToPosition(ARM_READY))
            .named("Superstructure Ready");
    }

    public Command scoreL2() {
        return elevator.scoreL2()
            .alongWith(arm.goToPosition(ARM_SCORE_L2))
            .named("Superstructure Score L2");
    }

    public Command scoreL3() {
        return elevator.scoreL3()
            .alongWith(arm.goToPosition(ARM_SCORE_L3))
            .named("Superstructure Score L3");
    }

    public Command scoreL4() {
        return elevator.scoreL4()
            .alongWith(arm.goToPosition(ARM_SCORE_L4))
            .named("Superstructure Score L4");
    }

    public Command intake() {
        return intake.intake();
    }

    public Command intakePosition() {
        return intake.intakePosition();
    }

    public Command stowIntake() {
        return intake.stow();
    }
    public Command stateMachineCommand() {
    return stateMachine;
}
public void configureStateTransitions(
    Trigger intakeButton,
    Trigger scoreButton,
    Trigger stowButton
) {
    stowedState.switchTo(intakingState).when(intakeButton);
    stowedState.switchTo(scoringState).when(scoreButton);

    intakingState.switchTo(stowedState).when(stowButton);
    intakingState.switchTo(scoringState).when(scoreButton);

    scoringState.switchTo(stowedState).when(stowButton);
    scoringState.switchTo(intakingState).when(intakeButton);
}
}
