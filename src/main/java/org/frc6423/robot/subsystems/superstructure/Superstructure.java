package org.frc6423.robot.subsystems.superstructure;

import org.frc6423.robot.subsystems.superstructure.arm.ArmIO;
import org.frc6423.robot.subsystems.superstructure.arm.ArmIOSim;

import org.frc6423.robot.subsystems.superstructure.intake.IntakeIO;
import org.frc6423.robot.subsystems.superstructure.intake.IntakeIOSim;

import org.frc6423.robot.subsystems.superstructure.elevator.ElevatorIO;
import org.frc6423.robot.subsystems.superstructure.elevator.ElevatorIOSim;

import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.epilogue.Logged;

import org.wpilib.command3.StateMachine;
import java.util.function.BooleanSupplier;

public class Superstructure implements Mechanism {
    
    @Logged(name = "Arm")
    private final ArmIO mArm;

    @Logged(name = "Intake")
    private final IntakeIO mIntake;

    @Logged(name = "Elevator")
    private final ElevatorIO mElevator;

    private Superstructure(ArmIO mArm, IntakeIO intake, ElevatorIO elevator) {
        this.mArm = mArm;
        this.mIntake = intake;
        this.mElevator = elevator;
    }

    public static Superstructure create() {
        ArmIO arm = new ArmIOSim();
        IntakeIO intake = new IntakeIOSim();
        ElevatorIO elevator = new ElevatorIOSim();

        return new Superstructure(arm, intake, elevator);
    }

    public StateMachine createStateMachine(
        BooleanSupplier stowedTrigger,
        BooleanSupplier intakingTrigger,
        BooleanSupplier scoringTrigger
    ) {
    StateMachine sm = new StateMachine("Superstructure State Machine");
    var stowedState = sm.addState(stowedCommand());
    var intakingState = sm.addState(intakingCommand());
    var scoringState = sm.addState(scoringCommand());

    stowedState.switchTo(intakingState).when(intakingTrigger);
    stowedState.switchTo(scoringState).when(scoringTrigger);
    intakingState.switchTo(stowedState).when(stowedTrigger);
    intakingState.switchTo(scoringState).when(scoringTrigger);
    scoringState.switchTo(stowedState).when(stowedTrigger);
    scoringState.switchTo(intakingState).when(intakingTrigger);

    sm.setInitialState(stowedState);

    return sm;
    }


    public void periodic() {
        mArm.periodic();
        mIntake.periodic();
        mElevator.periodic();
    }

    public Command stowedCommand(){
        return run(coroutine -> {
            mIntake.setAngleSetpoint(IntakeIO.POSE_STOW);
            mArm.setAngleSetpoint(ArmIO.POSE_READY);
            mElevator.setHeightSetpoint(ElevatorIO.POSE_READY);
            coroutine.yield();
        }).named("Stow Superstructure");
    }

    public Command intakingCommand() {
    return run(coroutine -> {
        while (true) {
            mIntake.setAngleSetpoint(IntakeIO.POSE_INTAKE);
            mArm.setAngleSetpoint(ArmIO.POSE_READY);
            mElevator.setHeightSetpoint(ElevatorIO.POSE_READY);
            coroutine.yield();
        }
    }).named("INTAKING");
}  

    public Command scoringCommand() {
        return run(coroutine -> {
            while(true){
                mIntake.setAngleSetpoint(IntakeIO.POSE_STOW);
                mArm.setAngleSetpoint(ArmIO.POSE_SCORE_L4);
                mElevator.setHeightSetpoint(ElevatorIO.POSE_SCORE_L4);
                coroutine.yield();
            }
        }).named("SCORING");
    }




    public Command setPoseReady() {
        return run(coroutine -> {
            mArm.setAngleSetpoint(ArmIO.POSE_READY);
        }).named("Set Superstructure to Ready Pose");
    }

    public Command setIntakePosition(double angleDegrees) {
        return run(coroutine -> {
            mIntake.setAngleSetpoint(angleDegrees);
        }).named("Set Intake Position");
    }

    public Command intake() {
        return setIntakePosition(IntakeIO.POSE_INTAKE);
    }

    public Command stowIntake() {
        return setIntakePosition(IntakeIO.POSE_STOW);
    }

    public Command handoffIntake() {
        return setIntakePosition(IntakeIO.POSE_HANDOFF);
    }

    public Command setElevatorPosition(double heightMeters) {
        return run(coroutine -> {
            mElevator.setHeightSetpoint(heightMeters);
        }).named("Set Elevator Position");
    }

    public Command elevatorReady() {
        return setElevatorPosition(ElevatorIO.POSE_READY);
    }

    public Command elevatorHandoff() {
        return setElevatorPosition(ElevatorIO.POSE_HANDOFF);
    }

    public Command elevatorScoreL2() {
        return setElevatorPosition(ElevatorIO.POSE_SCORE_L2);
    }

    public Command elevatorScoreL3() {
        return setElevatorPosition(ElevatorIO.POSE_SCORE_L3);
    }

    public Command elevatorScoreL4() {
        return setElevatorPosition(ElevatorIO.POSE_SCORE_L4);
    }
}
