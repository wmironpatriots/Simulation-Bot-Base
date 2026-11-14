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

    public void periodic() {
        mArm.periodic();
        mIntake.periodic();
        mElevator.periodic();
    }

    public Command setPoseReady() {
        return run(coroutine -> {
            mIntake.setAngleSetpoint(IntakeIO.POSE_STOW);
            mArm.setAngleSetpoint(ArmIO.POSE_READY);
            mElevator.setHeightSetpoint(ElevatorIO.POSE_READY);
        }).named("Set Superstructure to Ready Pose");
    }

    public Command setPoseHandoff() {
        return run(coroutine -> {
            mIntake.setAngleSetpoint(IntakeIO.POSE_HANDOFF);
            mArm.setAngleSetpoint(ArmIO.POSE_HANDOFF);
            mElevator.setHeightSetpoint(ElevatorIO.POSE_HANDOFF);
        }).named("Set Superstructure to Handoff Pose");
    }
    public Command setPoseScoreL2() {
        return run(coroutine -> {
            mIntake.setAngleSetpoint(IntakeIO.POSE_STOW);
            mArm.setAngleSetpoint(ArmIO.POSE_SCORE_L2);
            mElevator.setHeightSetpoint(ElevatorIO.POSE_SCORE_L2);
        }).named("Set Superstructure to Score L2 Pose");
    }

    public Command setPoseScoreL3() {
        return run(coroutine -> {
            mIntake.setAngleSetpoint(IntakeIO.POSE_STOW);
            mArm.setAngleSetpoint(ArmIO.POSE_SCORE_L3);
            mElevator.setHeightSetpoint(ElevatorIO.POSE_SCORE_L3);
        }).named("Set Superstructure to Score L3 Pose");
    }

    public Command setPoseScoreL4() {
        return run(coroutine -> {
            mIntake.setAngleSetpoint(IntakeIO.POSE_STOW);
            mArm.setAngleSetpoint(ArmIO.POSE_SCORE_L4);
            mElevator.setHeightSetpoint(ElevatorIO.POSE_SCORE_L4);
        }).named("Set Superstructure to Score L4 Pose");
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
    private Command setArmPosition(double angleDegrees) {
    return run(coroutine -> {
        mArm.setAngleSetpoint(angleDegrees);
    }).named("Set Arm Position");
}
    public Command armHandoff() {
        return setArmPosition(ArmIO.POSE_HANDOFF);
    }
    public Command armReady() {
        return setArmPosition(ArmIO.POSE_READY);
    }
    public Command armScoreL2() {
        return setArmPosition(ArmIO.POSE_SCORE_L2);
    }

    public Command armScoreL3() {
        return setArmPosition(ArmIO.POSE_SCORE_L3);
    }

    public Command armScoreL4() {
        return setArmPosition(ArmIO.POSE_SCORE_L4);
    }
    public Command armAfterL2() {
        return setArmPosition(ArmIO.POSE_AFTER_L2);
    }
    public Command armAfterL3() {
        return setArmPosition(ArmIO.POSE_AFTER_L3);
    }
    public Command armAfterL4() {
        return setArmPosition(ArmIO.POSE_AFTER_L4);
    }
}
