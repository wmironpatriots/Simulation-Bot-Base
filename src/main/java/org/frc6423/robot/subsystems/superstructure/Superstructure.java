package org.frc6423.robot.subsystems.superstructure;
import static org.wpilib.units.Units.Seconds;

import org.frc6423.lib.util.NodeSelector;
import org.frc6423.robot.subsystems.superstructure.arm.ArmIO;
import org.frc6423.robot.subsystems.superstructure.arm.ArmIOSim;
import org.frc6423.robot.subsystems.superstructure.elevator.ElevatorIO;
import org.frc6423.robot.subsystems.superstructure.elevator.ElevatorIOSim;
import org.frc6423.robot.subsystems.superstructure.intake.IntakeIO;
import org.frc6423.robot.subsystems.superstructure.intake.IntakeIOSim;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Command;
import org.wpilib.epilogue.Logged;
import org.wpilib.epilogue.Logged.Importance;

public class Superstructure implements Mechanism {

    public static Superstructure create() {
        ArmIO armIO = new ArmIOSim();
        ElevatorIO elevatorIO = new ElevatorIOSim();
        IntakeIO intakeIO = new IntakeIOSim();
        return new Superstructure(armIO, elevatorIO, intakeIO);
    }
    @Logged private final ArmIO mArm;
    @Logged private final ElevatorIO mElevator;
    @Logged private final IntakeIO mIntake;

    protected Superstructure(ArmIO arm, ElevatorIO elevator, IntakeIO intake) {
        mElevator = elevator;
        mArm = arm;
        mIntake = intake;
    }

    public void periodic() {
        mElevator.periodic();
        mArm.periodic();
        mIntake.periodic();
    }

    public void stop() {
        mElevator.stop();
        mArm.stop();
        mIntake.stop();
    }

    public Command setPoseHandoff() {
        return run(coroutine -> {
            mElevator.setHeightSetpoint(ElevatorIO.POSE_HANDOFF);
            mArm.setAngleSetpoint(ArmIO.POSE_HANDOFF);
            mIntake.setAngleSetpoint(IntakeIO.POSE_HANDOFF);
        }).named("Set Superstructure to Handoff Pose");
    }

    public Command setPoseReady() {
        return run(coroutine -> {
            mElevator.setHeightSetpoint(ElevatorIO.POSE_READY);
            mArm.setAngleSetpoint(ArmIO.POSE_READY);
        }).named("Set Superstructure to Ready Pose");
    }

    public Command setPoseScoreL4() {
        return run(coroutine -> {
            mElevator.setHeightSetpoint(ElevatorIO.POSE_SCORE_L4);
            mArm.setAngleSetpoint(ArmIO.POSE_SCORE_L4);
        }).named("Set Superstructure to Score L4 Pose");
    }

    public Command setPoseAfterL4() {
        return run(coroutine -> {
            mElevator.setHeightSetpoint(ElevatorIO.POSE_AFTER_L4);
            mArm.setAngleSetpoint(ArmIO.POSE_AFTER_L4);
        }).named("Set Superstructure to After L4 Pose");
    }

    public Command setPoseScoreL3() {
        return run(coroutine -> {
            mElevator.setHeightSetpoint(ElevatorIO.POSE_SCORE_L3);
            mArm.setAngleSetpoint(ArmIO.POSE_SCORE_L3);
        }).named("Set Superstructure to Score L3 Pose");
    }

    public Command setPoseAfterL3() {
        return run(coroutine -> {
            mElevator.setHeightSetpoint(ElevatorIO.POSE_AFTER_L3);
            mArm.setAngleSetpoint(ArmIO.POSE_AFTER_L3);
        }).named("Set Superstructure to After L3 Pose");
    }   

    public Command setPoseScoreL2() {
        return run(coroutine -> {
            mElevator.setHeightSetpoint(ElevatorIO.POSE_SCORE_L2);
            mArm.setAngleSetpoint(ArmIO.POSE_SCORE_L2);
        }).named("Set Superstructure to Score L2 Pose");
    }

    public Command setPoseAfterL2() {
        return run(coroutine -> {
            mElevator.setHeightSetpoint(ElevatorIO.POSE_AFTER_L2);
            mArm.setAngleSetpoint(ArmIO.POSE_AFTER_L2);
        }).named("Set Superstructure to After L2 Pose");
    }

    public Command setPoseIntake() {
        return run(coroutine -> {
            mElevator.setHeightSetpoint(ElevatorIO.POSE_HANDOFF);
            mArm.setAngleSetpoint(ArmIO.POSE_HANDOFF);
            mIntake.setAngleSetpoint(IntakeIO.POSE_INTAKE);
        }).named("Set Superstructure to Intake Pose");
    }

    public Command simSetIntakeHasCoral(boolean hasCoral) {
        return Command.noRequirements(coroutine -> {
            ((IntakeIOSim) mIntake).setHasCoral(hasCoral);
        }).named("Sim Set Intake Has Coral");
    }
    
    
}
