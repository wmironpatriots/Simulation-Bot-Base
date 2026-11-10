package org.frc6423.robot.subsystems.superstructure;

import org.frc6423.robot.subsystems.superstructure.arm.ArmIO;
import org.frc6423.robot.subsystems.superstructure.arm.ArmIOSim;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.epilogue.Logged;

public class Superstructure implements Mechanism {
    
    @Logged(name = "Arm")
    private final ArmIO mArm;

    private Superstructure(ArmIO mArm) {
        this.mArm = mArm;
    }

    public static Superstructure create() {
        ArmIO arm = new ArmIOSim();
        return new Superstructure(arm);
    }

    public void periodic() {
        mArm.periodic();
    }

    public Command setPoseReady() {
        return run(coroutine -> {
            mArm.setAngleSetpoint(ArmIO.POSE_READY);
        }).named("Set Superstructure to Ready Pose");
    }
}
