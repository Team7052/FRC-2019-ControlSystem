package frc.robot.commands.arm;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.Motor;

public class ArmControllerCommand{
    ArmSubsystem arm;
    public JointController shoulderController;
    public JointController elbowController;
    public JointController wristController;

    public ArmControllerCommand() {
        arm = ArmSubsystem.getInstance();
        shoulderController = new JointController(Motor.SHOULDER_JOINT);
        elbowController = new JointController(Motor.ELBOW_JOINT);
        wristController = new JointController(Motor.WRIST_JOINT);

        arm.getSuperState().addShoulderDelegate(shoulderController);
        arm.getSuperState().addElbowDelegate(elbowController);
        arm.getSuperState().addWristDelegate(wristController);
    }

    protected boolean isFinished() {
        return false;
    }

    public void execute() {
        this.shoulderController.execute();
        this.elbowController.execute();
        this.wristController.execute();
    }
}