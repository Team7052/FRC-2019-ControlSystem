package frc.robot.commands.arm;

import frc.robot.states.ArmSuperState;
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

        ArmSuperState.getInstance().addShoulderDelegate(shoulderController);
        ArmSuperState.getInstance().addElbowDelegate(elbowController);
        ArmSuperState.getInstance().addWristDelegate(wristController);
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