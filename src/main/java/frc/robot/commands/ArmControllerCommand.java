package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.motionProfiling.MotionProfileState;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.Motor;

public class ArmControllerCommand extends Command {
    RotateShoulderJoint shoulderCommand;
    RotateElbowJoint elbowCommand;
    RotateWristJoint wristCommand;
    ArmSubsystem arm;
    public ArmControllerCommand() {
        arm = ArmSubsystem.getInstance();
        shoulderCommand = new RotateShoulderJoint();
        elbowCommand = new RotateElbowJoint();
        wristCommand = new RotateWristJoint();
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

    public boolean motionProfilesRunning() {
        return shoulderCommand.shoulderMotionProfiler.getState() == MotionProfileState.RUNNING || elbowCommand.elbowMotionProfiler.getState() == MotionProfileState.RUNNING;
    }

    String currentProfile = "none";

    @Override
    protected void execute() {
        super.execute();
        shoulderCommand.execute();
        elbowCommand.execute();
        wristCommand.execute();
        if (Robot.oi.button_A() && !currentProfile.equals("bA")) {
            currentProfile = "bA";
            this.setAngles(40, 180);
        }
        else if (Robot.oi.button_X() && !currentProfile.equals("bX")) {
            this.setAngles(50, 110);
            currentProfile = "bX";
        }
        else if (Robot.oi.button_Y() && !currentProfile.equals("bY")) {
            this.setAngles(225, 200);
            currentProfile = "bY";
        }
        else if (Robot.oi.button_B() && !currentProfile.equals("bB")) {
            this.setAngles(300, 180);
        }
        if (Robot.oi.dPad_UP()) {
            currentProfile = "bD";
            if (!this.motionProfilesRunning()) {
                this.setDeltaAngles(3, 3);
            }
        }
        else if (Robot.oi.dPad_DOWN()) {
            currentProfile = "bD";
            if (!this.motionProfilesRunning()) {
                this.setDeltaAngles(-3, -3);
            }
        }
        System.out.println(this.motionProfilesRunning());

    }

    public void setDeltaAngles(double shoulderDelta, double elbowDelta) {
        double shoulderDegrees = arm.getDegrees(Motor.SHOULDER_JOINT);
        double elbowDegrees = arm.getDegrees(Motor.ELBOW_JOINT);

        shoulderCommand.restartMotionProfile(shoulderDegrees + shoulderDelta, Math.PI / 2, Math.PI / 2);
        elbowCommand.restartMotionProfile(elbowCommand.targetHoldAbsoluteAngle + elbowDelta, shoulderCommand.shoulderMotionProfiler, Math.PI / 2, Math.PI / 2);

    }

    public void setAngles(double shoulderAngle, double elbowAngle) {
        double shoulderMaxVelocity = Math.PI;
        double shoulderMaxAcceleration = Math.PI;
        if (shoulderAngle > 180 && arm.getDegrees(Motor.SHOULDER_JOINT) < 180) {
            shoulderMaxVelocity = Math.PI / 3.0;
            shoulderMaxAcceleration = Math.PI / 4.0;
        }
        else if (shoulderAngle < 180 && arm.getDegrees(Motor.SHOULDER_JOINT) > 180) {
            shoulderMaxVelocity = Math.PI / 3.0;
            shoulderMaxAcceleration = Math.PI / 4.0;
        }
        shoulderCommand.restartMotionProfile(shoulderAngle, shoulderMaxVelocity, shoulderMaxAcceleration);
        elbowCommand.restartMotionProfile(elbowAngle, shoulderCommand.shoulderMotionProfiler, Math.PI / 2.0, Math.PI * 3 / 2);
    }
}