package frc.robot.commands;

import frc.robot.helpers.RotationMotor;
import frc.robot.motionProfiling.MotionProfileState;
import frc.robot.motionProfiling.MotionProfiler;
import frc.robot.motionProfiling.MotionTriplet;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.Motor;

public class RotateShoulderJoint {

    MotionProfiler shoulderMotionProfiler;
    ArmSubsystem arm;

    private boolean isRunning = false;

    public RotateShoulderJoint() {
        arm = ArmSubsystem.getInstance();

        shoulderMotionProfiler = new MotionProfiler();
    }

    public boolean isFinished() {
        return this.isRunning == false;
    }

    public void restartMotionProfile(double angle, double maxVelocity, double maxAcceleration) {
        RotationMotor shoulderMotor = arm.getRotationMotor(Motor.SHOULDER_JOINT);
        double initRadians = shoulderMotor.getCurrentDegrees() / 180 * Math.PI;
        double endRadians =  angle / 180 * Math.PI;
        shoulderMotionProfiler.setVelocityPoints(MotionProfiler.generateTrapezoidalProfile(initRadians, endRadians, maxVelocity, maxAcceleration), initRadians);
        shoulderMotionProfiler.restartMotionProfile();
        this.isRunning = true;
    }

    public void execute() {
        if (this.isRunning) {
            shoulderMotionProfiler.startMotionProfile();
            MotionTriplet shoulderTriplet = shoulderMotionProfiler.updateMotionProfile();
            if (shoulderMotionProfiler.getState() == MotionProfileState.RUNNING && shoulderTriplet != null) {
                arm.setDegrees(Motor.SHOULDER_JOINT, shoulderTriplet.position / (2*Math.PI) * 360);
            }
            else if (shoulderMotionProfiler.getState() == MotionProfileState.FINISHED) {
                if (shoulderMotionProfiler.getPositionFunction().size() > 0) {
                    double position = shoulderMotionProfiler.getPositionFunction().get(shoulderMotionProfiler.getPositionFunction().size() - 1).y;
                    arm.setDegrees(Motor.SHOULDER_JOINT, position / (2*Math.PI) * 360);
                }
            }
        }
    }
}
