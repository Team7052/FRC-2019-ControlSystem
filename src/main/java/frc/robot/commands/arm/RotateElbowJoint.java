package frc.robot.commands.arm;

import frc.robot.motionProfiling.MotionProfileState;
import frc.robot.motionProfiling.MotionProfiler;
import frc.robot.motionProfiling.MotionTriplet;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.Motor;

public class RotateElbowJoint implements CoupledArmProfiler.ElbowProfileDelegate {
    public CommandDelegate delegate;
    
    MotionProfiler motionProfiler;
    ArmSubsystem arm;

    private double targetHoldAngle;


    public RotateElbowJoint() {
        arm = ArmSubsystem.getInstance();
        targetHoldAngle = arm.getHomeDegrees(Motor.ELBOW_JOINT) / 180 * Math.PI;
    }

    public boolean isRunning() {
        if (this.motionProfiler == null) return false;
        return this.motionProfiler.getState() == MotionProfileState.RUNNING;
    }

    public boolean isFinished() {
        if (this.motionProfiler == null) return false;
        return this.motionProfiler.getState() == MotionProfileState.FINISHED;
    }

    public void execute() {
        if (this.isRunning()) {
            MotionTriplet profileTriplet = motionProfiler.updateMotionProfile();
            if (profileTriplet != null) {
                // set position relative to shoulder joint
                double degrees = profileTriplet.position / (2 * Math.PI) * 360;
                arm.setDegrees(Motor.ELBOW_JOINT, degrees);
            }
        }
        else if (this.isFinished()) {
            double position = this.targetHoldAngle / Math.PI * 180;
            arm.setDegrees(Motor.ELBOW_JOINT, degreesRelativeToShoulder(position));
        }
    }

    private double degreesRelativeToShoulder(double absoluteDegrees) {
        return absoluteDegrees - arm.getDegrees(Motor.SHOULDER_JOINT) + arm.getHomeDegrees(Motor.SHOULDER_JOINT);
    }

    public double getTargetElbowAbsolutePosition() {
        return this.targetHoldAngle;
    }
    @Override
    public void setTargetElbowAbsolutePosition(double target) {
        this.targetHoldAngle = target;
    }

    @Override
    public void updateNewProfile(MotionProfiler profile) {
        this.motionProfiler = profile;
        this.motionProfiler.reset();
        this.arm.getRotationMotor(Motor.ELBOW_JOINT).setIntegralAccumulator(0);
        this.motionProfiler.startMotionProfile();
        if (delegate != null) delegate.beganMotionProfile("RotateElbowJoint", this.motionProfiler);
    }
}
