package frc.robot.commands.arm;

import frc.robot.motionProfiling.MotionProfileState;
import frc.robot.motionProfiling.MotionProfiler;
import frc.robot.motionProfiling.MotionTriplet;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.Motor;

public class RotateShoulderJoint implements CoupledArmProfiler.ShoulderProfileDelegate{

    MotionProfiler motionProfiler;
    ArmSubsystem arm;


    public RotateShoulderJoint() {
        arm = ArmSubsystem.getInstance();
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
                arm.setDegrees(Motor.SHOULDER_JOINT, profileTriplet.position / Math.PI * 180);
            }
        }
        else if (this.isFinished()) {
            double position = motionProfiler.getFinalPosition();
            arm.setDegrees(Motor.SHOULDER_JOINT, position / Math.PI * 180);
        }
    }

    @Override
    public void updateNewProfile(MotionProfiler profile) {
        this.motionProfiler = profile;
        this.motionProfiler.reset();
        this.motionProfiler.startMotionProfile();
    }
}
