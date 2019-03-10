package frc.robot.commands.arm;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.ControlMode;

import frc.robot.motionProfiling.LowPassFilter;
import frc.robot.motionProfiling.MotionProfileState;
import frc.robot.motionProfiling.MotionProfiler;
import frc.robot.motionProfiling.MotionTriplet;
import frc.robot.motionProfiling.Point;
import frc.robot.subsystems.ArmSubsystem;

public class RotateWristJoint implements CoupledArmProfiler.WristProfileDelegate {
    public CommandDelegate delegate;

    LowPassFilter wristFilter;
    MotionProfiler motionProfiler;

    ArmSubsystem arm;

    public RotateWristJoint() {
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

    double motorSpeed = 0;

    boolean prevPressed = false;
    int initialPosition = 0;
    double wristMotorOutput = 0;
    double prevPitchError = 0;

    ArrayList<Point> errorFunction = new ArrayList<>();
    double time = 0;
    public void execute() {
        if (!wristEnabled) {
            arm.getWristMotor().set(ControlMode.PercentOutput, 0);
            return;
        }
        if (this.isRunning()) {
            MotionTriplet profileTriplet = motionProfiler.updateMotionProfile();
            if (profileTriplet != null) {
                arm.getWristMotor().setDegrees(profileTriplet.position / Math.PI * 180.0);
            }
        }
        else if (this.isFinished()) {
            double position = motionProfiler.getFinalPosition();
            arm.getWristMotor().setDegrees(position / Math.PI * 180.0);
        }
    }
    boolean wristEnabled = true;

    public void disableWrist() {
        wristEnabled = false;
    }
    public void enableWrist() {
        wristEnabled = true;
    }

    @Override
    public void updateNewProfile(MotionProfiler profile) {
        this.motionProfiler = profile;
        this.motionProfiler.reset();
        this.motionProfiler.startMotionProfile();
        if (delegate != null) delegate.beganMotionProfile("RotateWristJoint", this.motionProfiler);
    }
}
