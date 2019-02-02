package frc.robot.commands;


import java.util.ArrayList;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.motionProfiling.FunctionGenerator;
import frc.robot.motionProfiling.FunctionSet;
import frc.robot.motionProfiling.MotionProfileState;
import frc.robot.motionProfiling.MotionProfiler;
import frc.robot.motionProfiling.MotionTriplet;
import frc.robot.motionProfiling.Point;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.Motor;

public class RotateShoulderJoint extends Command implements PIDSource, PIDOutput {

    public int targetDegrees = 90;
    MotionProfiler shoulderMotionProfiler;
    MotionProfiler elbowMotionProfiler;

    PIDController pidController;

    ArmSubsystem arm;
    public RotateShoulderJoint() {
        arm = ArmSubsystem.getInstance();
        requires(arm);
        
        shoulderMotionProfiler = new MotionProfiler();
        elbowMotionProfiler = new MotionProfiler();

        ArrayList<Point> points = new ArrayList<>();
        double t1 = 0.4;
        double t2 = 1.0;
        double maxVelocity = Math.PI / 3;
        
        points.add(new Point(0,0));
        points.add(new Point(t1, maxVelocity));
        points.add(new Point(t2, maxVelocity));
        points.add(new Point(t1 + t2, 0));

        ArrayList<Point> interpolatedShoulderPoints = shoulderMotionProfiler.getLinearInterpolation(points, 0.01);

        shoulderMotionProfiler.setVelocityPoints(interpolatedShoulderPoints);

        ArrayList<Point> elbowPoints = new ArrayList<>();
        elbowPoints.add(new Point(0,0));
        elbowPoints.add(new Point(0.4, Math.PI / 2));
        elbowPoints.add(new Point(1.0, Math.PI / 2));
        elbowPoints.add(new Point(1.4, 0));

        ArrayList<Point> interpolatedElbowPoints = elbowMotionProfiler.getLinearInterpolation(elbowPoints, 0.01);
        elbowMotionProfiler.setVelocityPoints(interpolatedElbowPoints);

        pidController = new PIDController(0.8, 0, 0, this, this);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    double motorSpeed = 0;

    boolean prevPressed = false;
    int initialPosition = 0;

    ArrayList<Point> errorFunction = new ArrayList<>();

    @Override
    protected void execute() {
        super.execute();
        shoulderMotionProfiler.startMotionProfile();
        MotionTriplet shoulderTriplet = shoulderMotionProfiler.updateMotionProfile(2.0);
        if (shoulderMotionProfiler.getState() == MotionProfileState.RUNNING && shoulderTriplet != null) {
            arm.setDegrees(Motor.SHOULDER_JOINT, shoulderTriplet.position / (2*Math.PI) * 360);
        }
        else if (shoulderMotionProfiler.getState() == MotionProfileState.FINISHED) {
            if (shoulderMotionProfiler.getPositionFunction().size() > 0) {
                double position = shoulderMotionProfiler.getPositionFunction().get(shoulderMotionProfiler.getPositionFunction().size() - 1).y;
                arm.setDegrees(Motor.SHOULDER_JOINT, position / (2*Math.PI) * 360);
            }
        }

        elbowMotionProfiler.startMotionProfile();
        MotionTriplet elbowTriplet = elbowMotionProfiler.updateMotionProfile(1.4);
        if (elbowMotionProfiler.getState() == MotionProfileState.RUNNING && elbowTriplet != null) {
            arm.setDegrees(Motor.ELBOW_JOINT, elbowTriplet.position / (2*Math.PI) * 360);
        }
        else if (elbowMotionProfiler.getState() == MotionProfileState.FINISHED) {
            if (elbowMotionProfiler.getPositionFunction().size() > 0) {
                double position = elbowMotionProfiler.getPositionFunction().get(elbowMotionProfiler.getPositionFunction().size() - 1).y;
                arm.setDegrees(Motor.ELBOW_JOINT, position / (2 * Math.PI) * 360);
            }
        }
    }

    @Override
    public void pidWrite(double output) {

    }

    @Override
    public void setPIDSourceType(PIDSourceType pidSource) {
        
    }

    @Override
    public PIDSourceType getPIDSourceType() {
        return null;
    }

    @Override
    public double pidGet() {
        float pitch = this.arm.getIMUSensor().getPitch();
        System.out.println("YAxis: " + pitch);
        return pitch;
	}
}