package frc.robot.commands;


import java.util.ArrayList;

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

public class RotateShoulderJoint extends Command {

    public int targetDegrees = 90;
    MotionProfiler shoulderMotionProfiler;
    MotionProfiler elbowMotionProfiler;

    ArmSubsystem arm;
    public RotateShoulderJoint() {
        arm = ArmSubsystem.getInstance();
        requires(arm);
        
        shoulderMotionProfiler = new MotionProfiler();
        elbowMotionProfiler = new MotionProfiler();

        ArrayList<Point> points = new ArrayList<>();
        double t1 = 0.4;
        double t2 = 2.0;
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
        double realElbowTarget = 0;
        MotionTriplet shoulderTriplet = shoulderMotionProfiler.updateMotionProfile(2.0);
        if (shoulderMotionProfiler.getState() == MotionProfileState.RUNNING && shoulderTriplet != null) {
            int targetPosition = (int) (arm.shoulderHomePosition + shoulderTriplet.position / (2 * Math.PI) * 4096);
            //arm.setPosition(Motor.SHOULDER_JOINT, targetPosition);
            realElbowTarget = targetPosition;
        }
        else if (shoulderMotionProfiler.getState() == MotionProfileState.FINISHED) {
            if (shoulderMotionProfiler.getPositionFunction().size() > 0) {
                double position = shoulderMotionProfiler.getPositionFunction().get(shoulderMotionProfiler.getPositionFunction().size() - 1).y;
                //arm.setPosition(Motor.SHOULDER_JOINT, (int) (arm.shoulderHomePosition + position / (2 * Math.PI) * 4096));
                realElbowTarget =  (int) (arm.shoulderHomePosition + position / (2 * Math.PI) * 4096);
            }
        }

        System.out.println("target: " + realElbowTarget + ", current: " + arm.getPosition(Motor.SHOULDER_JOINT));

        elbowMotionProfiler.startMotionProfile();
        MotionTriplet elbowTriplet = elbowMotionProfiler.updateMotionProfile(1.4);
        if (elbowMotionProfiler.getState() == MotionProfileState.RUNNING && elbowTriplet != null) {
            int targetPosition = (int) (arm.elbowHomePosition + elbowTriplet.position / (2 * Math.PI) * 4096);
            arm.setPosition(Motor.ELBOW_JOINT, targetPosition);
            realElbowTarget = targetPosition;
        }
        else if (elbowMotionProfiler.getState() == MotionProfileState.FINISHED) {
            if (elbowMotionProfiler.getPositionFunction().size() > 0) {
                double position = elbowMotionProfiler.getPositionFunction().get(elbowMotionProfiler.getPositionFunction().size() - 1).y;
                arm.setPosition(Motor.ELBOW_JOINT, (int) (arm.elbowHomePosition + position / (2 * Math.PI) * 4096));
                realElbowTarget =  (int) (arm.elbowHomePosition + position / (2 * Math.PI) * 4096);
            }
        }

        
        
        /*if (Robot.oi.button_A()) {
            if (!motionProfiler.running && !prevPressed) {
                motionProfiler.startMotionProfile();
                this.initialPosition = arm.getPosition(Motor.SHOULDER_JOINT);
            }
            
            if (triplet != null) {
                System.out.println(position % 4096 + " " +  target % 4096 + " ");
                arm.setPosition(Motor.SHOULDER_JOINT, target);
            }
            else {
                arm.setCurrent(Motor.SHOULDER_JOINT, arm.getPosition(Motor.SHOULDER_JOINT));
            }

            prevPressed = true;
        }
        else {
            prevPressed = false;
            motionProfiler.stopMotionProfile();
            arm.stop(Motor.SHOULDER_JOINT);
        }*/
    }
}