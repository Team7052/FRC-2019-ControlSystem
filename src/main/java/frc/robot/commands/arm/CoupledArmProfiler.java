package frc.robot.commands.arm;

import frc.robot.helpers.Pair;
import frc.robot.motionProfiling.MotionProfiler;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.Motor;
import frc.robot.motionProfiling.Point;

import java.util.ArrayList;

public class CoupledArmProfiler {
    ArmSubsystem arm;
    public interface ShoulderProfileDelegate {
        void updateNewProfile(MotionProfiler profile);
    }
    public interface ElbowProfileDelegate {
        void updateNewProfile(MotionProfiler profile);
        void setTargetElbowAbsolutePosition(double target);
    }


    ShoulderProfileDelegate shoulderProfileDelegate;
    ElbowProfileDelegate elbowProfileDelegate;


    private MotionProfiler shoulderMotionProfiler;
    private MotionProfiler elbowMotionProfiler;
    private MotionProfiler wristMotionProfiler;

    double elbowAbsoluteAngle;

    public double shoulderMaxVelocity = Math.PI;
    public double shoulderMaxAcceleration = Math.PI;

    public double elbowMaxVelocity = Math.PI * 2;
    public double elbowMaxAcceleration = Math.PI * 2;

    public CoupledArmProfiler(ShoulderProfileDelegate shoulderProfileDelegate, ElbowProfileDelegate elbowProfileDelegate) {
        this.shoulderMotionProfiler = new MotionProfiler();
        this.elbowMotionProfiler = new MotionProfiler();
        this.wristMotionProfiler = new MotionProfiler();

        this.shoulderProfileDelegate = shoulderProfileDelegate;
        this.elbowProfileDelegate = elbowProfileDelegate;

        arm = ArmSubsystem.getInstance();
        elbowAbsoluteAngle = arm.getHomeDegrees(Motor.ELBOW_JOINT) / 180 * Math.PI;
    }

    public void generateProfiles(double shoulderAngle, double elbowAngle) {
        // trapezoidal motion profiling.

        double initRadiansShoulder = arm.getDegrees(Motor.SHOULDER_JOINT) / 180 * Math.PI;
        double endRadiansShoulder = shoulderAngle;
        double initRadiansElbow = arm.getAbsoluteElbowDegrees() / 180 * Math.PI;
        double endRadiansElbow =  elbowAngle;

        Pair<ArrayList<Point>> trapezoidalProfiles = this.trapezoidalVelocities(initRadiansShoulder, endRadiansShoulder, initRadiansElbow, endRadiansElbow);
        shoulderMotionProfiler.setVelocityPoints(MotionProfiler.getLinearInterpolation(trapezoidalProfiles.a, 0.01), initRadiansShoulder);
        elbowMotionProfiler.setVelocityPoints(MotionProfiler.getLinearInterpolation(trapezoidalProfiles.b, 0.01), initRadiansElbow);

        this.elbowAbsoluteAngle = elbowMotionProfiler.getFinalPosition();

        ArrayList<Point> newElbowPositions = getRelativePositionProfile(elbowMotionProfiler.getPositionFunction(), shoulderMotionProfiler.getPositionFunction());
        elbowMotionProfiler.setPositionPoints(newElbowPositions);

        System.out.println(elbowMotionProfiler.getTotalTime());

        // generate wrist profile
        double maxTime = newElbowPositions.get(newElbowPositions.size() - 1).x;


        ArrayList<Point> wristPoints = new ArrayList<>();
        wristPoints.add(new Point(0, 90));
        wristPoints.add(new Point(maxTime, 90));
        // get total time of elbow positions
        this.shoulderProfileDelegate.updateNewProfile(shoulderMotionProfiler);
        this.elbowProfileDelegate.updateNewProfile(elbowMotionProfiler);
        this.elbowProfileDelegate.setTargetElbowAbsolutePosition(this.elbowAbsoluteAngle);
    }

    public void generateSequentialProfiles(ArrayList<Pair<Double>> anglePairs) {
        if (anglePairs.size() == 0) return;
        double initRadiansShoulder = arm.getDegrees(Motor.SHOULDER_JOINT) / 180 * Math.PI;
        double endRadiansShoulder = anglePairs.get(0).a;
        double initRadiansElbow = arm.getAbsoluteElbowDegrees() / 180 * Math.PI;
        double endRadiansElbow =  anglePairs.get(0).b;

        ArrayList<Point> shoulderVelocityPoints = new ArrayList<>();
        ArrayList<Point> elbowVelocityPoints = new ArrayList<>();

        Pair<ArrayList<Point>> firstSet = this.trapezoidalVelocities(initRadiansShoulder, endRadiansShoulder, initRadiansElbow, endRadiansElbow);
        shoulderVelocityPoints.addAll(firstSet.a);
        elbowVelocityPoints.addAll(firstSet.b);
        for (int i = 1; i < anglePairs.size(); i++) {
            initRadiansShoulder = endRadiansShoulder;
            initRadiansElbow = endRadiansElbow;
            endRadiansShoulder = anglePairs.get(i).a;
            endRadiansElbow = anglePairs.get(i).b;
            Pair<ArrayList<Point>> nextSet = this.trapezoidalVelocities(initRadiansShoulder, endRadiansShoulder, initRadiansElbow, endRadiansElbow);
            shoulderVelocityPoints.addAll(nextSet.a);
            elbowVelocityPoints.addAll(nextSet.b);
        }


        shoulderMotionProfiler.setVelocityPoints(MotionProfiler.getLinearInterpolation(shoulderVelocityPoints, 0.01), arm.getDegrees(Motor.SHOULDER_JOINT) / 180 * Math.PI);
        elbowMotionProfiler.setVelocityPoints(MotionProfiler.getLinearInterpolation(elbowVelocityPoints, 0.01), arm.getAbsoluteElbowDegrees() / 180 * Math.PI);

        this.elbowAbsoluteAngle = elbowMotionProfiler.getFinalPosition();

        ArrayList<Point> newElbowPositions = getRelativePositionProfile(elbowMotionProfiler.getPositionFunction(), shoulderMotionProfiler.getPositionFunction());
        elbowMotionProfiler.setPositionPoints(newElbowPositions);

        System.out.println(elbowMotionProfiler.getTotalTime());

        this.shoulderProfileDelegate.updateNewProfile(shoulderMotionProfiler);
        this.elbowProfileDelegate.updateNewProfile(elbowMotionProfiler);
        this.elbowProfileDelegate.setTargetElbowAbsolutePosition(this.elbowAbsoluteAngle);
    }

    private Pair<ArrayList<Point>> trapezoidalVelocities(double initShoulder, double endShoulder, double initElbow, double endElbow) {
        ArrayList<Point> shoulderTrapezoidShape = MotionProfiler.generateTrapezoidalProfile(initShoulder, endShoulder, shoulderMaxVelocity, shoulderMaxAcceleration);
        ArrayList<Point> elbowTrapezoidShape = MotionProfiler.generateTrapezoidalProfile(initElbow, endElbow, elbowMaxVelocity, elbowMaxAcceleration);

        double shoulderProfileTime = MotionProfiler.totalTimeOfProfile(shoulderTrapezoidShape);
        double elbowProfileTime = MotionProfiler.totalTimeOfProfile(elbowTrapezoidShape);

        if (shoulderProfileTime > elbowProfileTime) {
            elbowTrapezoidShape = MotionProfiler.transformTrapezoidByTime(elbowTrapezoidShape, shoulderProfileTime);
        }
        else if (elbowProfileTime > shoulderProfileTime) {
            shoulderTrapezoidShape = MotionProfiler.transformTrapezoidByTime(shoulderTrapezoidShape, elbowProfileTime);
        }
        return new Pair(shoulderTrapezoidShape, elbowTrapezoidShape);
    }

    public ArrayList<Point> getRelativePositionProfile(ArrayList<Point> absoluteElbowPositionPoints, ArrayList<Point> absoluteShoulderPositionPoints) {
        ArrayList<Point> relativeElbowPositions = new ArrayList<>();
        // reset elbow points
        if (absoluteElbowPositionPoints.size() >= absoluteShoulderPositionPoints.size()) {
            for (int i = 0; i < absoluteElbowPositionPoints.size(); i++) {
                // get shoulder profile position for time: x
                Point point = absoluteElbowPositionPoints.get(i);

                int shoulderIndex = i < absoluteShoulderPositionPoints.size() ? i : absoluteShoulderPositionPoints.size() - 1;
                relativeElbowPositions.add(new Point(point.x, this.radiansRelativeToShoulder(point.y, absoluteShoulderPositionPoints.get(shoulderIndex).y)));
            }
        }
        else {
            for (int i = 0; i < absoluteShoulderPositionPoints.size(); i++) {
                // get shoulder profile position for time: x
                Point point = i < absoluteElbowPositionPoints.size() ? absoluteElbowPositionPoints.get(i) : absoluteElbowPositionPoints.get(absoluteElbowPositionPoints.size() - 1);
                relativeElbowPositions.add(new Point(point.x, this.radiansRelativeToShoulder(point.y, absoluteShoulderPositionPoints.get(i).y)));
            }
        }


        return relativeElbowPositions;
    }

    private double radiansRelativeToShoulder(double absoluteRadians, double shoulderRadians) {
        //System.out.println("Radians: " + absoluteRadians + " " + shoulderRadians);
        return absoluteRadians - shoulderRadians + arm.getHomeDegrees(Motor.SHOULDER_JOINT) / 360.0 * (Math.PI * 2);
    }

    private double degrees(Motor type) {
        return arm.getRotationMotor(type).getCurrentDegrees();
    }
}