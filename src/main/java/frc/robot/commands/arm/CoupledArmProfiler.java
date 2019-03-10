package frc.robot.commands.arm;

import frc.robot.helpers.Pair;
import frc.robot.motionProfiling.MotionProfiler;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.Motor;
import frc.robot.motionProfiling.Point;

import java.util.ArrayList;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

public class CoupledArmProfiler implements Runnable {
    ArmSubsystem arm;
    public interface ShoulderProfileDelegate {
        void updateNewProfile(MotionProfiler profile);
    }
    public interface ElbowProfileDelegate {
        void updateNewProfile(MotionProfiler profile);
        void setTargetElbowAbsolutePosition(double target);
    }
    public interface WristProfileDelegate {
        void updateNewProfile(MotionProfiler profile);
    }

    ShoulderProfileDelegate shoulderProfileDelegate;
    ElbowProfileDelegate elbowProfileDelegate;
    WristProfileDelegate wristProfileDelegate;

    private MotionProfiler shoulderMotionProfiler;
    private MotionProfiler elbowMotionProfiler;
    private MotionProfiler wristMotionProfiler;

    double elbowAbsoluteAngle;

    public double shoulderMaxVelocity = Math.PI * 2 / 5;
    public double shoulderMaxAcceleration = Math.PI * 2 / 3;

    public double elbowMaxVelocity = Math.PI;
    public double elbowMaxAcceleration = Math.PI;

    Thread t;
    ScheduledExecutorService timer;

    public boolean shouldGenerateMotionProfiles = false;
    double shoulderTarget;
    double elbowTarget;
    double wristTarget;

    public boolean shouldGenerateSequentialProfiles = false;
    ArrayList<Pair<Double>> anglePairs = new ArrayList<>();

    @Override
    public void run() {
        if (shouldGenerateMotionProfiles) {
            shouldGenerateMotionProfiles = false;
            this.generateProfiles(this.shoulderTarget, this.elbowTarget, this.wristTarget);
        }
        else if (shouldGenerateSequentialProfiles) {
            System.out.println("Generate");
            shouldGenerateSequentialProfiles = false;
            this.generateSequentialProfilesWhenReady(this.anglePairs);
        }
    }

    public CoupledArmProfiler(ShoulderProfileDelegate shoulderProfileDelegate, ElbowProfileDelegate elbowProfileDelegate, WristProfileDelegate wristProfileDelegate) {
        this.shoulderMotionProfiler = new MotionProfiler();
        this.elbowMotionProfiler = new MotionProfiler();
        this.wristMotionProfiler = new MotionProfiler();

        this.shoulderProfileDelegate = shoulderProfileDelegate;
        this.elbowProfileDelegate = elbowProfileDelegate;
        this.wristProfileDelegate = wristProfileDelegate;

        arm = ArmSubsystem.getInstance();
        elbowAbsoluteAngle = arm.getHomeDegrees(Motor.ELBOW_JOINT) / 180 * Math.PI;

        t = new Thread(this, "motion profiler thread");
        timer = Executors.newSingleThreadScheduledExecutor();
        timer.scheduleAtFixedRate(this, 0, 20, TimeUnit.MILLISECONDS);
        t.start();
    }

    public void generateProfilesWhenReady(double shoulderAngle, double elbowAngle) {
        this.generateProfilesWhenReady(shoulderAngle, elbowAngle, Math.PI);
    }
    public void generateProfilesWhenReady(double shoulderAngle, double elbowAngle, double wristAngle) {
        this.shouldGenerateMotionProfiles = true;
        this.shoulderTarget = shoulderAngle;
        this.elbowTarget = elbowAngle;
        this.wristTarget = wristAngle;
    }

    public void generateSequentialProfilesWhenReady(ArrayList<Pair<Double>> anglePairs) {
        this.anglePairs = anglePairs;
        this.shouldGenerateSequentialProfiles = true;
    }

    private void generateProfiles(double shoulderAngle, double elbowAngle) {
        this.generateProfiles(shoulderAngle, elbowAngle, Math.PI);
    }

    private void generateProfiles(double shoulderAngle, double elbowAngle, double wristAngle) {
        // trapezoidal motion profiling.
        double initRadiansShoulder = arm.getDegrees(Motor.SHOULDER_JOINT) / 180 * Math.PI;
        double endRadiansShoulder = shoulderAngle;
        double initRadiansElbow = arm.getAbsoluteElbowDegrees() / 180 * Math.PI;
        double endRadiansElbow =  elbowAngle;

        Pair<ArrayList<Point>> trapezoidalProfiles = this.trapezoidalVelocities(initRadiansShoulder, endRadiansShoulder, initRadiansElbow, endRadiansElbow);
        shoulderMotionProfiler.setVelocityPoints(MotionProfiler.getLinearInterpolation(trapezoidalProfiles.a, 0.01), initRadiansShoulder);
        elbowMotionProfiler.setVelocityPoints(MotionProfiler.getLinearInterpolation(trapezoidalProfiles.b, 0.01), initRadiansElbow);

        this.elbowAbsoluteAngle = elbowMotionProfiler.getFinalPosition();

        // generate wrist profile
        ArrayList<Point> wristPoints = new ArrayList<>();

        // interpolate wrist points
        double initRadiansWrist = arm.getAbsoluteWristDegrees() / 180.0 * Math.PI;
        double totalTime = elbowMotionProfiler.getTotalTime();
        for (Point point: elbowMotionProfiler.getPositionFunction()) {
            double percentage = point.x / totalTime;
            wristPoints.add(new Point(point.x, initRadiansWrist + (wristAngle - initRadiansWrist) * percentage));
        }
        // take elbow absolute angles and transform the wrist to match those angles
        ArrayList<Point> relativeWristPoints = this.getRelativeWristPositionProfile(wristPoints, elbowMotionProfiler.getPositionFunction());
        wristMotionProfiler.setPositionPoints(relativeWristPoints);

        ArrayList<Point> newElbowPositions = getRelativeElbowPositionProfile(elbowMotionProfiler.getPositionFunction(), shoulderMotionProfiler.getPositionFunction());
        elbowMotionProfiler.setPositionPoints(newElbowPositions);

        System.out.println(elbowMotionProfiler.getTotalTime());
        
        // get total time of elbow positions
        this.shoulderProfileDelegate.updateNewProfile(shoulderMotionProfiler);
        this.elbowProfileDelegate.updateNewProfile(elbowMotionProfiler);
        this.elbowProfileDelegate.setTargetElbowAbsolutePosition(this.elbowAbsoluteAngle);
        this.wristProfileDelegate.updateNewProfile(wristMotionProfiler);
    }

    private void generateSequentialProfiles(ArrayList<Pair<Double>> anglePairs) {
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

         // generate wrist profile
         ArrayList<Point> wristPoints = new ArrayList<>();
         for (Point point: elbowMotionProfiler.getPositionFunction()) {
             wristPoints.add(new Point(point.x, Math.PI));
         }
         // take elbow absolute angles and transform the wrist to match those angles
         ArrayList<Point> relativeWristPoints = this.getRelativeWristPositionProfile(wristPoints, elbowMotionProfiler.getPositionFunction());
         wristMotionProfiler.setPositionPoints(relativeWristPoints);

        ArrayList<Point> newElbowPositions = getRelativeElbowPositionProfile(elbowMotionProfiler.getPositionFunction(), shoulderMotionProfiler.getPositionFunction());
        elbowMotionProfiler.setPositionPoints(newElbowPositions);

        System.out.println(elbowMotionProfiler.getTotalTime());

        this.shoulderProfileDelegate.updateNewProfile(shoulderMotionProfiler);
        this.elbowProfileDelegate.updateNewProfile(elbowMotionProfiler);
        this.elbowProfileDelegate.setTargetElbowAbsolutePosition(this.elbowAbsoluteAngle);
        this.wristProfileDelegate.updateNewProfile(wristMotionProfiler);
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
        return new Pair<ArrayList<Point>>(shoulderTrapezoidShape, elbowTrapezoidShape);
    }

    public ArrayList<Point> getRelativeWristPositionProfile(ArrayList<Point> absoluteWristPoints, ArrayList<Point> absoluteElbowPoints) {
        ArrayList<Point> relativeWristPoints = new ArrayList<>();
        boolean shouldProfile = true;
        Point lastElbowPoint = absoluteElbowPoints.get(absoluteElbowPoints.size() - 1);
        if (lastElbowPoint.x < 0.5) {
            shouldProfile = false;
        }
        for (int i = 0; i < absoluteWristPoints.size(); i++) {
            Point point = absoluteWristPoints.get(i);
            relativeWristPoints.add(new Point(point.x, radiansWristRelativeToElbow(point.y, shouldProfile ? absoluteElbowPoints.get(i).y : lastElbowPoint.y)));
        }
        return relativeWristPoints;
    }

    public ArrayList<Point> getRelativeElbowPositionProfile(ArrayList<Point> absoluteElbowPositionPoints, ArrayList<Point> absoluteShoulderPositionPoints) {
        ArrayList<Point> relativeElbowPositions = new ArrayList<>();
        // reset elbow points
        if (absoluteElbowPositionPoints.size() >= absoluteShoulderPositionPoints.size()) {
            for (int i = 0; i < absoluteElbowPositionPoints.size(); i++) {
                // get shoulder profile position for time: x
                Point point = absoluteElbowPositionPoints.get(i);

                int shoulderIndex = i < absoluteShoulderPositionPoints.size() ? i : absoluteShoulderPositionPoints.size() - 1;
                relativeElbowPositions.add(new Point(point.x, this.radiansElbowRelativeToShoulder(point.y, absoluteShoulderPositionPoints.get(shoulderIndex).y)));
            }
        }
        else {
            for (int i = 0; i < absoluteShoulderPositionPoints.size(); i++) {
                // get shoulder profile position for time: x
                Point point = i < absoluteElbowPositionPoints.size() ? absoluteElbowPositionPoints.get(i) : absoluteElbowPositionPoints.get(absoluteElbowPositionPoints.size() - 1);
                relativeElbowPositions.add(new Point(point.x, this.radiansElbowRelativeToShoulder(point.y, absoluteShoulderPositionPoints.get(i).y)));
            }
        }


        return relativeElbowPositions;
    }

    private double radiansElbowRelativeToShoulder(double absoluteRadians, double shoulderRadians) {
        //System.out.println("Radians: " + absoluteRadians + " " + shoulderRadians);
        return absoluteRadians - shoulderRadians + arm.getHomeDegrees(Motor.SHOULDER_JOINT) / 180.0 * Math.PI;
    }

    private double radiansWristRelativeToElbow(double absoluteRadians, double elbowRadiansAbsolute) {
        //System.out.println("Radians: " + absoluteRadians + " " + shoulderRadians);
        return absoluteRadians - elbowRadiansAbsolute + arm.getHomeDegrees(Motor.ELBOW_JOINT) / 180.0 * Math.PI;
    }

    private double degrees(Motor type) {
        return arm.getRotationMotor(type).getCurrentDegrees();
    }
}