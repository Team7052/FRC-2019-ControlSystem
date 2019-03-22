package frc.robot.commands.arm;

import java.util.ArrayList;

import frc.robot.helpers.Pair;
import frc.robot.helpers.Triplet;
import frc.robot.motionProfiling.MotionProfiler;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.Motor;
import frc.robot.motionProfiling.Point;

public class CoupledArmProfiler {
    public static Triplet<MotionProfiler> generateProfiles(double shoulderAngle, double elbowAngle, double wristAngle) {
        double initShoulder = ArmSubsystem.getInstance().getDegrees(Motor.SHOULDER_JOINT) / 180 * Math.PI;
        double endShoulder = shoulderAngle;
        double initElbow = ArmSubsystem.getInstance().getAbsoluteDegrees(Motor.ELBOW_JOINT) / 180 * Math.PI;
        double endElbow =  elbowAngle;
        double initWrist = ArmSubsystem.getInstance().getAbsoluteDegrees(Motor.WRIST_JOINT) / 180.0 * Math.PI;
        double endWrist = wristAngle;

        return CoupledArmProfiler.generateProfiles(initShoulder, initElbow, initWrist, endShoulder, endElbow, endWrist);
    }
    public static Triplet<MotionProfiler> generateProfiles(double initShoulder, double initElbow, double initWrist, double endShoulder, double endElbow, double endWrist) {
        // trapezoidal motion profiling.
        MotionProfiler shoulderMotionProfiler = new MotionProfiler();
        MotionProfiler elbowMotionProfiler = new MotionProfiler();
        MotionProfiler wristMotionProfiler = new MotionProfiler();
        Pair<ArrayList<Point>> trapezoidalProfiles = CoupledArmProfiler.trapezoidalVelocities(initShoulder, endShoulder, initElbow, endElbow);
        shoulderMotionProfiler.setVelocityPoints(MotionProfiler.getLinearInterpolation(trapezoidalProfiles.a, 0.01), initShoulder);
        elbowMotionProfiler.setVelocityPoints(MotionProfiler.getLinearInterpolation(trapezoidalProfiles.b, 0.01), initElbow);

        // generate wrist profile
        ArrayList<Point> wristPoints = new ArrayList<>();

        // interpolate wrist points
        double totalTime = elbowMotionProfiler.getTotalTime();
        for (Point point: elbowMotionProfiler.getPositionFunction()) {
            double percentage = point.x / totalTime;
            wristPoints.add(new Point(point.x, initWrist + (endWrist - initWrist) * percentage));
        }
        // take elbow absolute angles and transform the wrist to match those angles
        ArrayList<Point> relativeWristPoints = CoupledArmProfiler.getRelativeWristPositionProfile(wristPoints, elbowMotionProfiler.getPositionFunction());
        wristMotionProfiler.setPositionPoints(relativeWristPoints);

        ArrayList<Point> newElbowPositions = getRelativeElbowPositionProfile(elbowMotionProfiler.getPositionFunction(), shoulderMotionProfiler.getPositionFunction());
        elbowMotionProfiler.setPositionPoints(newElbowPositions);

        return new Triplet<>(shoulderMotionProfiler, elbowMotionProfiler, wristMotionProfiler);
    }

    public static void generateShoulderProfile(double shoulderAngle) {
        // trapezoidal motion profiling.
        double initRadiansShoulder = ArmSubsystem.getInstance().getDegrees(Motor.SHOULDER_JOINT) / 180 * Math.PI;
        double endRadiansShoulder = shoulderAngle;

        ArrayList<Point> points = MotionProfiler.generateTrapezoidalProfile(initRadiansShoulder, endRadiansShoulder, Math.PI * 4/5, Math.PI);
        //shoulderMotionProfiler.setVelocityPoints(MotionProfiler.getLinearInterpolation(points, 0.01), initRadiansShoulder);
    }

    private static Pair<ArrayList<Point>> trapezoidalVelocities(double initShoulder, double endShoulder, double initElbow, double endElbow) {
        ArrayList<Point> shoulderTrapezoidShape = MotionProfiler.generateTrapezoidalProfile(initShoulder, endShoulder, ArmSequences.shoulderMaxVelocity, ArmSequences.shoulderMaxAcceleration);
        ArrayList<Point> elbowTrapezoidShape = MotionProfiler.generateTrapezoidalProfile(initElbow, endElbow, ArmSequences.elbowMaxVelocity, ArmSequences.elbowMaxAcceleration);

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

    private static ArrayList<Point> getRelativeWristPositionProfile(ArrayList<Point> absoluteWristPoints, ArrayList<Point> absoluteElbowPoints) {
        ArrayList<Point> relativeWristPoints = new ArrayList<>();
        
        for (int i = 0; i < absoluteWristPoints.size(); i++) {
            Point point = absoluteWristPoints.get(i);
            relativeWristPoints.add(new Point(point.x, CoupledArmProfiler.radiansWristRelativeToElbow(point.y, absoluteElbowPoints.get(i).y)));

        }
        return relativeWristPoints;
    }

    private static ArrayList<Point> getRelativeElbowPositionProfile(ArrayList<Point> absoluteElbowPositionPoints, ArrayList<Point> absoluteShoulderPositionPoints) {
        ArrayList<Point> relativeElbowPositions = new ArrayList<>();
        // reset elbow points
        if (absoluteElbowPositionPoints.size() >= absoluteShoulderPositionPoints.size()) {
            for (int i = 0; i < absoluteElbowPositionPoints.size(); i++) {
                // get shoulder profile position for time: x
                Point point = absoluteElbowPositionPoints.get(i);

                int shoulderIndex = i < absoluteShoulderPositionPoints.size() ? i : absoluteShoulderPositionPoints.size() - 1;
                relativeElbowPositions.add(new Point(point.x, CoupledArmProfiler.radiansElbowRelativeToShoulder(point.y, absoluteShoulderPositionPoints.get(shoulderIndex).y)));
            }
        }
        else {
            for (int i = 0; i < absoluteShoulderPositionPoints.size(); i++) {
                // get shoulder profile position for time: x
                Point point = i < absoluteElbowPositionPoints.size() ? absoluteElbowPositionPoints.get(i) : absoluteElbowPositionPoints.get(absoluteElbowPositionPoints.size() - 1);
                relativeElbowPositions.add(new Point(point.x, CoupledArmProfiler.radiansElbowRelativeToShoulder(point.y, absoluteShoulderPositionPoints.get(i).y)));
            }
        }
        return relativeElbowPositions;
    }

    private static double radiansElbowRelativeToShoulder(double absoluteRadians, double shoulderRadians) {
        //System.out.println("Radians: " + absoluteRadians + " " + shoulderRadians);
        return absoluteRadians - shoulderRadians + ArmSubsystem.getInstance().getHomeDegrees(Motor.SHOULDER_JOINT) / 180.0 * Math.PI;
    }

    private static double radiansWristRelativeToElbow(double absoluteRadians, double elbowRadiansAbsolute) {
        //System.out.println("Radians: " + absoluteRadians + " " + shoulderRadians);
        return absoluteRadians - elbowRadiansAbsolute + ArmSubsystem.getInstance().getHomeDegrees(Motor.ELBOW_JOINT) / 180.0 * Math.PI;
    }
}