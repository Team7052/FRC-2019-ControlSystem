package frc.robot.commands.arm;

import frc.robot.util.physics.PhysicsConstants;
import frc.robot.util.physics.PhysicsWorld;

import java.util.ArrayList;

import frc.robot.helpers.Pair;
import frc.robot.helpers.Triplet;
import frc.robot.motionProfiling.FunctionGenerator;
import frc.robot.motionProfiling.MotionProfiler;
import frc.robot.motionProfiling.MotionTriplet;
import frc.robot.motionProfiling.Point;
import frc.robot.motionProfiling.TrapezoidalFunctions;
import frc.robot.sequencing.Sequence;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.Motor;

public class ArmSequences {
    public static final double shoulderMaxVelocity = Math.PI * 2 / 5;
    public static final double shoulderMaxAcceleration = Math.PI * 2 / 3;

    public static final double elbowMaxVelocity = Math.PI;
    public static final double elbowMaxAcceleration = Math.PI;
    
    public static Triplet<Sequence<MotionTriplet>> homeSequence() {
        return toSequence(generateProfiles(radians(25), radians(180), radians(180)));
    }
    public static Triplet<Sequence<MotionTriplet>> intakeHatchSequence() {
        return toSequence(setDistances(16, 20, radians(180)));
    }
    public static Triplet<Sequence<MotionTriplet>> lowerRocketHatchSequence() {
        return toSequence(setDistances(16, 26, radians(180)));
    }
    public static Triplet<Sequence<MotionTriplet>> midRocketHatchSequence() {
        return toSequence(setDistances(14, 52, radians(180)));
    }
    public static Triplet<Sequence<MotionTriplet>> highRocketHatchSequence() {
        return toSequence(setDistances(2, 78, radians(180)));
    }

    public static Triplet<Sequence<MotionTriplet>> raiseArmSequence() {
        // get current displacements
        Pair<Double> currentDisplacements = PhysicsWorld.getInstance().solveArmKinematics();
        double normalized_l = currentDisplacements.a - PhysicsConstants.backToArm - PhysicsConstants.thickness / 2 + PhysicsConstants.hand;
        double normalized_h = PhysicsConstants.armHeight + PhysicsConstants.baseHeight - currentDisplacements.b;
        System.out.println("Raise the arm");
        return toSequence(setDistances(normalized_l, normalized_h + 3, radians(180)));
    }
    public static Triplet<Sequence<MotionTriplet>> lowerArmSequence() {
        // get current displacements
        Pair<Double> currentDisplacements = PhysicsWorld.getInstance().solveArmKinematics();
        double normalized_l = currentDisplacements.a - PhysicsConstants.backToArm - PhysicsConstants.thickness / 2 + PhysicsConstants.hand;
        double normalized_h = PhysicsConstants.armHeight + PhysicsConstants.baseHeight - currentDisplacements.b;
        System.out.println("Lower the arm");
        return toSequence(setDistances(normalized_l, normalized_h - 2, radians(180)));
    }
    public static Triplet<Sequence<MotionTriplet>> intakeCargoSequence() {
        return toSequence(generateProfiles(radians(40),  radians(160), radians(270)));
    }
    public static Triplet<Sequence<MotionTriplet>> lowRocketCargoSequence() {
        return toSequence(generateProfiles(radians(60), radians(70), radians(270)));
    }
    public static Triplet<Sequence<MotionTriplet>> midRocketCargoSequence() {
        return toSequence(generateProfiles(radians(60), radians(70), radians(270)));
    }
    private static Triplet<MotionProfiler> setDistances(double l, double h, double wristRadians) {
        Pair<Double> angles = PhysicsWorld.getInstance().armInverseKinematics(l + PhysicsConstants.backToArm + PhysicsConstants.thickness / 2 - PhysicsConstants.hand, PhysicsConstants.armHeight + PhysicsConstants.baseHeight - h);
        return generateProfiles(angles.a, angles.b, wristRadians);
    }

    private static double radians(double degrees) {
        return degrees / 180.0 * Math.PI;
    }

    private static Triplet<Sequence<MotionTriplet>> toSequence(Triplet<MotionProfiler> triplet) {
        Triplet<Sequence<MotionTriplet>> sequence = new Triplet<>(new Sequence<>(), new Sequence<>(), new Sequence<>());
        sequence.a.addStep(triplet.a);
        sequence.b.addStep(triplet.b);
        sequence.c.addStep(triplet.c);
        return sequence;
    }

    private static Triplet<MotionProfiler> generateProfiles(double shoulderAngle, double elbowAngle, double wristAngle) {
        double initShoulder = ArmSubsystem.getInstance().getDegrees(Motor.SHOULDER_JOINT) / 180 * Math.PI;
        double endShoulder = shoulderAngle;
        double initElbow = ArmSubsystem.getInstance().getAbsoluteDegrees(Motor.ELBOW_JOINT) / 180 * Math.PI;
        double endElbow =  elbowAngle;
        double initWrist = ArmSubsystem.getInstance().getAbsoluteDegrees(Motor.WRIST_JOINT) / 180.0 * Math.PI;
        double endWrist = wristAngle; 

        return generateProfiles(initShoulder, initElbow, initWrist, endShoulder, endElbow, endWrist);
    }
    private static Triplet<MotionProfiler> generateProfiles(double initShoulder, double initElbow, double initWrist, double endShoulder, double endElbow, double endWrist) {
        // trapezoidal motion profiling.
        MotionProfiler shoulderMotionProfiler = new MotionProfiler();
        MotionProfiler elbowMotionProfiler = new MotionProfiler();
        MotionProfiler wristMotionProfiler = new MotionProfiler();

        ArrayList<Point> shoulderShape = TrapezoidalFunctions.generateTrapezoidalProfile(initShoulder, endShoulder, shoulderMaxVelocity, shoulderMaxAcceleration);
        ArrayList<Point> elbowShape = TrapezoidalFunctions.generateTrapezoidalProfile(initElbow, endElbow, elbowMaxVelocity, elbowMaxAcceleration);
        Pair<ArrayList<Point>> trapezoidalProfiles = TrapezoidalFunctions.matchedTotalTimeForShapes(shoulderShape, elbowShape);
        shoulderMotionProfiler.setVelocityPoints(FunctionGenerator.getLinearInterpolation(trapezoidalProfiles.a, 0.01), initShoulder);
        elbowMotionProfiler.setVelocityPoints(FunctionGenerator.getLinearInterpolation(trapezoidalProfiles.b, 0.01), initElbow);

        // generate wrist profile
        ArrayList<Point> wristPoints = new ArrayList<>();

        // interpolate wrist points
        double totalTime = elbowMotionProfiler.getTotalTime();
        for (Point point: elbowMotionProfiler.getPositionFunction()) {
            double percentage = point.x / totalTime;
            wristPoints.add(new Point(point.x, initWrist + (endWrist - initWrist) * percentage));
        }
        // take elbow absolute angles and transform the wrist to match those angles
        ArrayList<Point> relativeWristPoints = getRelativeWristPositionProfile(wristPoints, elbowMotionProfiler.getPositionFunction());
        wristMotionProfiler.setPositionPoints(relativeWristPoints);
        ArrayList<Point> newElbowPositions = getRelativeElbowPositionProfile(elbowMotionProfiler.getPositionFunction(), shoulderMotionProfiler.getPositionFunction());
        elbowMotionProfiler.setPositionPoints(newElbowPositions);

        return new Triplet<>(shoulderMotionProfiler, elbowMotionProfiler, wristMotionProfiler);
    }

    private static ArrayList<Point> getRelativeWristPositionProfile(ArrayList<Point> absoluteWristPoints, ArrayList<Point> absoluteElbowPoints) {
        ArrayList<Point> relativeWristPoints = new ArrayList<>();
        
        for (int i = 0; i < absoluteWristPoints.size(); i++) {
            Point point = absoluteWristPoints.get(i);
            relativeWristPoints.add(new Point(point.x, radiansWristRelativeToElbow(point.y, absoluteElbowPoints.get(i).y)));

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
                relativeElbowPositions.add(new Point(point.x, radiansElbowRelativeToShoulder(point.y, absoluteShoulderPositionPoints.get(shoulderIndex).y)));
            }
        }
        else {
            for (int i = 0; i < absoluteShoulderPositionPoints.size(); i++) {
                // get shoulder profile position for time: x
                Point point = i < absoluteElbowPositionPoints.size() ? absoluteElbowPositionPoints.get(i) : absoluteElbowPositionPoints.get(absoluteElbowPositionPoints.size() - 1);
                relativeElbowPositions.add(new Point(point.x, radiansElbowRelativeToShoulder(point.y, absoluteShoulderPositionPoints.get(i).y)));
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