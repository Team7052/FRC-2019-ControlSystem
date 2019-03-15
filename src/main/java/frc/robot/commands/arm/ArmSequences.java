package frc.robot.commands.arm;

import frc.robot.PhysicsConstants;
import frc.robot.PhysicsWorld;
import frc.robot.helpers.Pair;
import frc.robot.helpers.Triplet;
import frc.robot.motionProfiling.MotionProfiler;
import frc.robot.motionProfiling.MotionTriplet;
import frc.robot.sequencing.Sequence;

public class ArmSequences {
    public static final double shoulderMaxVelocity = Math.PI * 2 / 5;
    public static final double shoulderMaxAcceleration = Math.PI * 2 / 3;

    public static final double elbowMaxVelocity = Math.PI;
    public static final double elbowMaxAcceleration = Math.PI;
    
    public static Triplet<Sequence<MotionTriplet>> homeSequence() {
        return toSequence(CoupledArmProfiler.generateProfiles(radians(25), radians(180), radians(180)));
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
    public static Triplet<Sequence<MotionTriplet>> intakeCargoSequence() {
        return toSequence(CoupledArmProfiler.generateProfiles(radians(40),  radians(160), radians(180)));
    }
    public static Triplet<Sequence<MotionTriplet>> lowRocketCargoSequence() {
        return toSequence(CoupledArmProfiler.generateProfiles(radians(60), radians(70), radians(0)));
    }
    public static Triplet<Sequence<MotionTriplet>> midRocketCargoSequence() {
        return toSequence(CoupledArmProfiler.generateProfiles(radians(60), radians(70), radians(90)));
    }

    private static Triplet<MotionProfiler> setDistances(double l, double h, double wristRadians) {
        Pair<Double> angles = PhysicsWorld.getInstance().inverseKinematics(l + PhysicsConstants.backToArm + PhysicsConstants.thickness - PhysicsConstants.hand, PhysicsConstants.armHeight + PhysicsConstants.baseHeight - h);
        return CoupledArmProfiler.generateProfiles(angles.a, angles.b, wristRadians);
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
}