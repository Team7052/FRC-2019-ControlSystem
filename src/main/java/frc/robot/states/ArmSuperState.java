package frc.robot.states;

import frc.robot.helpers.Pair;
import frc.robot.helpers.Triplet;
import frc.robot.motionProfiling.FilterOutputModifier;
import frc.robot.motionProfiling.FilterStep;
import frc.robot.motionProfiling.MotionTriplet;
import frc.robot.motionProfiling.TrapezoidShape;
import frc.robot.motionProfiling.TrapezoidalFunctions;
import frc.robot.sequencing.DelayStep;
import frc.robot.sequencing.Sequence;
import frc.robot.states.substates.ArmState;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.Motor;
import frc.robot.util.physics.PhysicsConstants;
import frc.robot.util.physics.PhysicsWorld;

public class ArmSuperState extends SuperState<ArmState> {
    public enum MotionState {
        waitingForMotion, followingMotionProfiles, finishedMotion
    }

    private ArmSuperStateDelegate shoulderDelegate, elbowDelegate, wristDelegate;

    private MotionState armMotionState;
    private MotionState shoulderMotionState, elbowMotionState, wristMotionState;

    public ArmSuperState() {
        systemState = ArmState.disabled;
        armMotionState = MotionState.waitingForMotion;
        shoulderMotionState = MotionState.waitingForMotion;
        elbowMotionState = MotionState.waitingForMotion;
        wristMotionState = MotionState.waitingForMotion;
    }

    public void addShoulderDelegate(ArmSuperStateDelegate delegate) {
        this.shoulderDelegate = delegate;
    }
    public void addElbowDelegate(ArmSuperStateDelegate delegate) {
        this.elbowDelegate = delegate;
    }
    public void addWristDelegate(ArmSuperStateDelegate delegate) {
        this.wristDelegate = delegate;
    }

    @Override
    public void setState(ArmState newState) {
        if (this.systemState == newState) return;
        /*if (this.systemState == ArmState.autonomousHome  && newState == ArmState.home) return;
        if (this.systemState == ArmState.disabled && newState == ArmState.home) {
            this.systemState = ArmState.autonomousHome;
        }
        else {*/
            this.systemState = newState;
        //}
        this.armMotionState = MotionState.waitingForMotion;
    }

    @Override
    public void update() {
        if (this.armMotionState == MotionState.waitingForMotion) {
            Triplet<Sequence<MotionTriplet>> triplet = getSequenceFromArmState();
            if (triplet != null) {
                this.setMotionStates(MotionState.followingMotionProfiles);
                // add a callback to each sequence to know when it is finished
                triplet.a.callback = () -> this.shoulderMotionState = MotionState.finishedMotion;
                triplet.b.callback = () -> this.elbowMotionState = MotionState.finishedMotion;
                triplet.c.callback = () -> this.wristMotionState = MotionState.finishedMotion;

                // trigger delegate to update sequence in each of the joint controllers
                if (shoulderDelegate != null) shoulderDelegate.setSequence(triplet.a);
                if (elbowDelegate != null) elbowDelegate.setSequence(triplet.b);
                if (wristDelegate != null) wristDelegate.setSequence(triplet.c);
            }
        }
        else if (this.armMotionState == MotionState.followingMotionProfiles) {
            if (shoulderMotionState == MotionState.finishedMotion && elbowMotionState == MotionState.finishedMotion && wristMotionState == MotionState.finishedMotion) {
                this.setMotionStates(MotionState.finishedMotion);
                if (this.systemState == ArmState.lowerArm || this.systemState == ArmState.raiseArm) {
                    this.systemState = ArmState.adjustedPosition;
                }
                if (this.systemState == ArmState.autonomousHome) {
                    this.systemState = ArmState.home;
                }
            }
        }

        /* Disable the wrist motor if trying to pull out */
        if (this.systemState == ArmState.pullOutSequence) {
            wristDelegate.setEnabled(false);
        }
        else {
            wristDelegate.setEnabled(true);
        }
    }

    private void setMotionStates(MotionState state) {
        armMotionState = state;
        shoulderMotionState = state;
        elbowMotionState = state;
        wristMotionState = state;
    }

    public Triplet<Sequence<MotionTriplet>> getSequenceFromArmState() {
        switch (this.systemState) {
            case home:
                return ArmSequences.homeSequence();
            case autonomousHome:
                return ArmSequences.autonomousHomeSequence();
            case intakeHatch:
                return ArmSequences.intakeHatchSequence();
            case pullOutSequence:
                return ArmSequences.pullOutSequence();
            case intakeCargo:
                return ArmSequences.intakeCargoSequence();
            case lowRocketHatch:
                return ArmSequences.lowerRocketHatchSequence();
            case midRocketHatch:
                return ArmSequences.midRocketHatchSequence();
            case highRocketHatch:
                return ArmSequences.highRocketHatchSequence();
            case lowRocketCargo:
                return ArmSequences.lowRocketCargoSequence();
            case midRocketCargo:
                return ArmSequences.midRocketCargoSequence();
            case raiseArm:
                System.out.println(this.armMotionState + " " + this.systemState);
                if (this.systemState == ArmState.home && (this.armMotionState == MotionState.waitingForMotion || this.armMotionState == MotionState.finishedMotion)) return null;
                return ArmSequences.raiseArmSequence();
            case lowerArm:
                if (this.systemState == ArmState.home && this.armMotionState != MotionState.waitingForMotion) return null;
                return ArmSequences.lowerArmSequence();
            default:
                return null;
        }
    }
}

class ArmSequences {
    public static final double shoulderMaxVelocity = Math.PI * 2 / 5;
    public static final double shoulderMaxAcceleration = Math.PI * 2 / 3;

    public static final double elbowMaxVelocity = Math.PI;
    public static final double elbowMaxAcceleration = Math.PI;
    
    public static Triplet<Sequence<MotionTriplet>> homeSequence() {
        return toSequence(generateProfiles(radians(25), radians(180), radians(180)));
    }
    public static Triplet<Sequence<MotionTriplet>> autonomousHomeSequence() {
        ArmSubsystem arm = ArmSubsystem.getInstance();
        double initShoulder = arm.getAbsoluteDegrees(Motor.SHOULDER_JOINT);
        double initElbow = arm.getAbsoluteDegrees(Motor.ELBOW_JOINT);
        Triplet<FilterStep<MotionTriplet>> sequences = generateProfiles(initShoulder, initElbow, radians(180));
        Triplet<FilterStep<MotionTriplet>> newSequences = generateProfiles(initShoulder, initElbow, radians(180), radians(25), radians(180), radians(180));
        Sequence<MotionTriplet> shoulderSeq = new Sequence<>();
        Sequence<MotionTriplet> elbowSeq = new Sequence<>();
        Sequence<MotionTriplet> wristSeq = new Sequence<>();

        shoulderSeq.addStep(sequences.a);
        shoulderSeq.addStep(newSequences.a);
        elbowSeq.addStep(sequences.b);
        elbowSeq.addStep(newSequences.b);
        wristSeq.addStep(sequences.c);
        wristSeq.addStep(newSequences.c);

        return new Triplet<>(shoulderSeq, elbowSeq, wristSeq);
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
        return toSequence(setDistances(5, 77, radians(180)));
    }

    public static Triplet<Sequence<MotionTriplet>> intakeCargoSequence() {
        return toSequence(generateProfiles(radians(40),  radians(160), radians(150)));
    }
    public static Triplet<Sequence<MotionTriplet>> lowRocketCargoSequence() {
        return toSequence(generateProfiles(radians(60), radians(70), radians(90)));
    }
    public static Triplet<Sequence<MotionTriplet>> midRocketCargoSequence() {
        return toSequence(generateProfiles(radians(60), radians(70), radians(90)));
    }

    public static Triplet<Sequence<MotionTriplet>> raiseArmSequence() {
        // get current displacements
        Pair<Double> currentDisplacements = PhysicsWorld.getInstance().solveArmKinematics();
        double normalized_l = currentDisplacements.a - PhysicsConstants.backToArm - PhysicsConstants.thickness / 2 + PhysicsConstants.hand;
        double normalized_h = PhysicsConstants.armHeight + PhysicsConstants.baseHeight - currentDisplacements.b;
        return toSequence(setDistances(normalized_l, normalized_h + 3, radians(190)));
    }
    public static Triplet<Sequence<MotionTriplet>> lowerArmSequence() {
        // get current displacements
        Pair<Double> currentDisplacements = PhysicsWorld.getInstance().solveArmKinematics();
        double normalized_l = currentDisplacements.a - PhysicsConstants.backToArm - PhysicsConstants.thickness / 2 + PhysicsConstants.hand;
        double normalized_h = PhysicsConstants.armHeight + PhysicsConstants.baseHeight - currentDisplacements.b;
        return toSequence(setDistances(normalized_l, normalized_h - 2, radians(180)));
    }
    public static Triplet<Sequence<MotionTriplet>> pullOutSequence() {
        // get current displacements
        Pair<Double> currentDisplacements = PhysicsWorld.getInstance().solveArmKinematics();
        double normalized_l = currentDisplacements.a - PhysicsConstants.backToArm - PhysicsConstants.thickness / 2 + PhysicsConstants.hand;
        double normalized_h = PhysicsConstants.armHeight + PhysicsConstants.baseHeight - currentDisplacements.b;
        return toSequence(setDistances(normalized_l, normalized_h - 2, radians(180)));
    }

    private static Triplet<FilterStep<MotionTriplet>> setDistances(double l, double h, double wristRadians) {
        Pair<Double> angles = PhysicsWorld.getInstance().armInverseKinematics(l + PhysicsConstants.backToArm + PhysicsConstants.thickness / 2 - PhysicsConstants.hand, PhysicsConstants.armHeight + PhysicsConstants.baseHeight - h);
        return generateProfiles(angles.a, angles.b, wristRadians);
    }

    private static double radians(double degrees) {
        return degrees / 180.0 * Math.PI;
    }

    private static Triplet<Sequence<MotionTriplet>> toSequence(Triplet<FilterStep<MotionTriplet>> triplet) {
        Triplet<Sequence<MotionTriplet>> sequence = new Triplet<>(new Sequence<>(), new Sequence<>(), new Sequence<>());
        sequence.a.addStep(triplet.a);
        sequence.b.addStep(triplet.b);
        sequence.c.addStep(triplet.c);
        return sequence;
    }

    private static Triplet<FilterStep<MotionTriplet>> generateProfiles(double shoulderAngle, double elbowAngle, double wristAngle) {
        double initShoulder = ArmSubsystem.getInstance().getDegrees(Motor.SHOULDER_JOINT) / 180 * Math.PI;
        double endShoulder = shoulderAngle;
        double initElbow = ArmSubsystem.getInstance().getAbsoluteDegrees(Motor.ELBOW_JOINT) / 180 * Math.PI;
        double endElbow =  elbowAngle;
        double initWrist = ArmSubsystem.getInstance().getAbsoluteDegrees(Motor.WRIST_JOINT) / 180.0 * Math.PI;
        double endWrist = wristAngle;

        return generateProfiles(initShoulder, initElbow, initWrist, endShoulder, endElbow, endWrist);
    }

    private static Triplet<FilterStep<MotionTriplet>> generateProfiles(double initShoulder, double initElbow, double initWrist, double endShoulder, double endElbow, double endWrist) {
        // trapezoidal motion profiling.
        TrapezoidShape initShoulderShape = TrapezoidalFunctions.generateTrapezoidShape(initShoulder, endShoulder, shoulderMaxVelocity, shoulderMaxAcceleration);
        TrapezoidShape initElbowShape = TrapezoidalFunctions.generateTrapezoidShape(initElbow, endElbow, elbowMaxVelocity, elbowMaxAcceleration);
        Pair<TrapezoidShape> newShapes = TrapezoidalFunctions.syncTrapezoidShapes(initShoulderShape, initElbowShape);
        // generate nonsensical elbow shape at beginning
        FilterStep<MotionTriplet> shoulderProfile = FilterStep.trapezoidalProfileFilter(newShapes.a, initShoulder);

        FilterOutputModifier<MotionTriplet> elbowFilter = (dt, endTime, triplet) -> {
            double absoluteShoulderPosition = shoulderProfile.getUpdateForDeltaTime(dt).getPosition();
            double relativeElbow = radiansElbowRelativeToShoulder(triplet.getPosition(), absoluteShoulderPosition);
            return new MotionTriplet(relativeElbow, triplet.getVelocity(), triplet.getAcceleration());
        };
        FilterStep<MotionTriplet> elbowProfile = FilterStep.trapezoidalProfileFilter(newShapes.b, initElbow);
        elbowProfile.addFilter(elbowFilter);
        // interpolate wrist points
        FilterOutputModifier<MotionTriplet> wristFilter = (dt, endTime, triplet) -> {
            double absoluteWristPosition = initWrist + (endWrist - initWrist) * (dt / endTime);

            double absoluteShoulderPosition = shoulderProfile.getUpdateForDeltaTime(dt).getPosition();
            double relativeElbowPosition = elbowProfile.getUpdateForDeltaTime(dt).getPosition();
            double absoluteElbowPosition = relativeElbowToAbsolute(relativeElbowPosition, absoluteShoulderPosition);
            double relativeWristPosition = radiansWristRelativeToElbow(absoluteWristPosition, absoluteElbowPosition);
            return new MotionTriplet(relativeWristPosition, 0.0, 0.0);
        };
        // take elbow absolute angles and transform the wrist to match those angles
        FilterStep<MotionTriplet> wristProfile = new FilterStep<MotionTriplet>(wristFilter, () -> newShapes.a.totalTime());

        return new Triplet<>(shoulderProfile, elbowProfile, wristProfile);
    }

    private static double radiansElbowRelativeToShoulder(double absoluteRadians, double shoulderRadians) {
        //System.out.println("Radians: " + absoluteRadians + " " + shoulderRadians);
        return absoluteRadians - shoulderRadians + ArmSubsystem.getInstance().getHomeDegrees(Motor.SHOULDER_JOINT) / 180.0 * Math.PI;
    }

    private static double radiansWristRelativeToElbow(double absoluteRadians, double elbowRadiansAbsolute) {
        //System.out.println("Radians: " + absoluteRadians + " " + shoulderRadians);
        return absoluteRadians - elbowRadiansAbsolute + ArmSubsystem.getInstance().getHomeDegrees(Motor.ELBOW_JOINT) / 180.0 * Math.PI;
    }

    private static double relativeElbowToAbsolute(double relativeElbow, double absoluteShoulder) {
        return relativeElbow + absoluteShoulder - ArmSubsystem.getInstance().getHomeDegrees(Motor.SHOULDER_JOINT) / 180 * Math.PI;
    }
}