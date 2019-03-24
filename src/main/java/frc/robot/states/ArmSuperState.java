package frc.robot.states;

import frc.robot.commands.arm.ArmSequences;
import frc.robot.helpers.Triplet;
import frc.robot.motionProfiling.MotionTriplet;
import frc.robot.sequencing.Sequence;
import frc.robot.states.substates.ArmState;

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
        this.systemState = newState;
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
            }
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
            case intakeHatch:
                return ArmSequences.intakeHatchSequence();
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