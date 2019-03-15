package frc.robot.states;

import frc.robot.commands.arm.ArmSequences;
import frc.robot.helpers.Triplet;
import frc.robot.motionProfiling.MotionTriplet;
import frc.robot.sequencing.Sequence;

public class ArmSuperState {
    public enum ArmState {
        disabled, home, intakeHatch, intakeCargo, adjustingPosition, lowRocketHatch, midRocketHatch, highRocketHatch, lowRocketCargo, midRocketCargo, cargoShip
    }
    public enum ArmMotionState {
        still, followingMotionProfiles
    }

    private ArmState armState;

    public ArmSuperState() {
        armState = ArmState.disabled;
    }

    public ArmState getState() {
        return this.armState;
    }

    public Triplet<Sequence<MotionTriplet>> setState(ArmState newState) {
        if (this.armState == newState) return null;
        this.armState = newState;
        System.out.println("new state: " + newState);
        switch (newState) {
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
        }
        return null;
    }
}