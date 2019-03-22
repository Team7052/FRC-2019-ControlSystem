package frc.robot.states;

import frc.robot.motionProfiling.MotionTriplet;
import frc.robot.sequencing.Sequence;

public interface ClimberSuperStateDelegate {
    void updateSequence(Sequence<MotionTriplet> sequence);
}