package frc.robot.states;

import frc.robot.motionProfiling.MotionTriplet;
import frc.robot.sequencing.Sequence;

public interface ArmSuperStateDelegate {
    void setSequence(Sequence<MotionTriplet> sequence);
    void setEnabled(boolean isEnabled);
}
