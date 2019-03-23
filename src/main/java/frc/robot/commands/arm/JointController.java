package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.motionProfiling.MotionTriplet;
import frc.robot.sequencing.Sequence;
import frc.robot.states.ArmSuperStateDelegate;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.Motor;

public class JointController implements ArmSuperStateDelegate {
    private ArmSubsystem arm;
    private Motor motor;
    Sequence<MotionTriplet> jointSequence;

    public JointController(Motor motor) {
        arm = ArmSubsystem.getInstance();
        this.motor = motor;
        jointSequence = new Sequence<>();
    }

    double prev = 0;

    public void execute() {
        if (prev == 0) prev = Timer.getFPGATimestamp();
        double timestamp = Timer.getFPGATimestamp();
        if (!jointSequence.hasBegan()) {
            System.out.println(timestamp - prev);
        }
        if (!jointSequence.hasBegan()) jointSequence.start(timestamp);
        MotionTriplet triplet = jointSequence.update(timestamp);

        

        if (jointSequence.isRunning() && triplet != null) {
            // a = position
            double position = triplet.a;
            arm.setDegrees(motor, position / Math.PI * 180);
        }
        if (jointSequence.isFinished(timestamp)) {
            MotionTriplet lastTriplet = jointSequence.getLastUpdate();
            if (lastTriplet != null) {
                double position = lastTriplet.a;
                arm.setDegrees(motor, position / Math.PI * 180);
            }
        }
        prev = timestamp;
    }

    @Override
    public void setSequence(Sequence<MotionTriplet> sequence) {
        this.jointSequence = sequence;
    }
}