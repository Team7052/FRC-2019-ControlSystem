package frc.robot.sequencing;

public class DelayStep<T> extends Step<T> {
    public DelayStep(double delayTime) {
        super(() -> delayTime);
    }
    @Override
    public T getUpdateForDeltaTime(double dt) {
        return null;
    }

}