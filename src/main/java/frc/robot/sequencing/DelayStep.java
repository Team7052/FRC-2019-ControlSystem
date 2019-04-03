package frc.robot.sequencing;

/**
 * Step that does nothing for a specified amount of time
 * @param <T>
 */
public class DelayStep<T> extends Step<T> {
    /**
     * Create a step that does nothing for a period in time.
     * @param delayTime The duration of the delay in seconds
     */
    public DelayStep(double delayTime) {
        super(() -> delayTime);
    }
    @Override
    public T getUpdateForDeltaTime(double dt) {
        return null;
    }
}