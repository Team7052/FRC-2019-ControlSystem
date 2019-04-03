package frc.robot.motionProfiling;

/**
 * This is an interface that acts as a filter to the original data type. 
 * Implement this interface to create a filter that modifies the current value
 * of a data type based on the current time and total time.
 * @param <T> The data type of the filter being inputted and outputted
 */
public interface FilterOutputModifier<T> {
    /**
     * transforms a the value current data type into a new value.
     * May use the currentValue to generate new value of the filter
     * @param dt delta time, or the time passed since the filter started
     * @param totalTime the total time that the filter will run for
     * @param currentValue the current value of the filter, use the current value to generate your new one
     * @return the transformed value for a new given delta time in the filter.
     */
    public T transform(double dt, double totalTime, T currentValue);
}