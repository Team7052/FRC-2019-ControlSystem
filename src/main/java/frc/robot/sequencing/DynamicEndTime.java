package frc.robot.sequencing;

/**
 * This interface is overriden to return a dynamic end time instead of a static value.
 */
public interface DynamicEndTime {
    /**
     * Execute to calculate dynamically the total time
     * @return The total calculated time
     */
    double get();
}