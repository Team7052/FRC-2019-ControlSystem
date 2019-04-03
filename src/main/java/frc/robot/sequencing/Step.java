package frc.robot.sequencing;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import frc.robot.util.Callback;

/**
 * The Step class is designed to be inherited. Can't instantiate Step directly.
 * Designed to run on a timed loop. Call start in order to start the step
 * Step doesn't run on a separate thread itself and does not contain its own timer.
 * Instead, feed a timestamp when calling start, update, or isFinished the step
 * Step takes the timestamp and returns whatever the value should be for the given 
 * timestamp and will stop itself if the time runs past it's total running time.
 * @param <T> 
 */
public abstract class Step<T> {
    /**
     * Manages the state of the step
     */
    protected StepState state = StepState.IDLE;
    /**
     * Start time of the step
     */
    protected double startTime = 0;
    /**
     * {@link DynamicEndTime}, an interface that allows the total running time to be calculated based on a method instead of static
     */
    protected DynamicEndTime totalRunningTime;
    private ArrayList<Callback> callbacks;

    /**
     * Lookup table to increase efficiency when a step is called multiple times with the same timestamp
     */
    Map<Double, T> lookupTable = new HashMap<>();

    /**
     * Initializer
     * @param runningTime specify a static total running time for the step
     */
    public Step(double runningTime) {
        this.totalRunningTime = () -> runningTime;
        this.callbacks = new ArrayList<>();
    }

    /**
     * Initializer
     * @param totalRunningTime specify a dynamic run time for the step
     */
    public Step(DynamicEndTime totalRunningTime) {
        this.totalRunningTime = totalRunningTime;
        this.callbacks = new ArrayList<>();
    }
    /**
     * Initializer without specifying a total run time. The total run time is 
     * assumed to be 0s. Must initialize total run time before starting step function
     */
    public Step() {
        this.totalRunningTime = () -> 0.0;
        this.callbacks = new ArrayList<>();
    }

    /**
     * 
     * @return the total time the step is designed to run for
     */
    public final double getTotalTime() {
        return this.totalRunningTime.get();
    }

    /**
     * Update function only updates if the step has started and is still running. 
     * If the running time of the step is past it's total run time, then it will stop itself.
     * Before calculating an update for a given timestamp, it will check a lookout table to see if it has been calculated before.
     * @param timeStamp the current timestamp. Will compare current timestamp to the start timestamp and calculate a value for 
     * @return T, the current type for the update function
     */
    public final T update(double timeStamp) {
        if (this.state == StepState.RUNNING) {
            double dt = timeStamp - this.startTime;
            // if the delta time exceeds the total running time, then end the step
            if (dt > this.totalRunningTime.get()) {
                this.endStep();
                dt = this.totalRunningTime.get();
            }
            // if the given dt has already been calculated before, return the value already in the lookup table
            if (lookupTable.containsKey(dt)) return lookupTable.get(timeStamp);
            // calculate the new value for the current deltatime
            return this.getUpdateForDeltaTime(dt);
        }
        // if the step is not running, then return null
        return null;
    }

    /**
     * Get the value of the last update of the step (right when the total time is reached)
     * @return T return the last value of the step (when the total time of the step is reached)
     */
    public final T getLastUpdate() {
        return this.getUpdateForDeltaTime(this.getTotalTime());
    }

    /**
     * Override this function when subclassing, call this function to calculate the result at an exact period of time
     * @return T returns the result for a given time for the specified data type of the step
     */
    public abstract T getUpdateForDeltaTime(double dt);

    /**
     * Start running the step (set a start time)
     * @param timeStamp specify the timestamp for when the step is started (usually pass through the current timestamp with Timer.getFPGATimestamp())
     */
    public final void start(double timeStamp) {
        if (this.state == StepState.IDLE) {
            this.state = StepState.RUNNING;
            this.startTime = timeStamp;
        }
    }

    /**
     * A boolean that specifies whether the step has began
     * @return returns true or false
     */
    public final boolean hasBegan() {
        return this.state != StepState.IDLE;
    }
    /**
     * Determine if the step is currently running.
     * If the time passed is greater than the total running time, then set running equal to false.
     * @param timeStamp pass the current timestamp. (Usually, pass through Timer.getFPGATimestamp())
     * @return returns a boolean to determine whether the step is currently running
     */
    public final boolean isRunning(double timeStamp) {
        if (timeStamp - startTime > this.totalRunningTime.get()) {
            this.endStep();
        }
        return this.state == StepState.RUNNING;
    }

    /**
     * Get the timestamp when the step started
     * @return double that contains the start time of the step
     */
    public final double getStartTime() {
        return this.startTime;
    }

    /**
     * Determine whether the step is still running. If the delta time (calculated from the timestamp passed) is greater than the total running time and the current step is running, it will stop the step as well.
     * @param timeStamp pass the current timestamp. (Usually, pass through Timer.getFPGATimestamp())
     * @return a boolean true or false
     */
    public final boolean isFinished(double timeStamp) {
        if (this.state == StepState.RUNNING && timeStamp - startTime >= this.totalRunningTime.get()) {
            this.endStep();
        }
        return this.state == StepState.FINISHED;
    }

    /**
     * Add a callback to the step to be called when the step finishes
     * @param callback pass a {@link Callback} (you can pass it through a lambda expression: () -> {})
     */
    public final void addCallback(Callback callback) {
        this.callbacks.add(callback);
    }

    /**
     * Stop the step no matter what. Executes all the callbacks
     */
    public final void endStep() {
        this.state = StepState.FINISHED;
        // execute callbacks
        for (Callback callback: this.callbacks) {
            callback.didFinish();
        }
    }

    /**
     * Get the current state of the step
     * @return {@link StepState}
     */
    public final StepState getState() {
        return this.state;
    }
}