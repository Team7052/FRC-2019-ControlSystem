package frc.robot.sequencing;

import java.util.ArrayList;

/**
 * A sequence takes a series of steps and executes them sequentially.
 * It inherits {@link frc.robot.sequencing.Step} so to use it is the same functionality as a Step.
 * @param <T> The data type of the result of each step
 */
public class Sequence<T> extends Step<T> {
    private ArrayList<Step<T>> steps;

    /**
     * Create an empty sequence without any steps
     */
    public Sequence() {
        steps = new ArrayList<>();
        this.totalRunningTime = () -> {
            double totalTime = 0;
            for (Step<T> step: steps) {
                totalTime += step.totalRunningTime.get();
            }
            return totalTime;
        };
    }

    /**
     * Add a step to the sequence (with the same data type as the sequence).
     * Cannot add steps if the sequence is already running.
     * @param step Step to be added
     * @return whether add step was successful
     */
    public boolean addStep(Step<T> step) {
        if (this.hasBegan()) return false;
        if (steps.size() > 0) {
            Step<T> prevStep = steps.get(steps.size() - 1);
            step.startTime = prevStep.startTime + prevStep.totalRunningTime.get();
        }
        else {
            step.startTime = 0;
        }
        this.steps.add(step);
        return true;
    }

    @Override
    public T getUpdateForDeltaTime(double dt) {
        /** Override from Step to calculate the result at any given time */
        double runningTotalTimeSum = 0.0;
        for (Step<T> step: this.steps) {
            if (step.totalRunningTime.get() >= dt - runningTotalTimeSum) {
                return step.getUpdateForDeltaTime(dt - runningTotalTimeSum);
            }
            runningTotalTimeSum += step.totalRunningTime.get();
        }
        return null;
    }

    /**
     * Check if there are any steps added to the sequence
     * @return True or false
     */
    public final boolean isEmpty() {
        return this.steps.size() == 0;
    }
}