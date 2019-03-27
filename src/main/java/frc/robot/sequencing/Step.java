package frc.robot.sequencing;

import java.util.HashMap;
import java.util.Map;

import frc.robot.util.Callback;

public abstract class Step<T> {
    protected StepState state = StepState.IDLE;
    protected double startTime = 0;
    protected DynamicEndTime totalRunningTime;
    public Callback callback;

    Map<Double, T> lookupTable = new HashMap<>();


    public Step(double runningTime) {
        this.totalRunningTime = () -> runningTime;
    }

    public final double getTotalTime() {
        return this.totalRunningTime.get();
    }

    public Step(DynamicEndTime totalRunningTime) {
        this.totalRunningTime = totalRunningTime;
    }
    public Step() {
        this.totalRunningTime = () -> 0.0;
    }

    public final T update(double timeStamp) {
        if (this.state == StepState.RUNNING) {
            double dt = timeStamp - this.startTime;
            if (dt > this.totalRunningTime.get()) {
                this.endStep();
                dt = this.totalRunningTime.get();
            }
            if (lookupTable.containsKey(dt)) return lookupTable.get(timeStamp);
            return this.getUpdateForDeltaTime(dt);
        }
        return null;
    }
    public final T getLastUpdate() {
        return this.getUpdateForDeltaTime(this.getTotalTime());
    }
    public abstract T getUpdateForDeltaTime(double dt);

    public final void start(double timeStamp) {
        if (this.state == StepState.IDLE) {
            this.state = StepState.RUNNING;
            this.startTime = timeStamp;
        }
    }

    public final boolean hasBegan() {
        return this.state != StepState.IDLE;
    }

    public final boolean isRunning() {
        return this.state == StepState.RUNNING;
    }

    public final double getStartTime() {
        return this.startTime;
    }

    public final boolean isFinished(double timeStamp) {
        if (this.state == StepState.RUNNING && timeStamp - startTime >= this.totalRunningTime.get()) {
            this.state = StepState.FINISHED;
            if (this.callback != null) callback.didFinish();
        }
        return this.state == StepState.FINISHED;
    }

    public final void endStep() {
        this.state = StepState.FINISHED;
        if (this.callback != null) callback.didFinish();
    }

    public final StepState getState() {
        return this.state;
    }
}