package frc.robot.sequencing;

import edu.wpi.first.wpilibj.Timer;

public abstract class Step<T> {
    protected StepState state = StepState.IDLE;
    protected double startTime = 0;
    protected DynamicEndTime totalRunningTime;

    public Step(double runningTime) {
        this.totalRunningTime = () -> runningTime;
        
    }

    public Step(DynamicEndTime totalRunningTime) {
        this.totalRunningTime = totalRunningTime;
    }
    public Step() {
        this.totalRunningTime = () -> 0.0;
    }

    public abstract T update(double timeStamp);
    public abstract T getLastUpdate();
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
        }
        return this.state == StepState.FINISHED;
    }

    public final void endStep() {
        this.state = StepState.FINISHED;
    }

    public final StepState getState() {
        return this.state;
    }
}