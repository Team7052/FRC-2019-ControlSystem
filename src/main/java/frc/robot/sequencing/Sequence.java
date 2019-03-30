package frc.robot.sequencing;

import java.util.ArrayList;

import frc.robot.util.Callback;

public class Sequence<T> extends Step<T> {
    private int currentStepIndex = 0;
    StepState state;

    private ArrayList<Step<T>> steps;

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
        double runningTotalTimeSum = 0.0;
        for (Step<T> step: this.steps) {
            if (step.totalRunningTime.get() >= dt - runningTotalTimeSum) {
                return step.getUpdateForDeltaTime(dt - runningTotalTimeSum);
            }
            runningTotalTimeSum += step.totalRunningTime.get();
        }
        return null;
    }

    public final boolean isEmpty() {
        return this.steps.size() == 0;
    }
/*
    @Override
    public T update(double timeStamp) {
        if (currentStepIndex >= steps.size()) return null;

        Step<T> currentStep = steps.get(currentStepIndex);
        if (currentStep.isFinished(timeStamp)) {
            this.currentStepIndex += 1;
            if (currentStepIndex >= steps.size()) return null;
            currentStep = steps.get(currentStepIndex);
        }
        if (!currentStep.hasBegan()) currentStep.start(currentStep.startTime + this.startTime);
        if (currentStep.isRunning()) {
            return currentStep.update(timeStamp);
        }
        
        return null;
    }
    */
}