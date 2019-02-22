package frc.robot.tests.calibration;

import java.util.ArrayList;

import frc.robot.tests.calibration.CalibrationStep.CalibrationMethods;


public class CalibrationProcedure {
    public interface ProcedureDelegate {
        void didFinish();
    }

    public ProcedureDelegate delegate;
    private int currentStep;
    private String name;
    private ArrayList<CalibrationStep> steps;

    public CalibrationProcedure(String name, ProcedureDelegate delegate) {
        this.name = name;
        this.currentStep = 0;
        this.delegate = delegate;
    }

    // return itself to chain addStep functions
    public CalibrationProcedure addStep(String goal, String instructions, CalibrationMethods methods) {
        this.steps.add(new CalibrationStep(goal, instructions, methods));
        return this;
    }

    public void update() {
        if (this.currentStep >= this.steps.size()) {
            delegate.didFinish();
        }
        boolean didFinish = this.steps.get(this.currentStep).run();
        if (didFinish) this.currentStep += 1;
    }

    public void nextStep() {

    }

    public String getName() {
        return name;
    }


    public ArrayList<CalibrationStep> getSteps() {
        return steps;
    }
}