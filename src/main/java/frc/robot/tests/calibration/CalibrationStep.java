package frc.robot.tests.calibration;

import java.util.HashMap;
import java.util.Map;;

public class CalibrationStep {

    public interface CalibrationMethods {
        public boolean run(CalibrationStep step);
        public void onFinish(CalibrationStep step);
    }

    // define goals
    String goal;

    // detailied instructions
    String instructions;

    // define constraints
    CalibrationMethods method;

    public Map<String, Object> data;

    private boolean canFinish = false;

    public CalibrationStep(String goal, String instructions, CalibrationMethods method) {
        this.instructions = instructions;
        this.goal = goal;
        this.method = method;
        this.data = new HashMap<>();
    }

    // returns if finished running
    public boolean run() {
        return method.run(this);
    }

    public boolean getFinishable() {
        return this.canFinish;
    }
    public void setFinishable(boolean finish) {
        this.canFinish = finish;
    }
    
}