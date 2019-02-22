package frc.robot.tests;

import frc.robot.tests.calibration.ArmCalibration;
import frc.robot.tests.calibration.CalibrationProcedure;

import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;

public class TestManager implements CalibrationProcedure.ProcedureDelegate{
    private static TestManager instance;

    public static final String kArmCalibrationName = "Arm Calibration";
    public static final String kDriveCalibrationName = "Drive Calibration";
    public static final String kLiftCalibrationName = "Lift Calibration";
    public static final String kShooterCalibrationName = "Shooter Calibration";
    public static final String kVisionCalibrationName = "Vision Calibration";
    public static final String kArmTestingName = "Arm Testing";
    public static final String kDriveTestingName = "Drive Testing";
    public static final String kLiftTestingName = "Lift Testing";
    public static final String kShooterTestingName = "Shooter Testing";
    public static final String kVisionTestingName = "Vision Testing";

    private Map<String, CalibrationProcedure> calibrationProcedures;
    private CalibrationProcedure currentCalibrationProcedure;
    public static TestManager getInstance() {
        if (instance == null) instance = new TestManager();
        return instance;
    }

    private TestManagerState state;

    private TestManager() {
        state = TestManagerState.IDLE;
        calibrationProcedures = new HashMap<>();
        // initialize all procedures
        calibrationProcedures.put(kArmCalibrationName, new ArmCalibration(kArmCalibrationName, this));
    }

    public String[] getCalibrationProcedureNames() {
        String listNames[] = new String[this.calibrationProcedures.size()];
        int iterator = 0;
        for (Map.Entry<String, CalibrationProcedure> entrySet: this.calibrationProcedures.entrySet()) {
            listNames[iterator++] = entrySet.getKey();
        }
        return listNames;
    }

    public void setState(TestManagerState state) {
        this.state = state;
    }

    public void setCalibrationProcedure(String name) {
        CalibrationProcedure selectedProcedure = this.calibrationProcedures.get(name);
        if (selectedProcedure == null) return;
        this.currentCalibrationProcedure = selectedProcedure;
        this.setState(TestManagerState.CALIBRATING);
    }

    public void removeCurrentCalibrationProcedure() {
        this.setCalibrationProcedure("");
    }

    public void update() {
        if (this.state == TestManagerState.IDLE) {
            return;
        }
        else if (this.state == TestManagerState.TESTING) {
        }
        else if (this.state == TestManagerState.CALIBRATING) {
            if (this.currentCalibrationProcedure != null) {
                this.currentCalibrationProcedure.update();
            }
        }
    }

    public void didFinish() {
        this.currentCalibrationProcedure = null;
        this.setState(TestManagerState.IDLE);
    }

    public TestManagerState getState() {
        return this.state;
    }
}