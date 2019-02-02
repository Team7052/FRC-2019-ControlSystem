package frc.robot.helpers;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class RotationMotor extends WPI_TalonSRX {

    private double k_P = 0.0;
    private double k_I = 0.0;
    private double k_D = 0.0;

    public final int slotIdx = 0;
    public final int timeoutMs = 20;

    // expected home position
    int expectedHomePositionMin = 0;
    int expectedHomePositionMax = 4096;

    // quadrature positions - this is initialized when the robot first turns on
    int homeQuadraturePosition = 0;

    // degree positions all relative to ground (pointing straight downwards = 0˚)
    public double homeDegrees = 0;
    public double maxDegrees = -1;
    public double minDegrees = -1;

    int currentTargetQuadraturePositoin = 0;

    private boolean positioningError = false;
    public boolean positionInverted = false;

    public RotationMotor(int canID, double configHomeDegrees, double configMaxDegrees, double configMinDegrees, int configExpectedHomePositionMin, int configExpectedHomePositionMax, boolean positionInverted, boolean sensorPhase) {
        super(canID);
        this.setSensorPhase(sensorPhase);
        initialize(configHomeDegrees, configMaxDegrees, configMinDegrees, configExpectedHomePositionMin, configExpectedHomePositionMax, positionInverted);
    }

    public RotationMotor(int canID, double configHomeDegrees, int configExpectedHomePositionMin, int configExpectedHomePositionMax, boolean positionInverted) {
        super(canID);
        initialize(configHomeDegrees, -1, -1, configExpectedHomePositionMin, configExpectedHomePositionMax, positionInverted);
    }

    public void initialize(double configHomeDegrees, double configMaxDegrees, double configMinDegrees, int configExpectedHomePositionMin, int configExpectedHomePositionMax, boolean positionInverted) {
        this.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, this.slotIdx, this.timeoutMs);

        this.homeDegrees = configHomeDegrees;
        this.maxDegrees = configMaxDegrees;
        this.minDegrees = configMinDegrees;
        this.expectedHomePositionMax = configExpectedHomePositionMax;
        this.expectedHomePositionMin = configExpectedHomePositionMin;

        homeQuadraturePosition = this.getSensorCollection().getPulseWidthPosition();
        this.setSelectedSensorPosition(homeQuadraturePosition, slotIdx, timeoutMs);
        System.out.println("AFDFDSF: " + this.getSelectedSensorPosition(slotIdx));
        if (minDegrees != maxDegrees && minDegrees != -1) {
           /* this.configForwardSoftLimitEnable(true);
            this.configReverseSoftLimitEnable(true);
            this.configForwardSoftLimitThreshold(homeQuadraturePosition + (int) ((maxDegrees - homeDegrees) / 360.0 * 4096.0));
            this.configReverseSoftLimitThreshold(homeQuadraturePosition + (int) ((minDegrees - homeDegrees) / 360.0 * 4096.0));*/
        }
        
        if (homeQuadraturePosition < expectedHomePositionMin || homeQuadraturePosition > expectedHomePositionMax) {
            positioningError = true;
        }
    }

    public double getCurrentDegrees() {
        return (double) (this.getSelectedSensorPosition(slotIdx) - homeQuadraturePosition) / 4096.0 * 360.0;
    }

    @Override
    public void set(ControlMode mode, double value) {
        // don't allow the motor to be changed/set if it's home position doesn't match the actual position
        if (this.isPositioningError()) {
            System.out.println("Positioning Error! Home position is expected to be between " + this.expectedHomePositionMin + " and " + this.expectedHomePositionMax + " but the home position is " + this.homeQuadraturePosition);
            return;
        }
        super.set(mode, value);
    }
    int realElbowSpeed;

    public int getTarget() {
        return this.currentTargetQuadraturePositoin;
    }

    public void setDegrees(double degrees) {
        int targetPosition = (int) ((double) homeQuadraturePosition + degrees / 360 * 4096 * (positionInverted ? -1 : 1));
        this.currentTargetQuadraturePositoin = targetPosition;
        this.set(ControlMode.Position, targetPosition);
    }

    public boolean isPositioningError() {
        if (homeQuadraturePosition < expectedHomePositionMin || homeQuadraturePosition > expectedHomePositionMax) {
            positioningError = true;
        }
        else {
            positioningError = false;
        }
        return this.positioningError;
    }

    public double getk_P() { return this.k_P; }
    public double getk_I() { return this.k_I; }
    public double getk_D() { return this.k_D; }

    public void setk_P(double value) { 
        this.k_P = value;
        this.config_kP(this.slotIdx, value, this.timeoutMs);
    }
    public void setk_I(double value) { 
        this.k_I = value;
        this.config_kI(this.slotIdx, value, this.timeoutMs);
    }
    public void setk_D(double value) {
        this.k_D = value; 
        this.config_kD(this.slotIdx, value, this.timeoutMs);
    }
}