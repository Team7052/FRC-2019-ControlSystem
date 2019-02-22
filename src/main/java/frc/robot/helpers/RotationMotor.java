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
    private boolean sensorPhase = false;
    public boolean positionInverted = true;

    // quadrature positions - this is initialized when the robot first turns on
    public int homeQuadraturePosition = 0;

    // degree positions all relative to ground (pointing straight downwards = 0˚)
    public double homeDegrees = 0;
    public double maxDegrees = -1;
    public double minDegrees = -1;

    public double gearRatio = 1.0;

    int currentTargetQuadraturePosition = 0;
    public RotationMotor(int canID) {
        super(canID);
        gearRatio = 1.0;
        this.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, this.slotIdx, this.timeoutMs);
    }

    public RotationMotor(int canID, double gearRatio) {
        super(canID);
        this.gearRatio = gearRatio;
        this.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, this.slotIdx, this.timeoutMs);
    }

    public void initializeHome(double homeDegrees) {
        this.homeQuadraturePosition = (positionInverted ? -1 : 1) * (int) ((homeDegrees / 360.0) * 4096 * gearRatio);
        this.setSelectedSensorPosition(this.homeQuadraturePosition, slotIdx, timeoutMs);
    }

    public double getSpeed() {
        return this.getMotorOutputPercent();
    }

    public void setSpeed(double speed) {
        this.set(ControlMode.PercentOutput, speed);
    }

    public double getCurrent() {
        return this.getOutputCurrent();
    }

    public int getPosition() {
        return this.getSelectedSensorPosition(this.slotIdx);
    }

    public double getCurrentDegrees() {
        return ((double) this.getSelectedSensorPosition(this.slotIdx) / (4096.0 * gearRatio)) * 360.0;
    }

    public double getVelocity() {
        return this.getSelectedSensorVelocity(this.slotIdx);
    }

    public int getTarget() {
        return this.currentTargetQuadraturePosition;
    }

    public boolean getInvertedPosition() {
        return this.getInverted();
    }
    public void setInvertedPosition(boolean inverted) {
        this.positionInverted = inverted;
        if (Math.abs(this.homeQuadraturePosition - this.getPosition()) < 100) {
            this.initializeHome(this.homeDegrees);
        }
    }

    @Override
    public void setSensorPhase(boolean PhaseSensor) {
        super.setSensorPhase(PhaseSensor);
        this.sensorPhase = PhaseSensor;
    }

    public boolean getPhase() {
        return this.sensorPhase;
    }

    public void setDegrees(double degrees) {
        int targetPosition = (positionInverted ? -1 : 1) * (int) ((degrees / 360.0) * 4096 * gearRatio);
        this.currentTargetQuadraturePosition = targetPosition;
        System.out.println("current: " + this.getPosition() + ", target: " + targetPosition);
        this.set(ControlMode.Position, targetPosition);
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