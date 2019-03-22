package frc.robot.helpers;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class RotationTalonSRX extends WPI_TalonSRX implements IRotationMotor {
    private double k_p = 0.0, k_i = 0.0, k_d = 0.0;
    public final int slotIdx = 0;
    public final int timeoutMs = 20;

    public double gearRatio = 1.0;

    private boolean positionInverted = false;
    private boolean sensorPhase = false;

    private double homeDegrees, minDegrees, maxDegrees;

    private int currentTargetQuadraturePosition = 0;

    public RotationTalonSRX(int canID) {
        super(canID);
        gearRatio = 1.0;
        this.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, this.slotIdx, this.timeoutMs);
    }

    public RotationTalonSRX(int canID, double gearRatio) {
        super(canID);
        this.gearRatio = gearRatio;
        this.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, this.slotIdx, this.timeoutMs);
    }

    @Override
    public double getCurrent() {
        return this.getOutputCurrent();
    }

    @Override
    public double getPercentOutput() {
        return this.getMotorOutputPercent();
    }

    @Override
    public double getVoltage() {
        return this.getMotorOutputVoltage();
    }

    @Override
    public int getPosition() {
        return this.getSelectedSensorPosition(this.slotIdx);
    }

    @Override
    public double getDegrees() {
        return positionToDegrees(this.getSelectedSensorPosition(this.slotIdx));
    }
    
    @Override
    public double getHomeDegrees() {
        return this.homeDegrees;
    }

    @Override
    public int getRawVelocity() {
        return this.getSelectedSensorVelocity(this.slotIdx);
    }

    @Override
    public double getVelocityDegrees() {
        return this.getSelectedSensorVelocity(this.slotIdx) / (4096 * this.gearRatio) * 360.0;
    }

    @Override
    public int getTargetPosition() {
        return this.currentTargetQuadraturePosition;
    }
    @Override
    public double getTargetDegrees() {
        return this.positionToDegrees(this.currentTargetQuadraturePosition);
    }

    @Override
    public boolean getInvertedPosition() {
        return this.positionInverted;
    }

    @Override
    public boolean getSensorPhase() {
        return this.sensorPhase;
    }

    /* Setters */
    @Override
    public void setPercentOutput(double speed) {
        this.set(ControlMode.PercentOutput, speed);
    }
    @Override
    public void setPosition(int position) {
        this.set(ControlMode.Position, position);
    }
    // if no degrees is specified, then set home degrees to PWM
    @Override
    public void initializeHomeDegreesFromPWM() {
        //int position = this.getSensorCollection().getPulseWidthPosition();
        //this.setSelectedSensorPosition(position);
        //this.homeDegrees = positionToDegrees(position);
    }
    @Override
    public void setHomeDegrees(double degrees) {
        this.homeDegrees = degrees;
        int homeQuadraturePosition = this.degreesToPosition(degrees);
        this.setSelectedSensorPosition(homeQuadraturePosition, slotIdx, timeoutMs);
    }
    @Override
    public void homeAbsolutePosition(double degrees) {
       // this.getSensorCollection().setPulseWidthPosition(this.degreesToPosition(degrees), this.timeoutMs);
       // this.setSelectedSensorPosition(this.degreesToPosition(degrees));
    }
    
    @Override
    public void setDegreesLimits(double minDegrees, double maxDegrees) {
        this.minDegrees = minDegrees;
        this.maxDegrees = maxDegrees;
    }
    @Override
    public void setDegrees(double degrees) {
        if (degrees < this.minDegrees) degrees = this.minDegrees;
        if (degrees > this.maxDegrees) degrees = this.maxDegrees;
        int targetPosition = degreesToPosition(degrees);
        this.set(ControlMode.Position, targetPosition);
    }
    @Override
    public void setInvertedPosition(boolean inverted) {
        this.positionInverted = inverted;
        this.setHomeDegrees(this.homeDegrees);
    }

    @Override
    public void setSensorPhase(boolean PhaseSensor) {
        super.setSensorPhase(PhaseSensor);
        this.sensorPhase = PhaseSensor;
    }

    @Override
    public void setIntegralValue(double value) {
        this.setIntegralAccumulator(value);
    }

    @Override
    public double get_kp() { return this.k_p; }
    @Override
    public double get_ki() { return this.k_i; }
    @Override
    public double get_kd() { return this.k_d; }


    @Override
    public void set_kp(double value) { 
        this.k_p = value;
        this.config_kP(this.slotIdx, value, this.timeoutMs);
    }
    @Override
    public void set_ki(double value) { 
        this.k_i = value;
        this.config_kI(this.slotIdx, value, this.timeoutMs);
    }
    @Override
    public void set_kd(double value) {
        this.k_d = value; 
        this.config_kD(this.slotIdx, value, this.timeoutMs);
    }

    @Override
    public int degreesToPosition(double degrees) {
        return (positionInverted ? -1 : 1) * (int) ((degrees / 360.0) * 4096.0 * gearRatio);
    }
    @Override
    public double positionToDegrees(int position) {
        return (positionInverted ? -1 : 1) * ((double) position / (4096.0 * gearRatio)) * 360.0;
    }

    /* Override Talon*/
    @Override
    public void set(ControlMode mode, double value) {
        int maxPosition = degreesToPosition(this.positionInverted ? this.minDegrees : this.maxDegrees);
        int minPosition = degreesToPosition(this.positionInverted ? this.maxDegrees : this.minDegrees);
        if (mode == ControlMode.Position) {
            this.currentTargetQuadraturePosition = (int) value;
            
            if (value > maxPosition) value = maxPosition;
            if (value < minPosition) value = minPosition;
        }
        else if (mode == ControlMode.PercentOutput) {
            if (this.getPosition() >= maxPosition) value = 0.0;
            if (this.getPosition() <= minPosition) value = 0.0;   
        }
        
        super.set(mode, value);
    }
}