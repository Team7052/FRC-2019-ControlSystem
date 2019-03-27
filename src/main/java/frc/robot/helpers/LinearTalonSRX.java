package frc.robot.helpers;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;


/* Linearly in meters 1 rotation = how many meters? */

public class LinearTalonSRX extends WPI_TalonSRX implements ILinearMotor {
    private double k_p, k_i, k_d;

    /* 1 revolution to x meters */
    private double conversionFactorRate;

    private double homeLinearPosition, minLinearPosition, maxLinearPosition;

    private boolean positionInverted = false;
    private boolean sensorPhase = false;

    private double currentTargetQuadraturePosition;

    public final int slotIdx = 0;
    public final int timeoutMs = 20;

    public LinearTalonSRX(int canID, double conversionRate) {
        super(canID);
        this.conversionFactorRate = conversionRate;
    }

    @Override
    public int getPosition() {
        return this.getSelectedSensorPosition(this.slotIdx);
    }

    @Override
    public double getLinearPosition() {
        return positionToLinear(this.getPosition());
    }

    @Override
    public double getRotationToDistanceRatio() {
        return this.conversionFactorRate;
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
    public double getHomeLinearPosition() {
        return this.homeLinearPosition;
    }

    @Override
    public double getMinLinearPosition() {
        return this.minLinearPosition;
    }

    @Override
    public double getMaxLinearPosition() {
        return this.maxLinearPosition;
    }

    @Override
    public int getRawVelocity() {
        return this.getSelectedSensorVelocity(this.slotIdx);
    }

    @Override
    public double getLinearVelocity() {
        return positionToLinear(this.getRawVelocity());
    }

    @Override
    public boolean getSensorPhase() {
        return this.sensorPhase;
    }

    @Override
    public boolean getInvertedPosition() {
        return this.positionInverted;
    }

    @Override
    public void setPosition(int position) {
        this.set(ControlMode.Position, position);
    }
    @Override
    public void setRotationToDistanceRatio(double ratio) {
        this.conversionFactorRate = ratio;
    }

    @Override
    public void setPercentOutput(double value) {
        this.set(ControlMode.PercentOutput, value);
    }

    @Override
    public void setTargetDisplacement(double displacement) {
        if (displacement < this.minLinearPosition) displacement = minLinearPosition;
        else if (displacement > this.maxLinearPosition) displacement = maxLinearPosition;
        int targetPosition = linearToPosition(displacement);
        this.set(ControlMode.Position, targetPosition);
    }

    @Override
    public void setHomeLinearPosition(double position) {
        this.homeLinearPosition = position;
        this.setSelectedSensorPosition(linearToPosition(this.homeLinearPosition), this.slotIdx, this.timeoutMs);
        System.out.println("Sensor position: " + this.getPosition());
    }

    @Override
    public void setDisplacementLimits(double minDisplacement, double maxDisplacement) {
        this.minLinearPosition = minDisplacement;
        this.maxLinearPosition = maxDisplacement;
    }

    @Override
    public void setInvertedPosition(boolean inverted) {
        this.positionInverted = inverted;
        this.setHomeLinearPosition(this.homeLinearPosition);
    }

    @Override
    public void setSensorPhase(boolean PhaseSensor) {
        super.setSensorPhase(PhaseSensor);
        this.sensorPhase = PhaseSensor;
        this.setHomeLinearPosition(this.homeLinearPosition);
    }

    @Override
    public void setIntegralValue(double value) {
        this.setIntegralAccumulator(value);
    }
    
    @Override
    public double positionToLinear(int position) {
        return (positionInverted ? -1 : 1) * (double) position / 4096.0 * this.conversionFactorRate;
    }

    @Override
    public int linearToPosition(double linearPosition) {
        return (positionInverted ? -1 : 1) * (int) (linearPosition / this.conversionFactorRate * 4096.0);
    }
    
    @Override
    public void set(ControlMode mode, double value) {
        int maxPosition = linearToPosition(this.positionInverted ? this.minLinearPosition : this.maxLinearPosition);
        int minPosition = linearToPosition(this.positionInverted ? this.maxLinearPosition : this.minLinearPosition);
        if (mode == ControlMode.Position) {
            this.currentTargetQuadraturePosition = (int) value;

            if (value > maxPosition) value = maxPosition;
            if (value < minPosition) value = minPosition;
        }
        else if (mode == ControlMode.PercentOutput) {
            if (this.getPosition() >= maxPosition) value = 0;
            if (this.getPosition() <= minPosition) value = 0;
        }
        super.set(mode, value);
    }

    @Override
    public double get_kp() {
        return this.k_p;
    }

    @Override
    public double get_ki() {
        return this.k_i;
    }

    @Override
    public double get_kd() {
        return this.k_d;
    }

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
}