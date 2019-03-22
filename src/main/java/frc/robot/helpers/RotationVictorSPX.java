package frc.robot.helpers;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;;


public class RotationVictorSPX extends WPI_VictorSPX implements IRotationMotor {
    private double k_p = 0.0, k_i = 0.0, k_d = 0.0;
    public final int slotIdx = 0;
    public final int timeoutMs = 20;

    private double gearRatio;

    private boolean sensorPhase = false;
    private boolean positionInverted = false;

    private double homeDegrees, minDegrees, maxDegrees;

    private PIDPositionController pidController;
    private CustomEncoder encoder;
    private int ticks;

    public RotationVictorSPX(int canID, int encoderA, int encoderB) {
        super(canID);
        this.init(canID, encoderA, encoderB, 1.0, 1024);
    }

    public RotationVictorSPX(int canID, int encoderA, int encoderB, double gearRatio, int ticks) {
        super(canID);
        this.init(canID, encoderA, encoderB, gearRatio, ticks);
    }

    private void init(int canID, int encoderA, int encoderB, double gearRatio, int ticks) {
        this.encoder = new CustomEncoder(encoderA, encoderB, false, EncodingType.k1X);
        pidController = new PIDPositionController(this.k_p, this.k_i, this.k_d);
        this.gearRatio = gearRatio;
        this.ticks = ticks;
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
        return this.encoder.get();
    }

    @Override
    public double getDegrees() {
        return positionToDegrees(this.getPosition());
    }

    @Override
    public double getHomeDegrees() {
        return this.homeDegrees;
    }

    @Override
    public int getRawVelocity() {
        return (int) this.encoder.getRate();
    }

    @Override
    public double getVelocityDegrees() {
        return this.encoder.getRate() / (double) ticks * 360.0;
    }

    @Override
    public int getTargetPosition() {
        return (int) pidController.getSetpoint();
    }

    @Override
    public double getTargetDegrees() {
        return this.positionToDegrees(this.getTargetPosition());
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
        System.out.println("Can't set position on VictorSPX");
    }

    @Override
    public void homeAbsolutePosition(double degrees) {
        
    }
    @Override
    public void initializeHomeDegreesFromPWM() {
        
    }
    @Override
    public void setHomeDegrees(double degrees) {
        this.homeDegrees = degrees;
        int homeQuadraturePosition = this.degreesToPosition(degrees);
        this.encoder.setSelectedSensorPosition(homeQuadraturePosition);
    }

    @Override
    public void setDegreesLimits(double minDegrees, double maxDegrees) {
        this.minDegrees = minDegrees;
        this.maxDegrees = maxDegrees;
    }

    @Override
    public void setDegrees(double degrees) {
        int targetPosition = degreesToPosition(degrees);
        this.pidController.setSetpoint(targetPosition);
        double value = (this.sensorPhase ? -1 : 1) * this.pidController.calculatePIDOutput(this.encoder.get());

        this.set(ControlMode.PercentOutput, value);
    }

    @Override
    public void setInvertedPosition(boolean inverted) {
        this.positionInverted = inverted;
        this.setHomeDegrees(this.homeDegrees);
    }
    @Override
    public void setSensorPhase(boolean phaseSensor) {
        super.setSensorPhase(phaseSensor);
        this.sensorPhase = phaseSensor;
    }

    @Override
    public void setIntegralValue(double value) {
        this.pidController.setIntegralAccumulator(value);
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
        this.pidController.setGains(k_p, k_i, k_d);
    }
    @Override
    public void set_ki(double value) { 
        this.k_i = value;
        this.pidController.setGains(k_p, k_i, k_d);
    }
    @Override
    public void set_kd(double value) {
        this.k_d = value; 
        this.pidController.setGains(k_p, k_i, k_d);
    }

    @Override
    public int degreesToPosition(double degrees) {
        return (positionInverted ? -1 : 1) * (int) ((degrees / 360.0) * (double) ticks * this.gearRatio);
    }

    @Override
    public double positionToDegrees(int position) {
        return (positionInverted ? -1 : 1) * ((double) position / (double) ticks / this.gearRatio) * 360.0;
    }

    /* Override set */
    @Override
    public void set(ControlMode mode, double value) {
        int maxPosition = degreesToPosition(this.positionInverted ? this.minDegrees : this.maxDegrees);
        int minPosition = degreesToPosition(this.positionInverted ? this.maxDegrees : this.minDegrees);
        if (mode == ControlMode.Position) {
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