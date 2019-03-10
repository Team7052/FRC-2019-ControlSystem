package frc.robot.helpers;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.Encoder;;


public class WristMotor extends WPI_VictorSPX {
    private double k_P = 0.0;
    private double k_I = 0.0;
    private double k_D = 0.0;

    public final int slotIdx = 0;
    public final int timeoutMs = 20;

    PIDPositionController pidController;

    Encoder encoder;

    // expected home position
    private boolean sensorPhase = false;
    public boolean positionInverted = true;

    // quadrature positions - this is initialized when the robot first turns on
    public int homeQuadraturePosition = 0;

    // degree positions all relative to ground (pointing straight downwards = 0˚)
    public double homeDegrees = 0;
    public double maxDegrees = -1;
    public double minDegrees = -1;

    public WristMotor(int canID, int encoderA, int encoderB) {
        super(canID);
        this.encoder = new Encoder(encoderA, encoderB);
        pidController = new PIDPositionController(this.k_P, this.k_I, this.k_D);
        this.homeDegrees = 180;
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
        return this.encoder.get();
    }

    public double getCurrentDegrees() {
        return (positionInverted ? -1 : 1) * ((double) this.encoder.get() / 1024.0) * 360.0 + this.homeDegrees;
    }

    public double getRawVelocity() {
        return this.encoder.getRate();
    }

    public double getVelocityDegrees() {
        return this.encoder.getRate() / 1024.0 * 360.0;
    }

    public boolean getInvertedPosition() {
        return this.getInverted();
    }

    public void setInvertedPosition(boolean inverted) {
        this.positionInverted = inverted;
    }

    @Override
    public void setSensorPhase(boolean PhaseSensor) {
        super.setSensorPhase(PhaseSensor);
        this.pidController.setSensorPhase(this.sensorPhase);
        this.sensorPhase = PhaseSensor;
    }

    public boolean getPhase() {
        return this.sensorPhase;
    }

    public void setDegrees(double degrees) {
        int targetPosition = (positionInverted ? -1 : 1) * (int) (((degrees - this.homeDegrees) / 360.0) * 1024);
        this.pidController.setSetpoint(targetPosition);
        double value = this.pidController.calculatePIDOutput(this.encoder.get());
        this.set(ControlMode.PercentOutput, value);
    }

    public double getk_P() { return this.k_P; }
    public double getk_I() { return this.k_I; }
    public double getk_D() { return this.k_D; }

    public void setk_P(double value) {
        this.k_P = value;
        this.pidController.setGains(this.k_P, this.k_I, this.k_D);
    }
    public void setk_I(double value) { 
        this.k_I = value;
        this.pidController.setGains(this.k_P, this.k_I, this.k_D);
    }
    public void setk_D(double value) {
        this.k_D = value;
        this.pidController.setGains(this.k_P, this.k_I, this.k_D);
    }
}