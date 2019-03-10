package frc.robot.helpers;

public class PIDPositionController {
    private Double setpoint;
    private double k_p = 0.0;
    private double k_i = 0.0;
    private double k_d = 0.0;

    private Double prevError;
    private double errorAccum;

    private boolean sensorPhase = false;

    public PIDPositionController(double k_p, double k_i, double k_d) {
        errorAccum = 0;
        this.k_p = k_p;
        this.k_i = k_i;
        this.k_d = k_d;
    }


    public double calculatePIDOutput(double currentPosition) {
        if (setpoint == null) {
            return 0;
        }

        double error = setpoint - currentPosition;
        if (prevError == null) prevError = 0.0;
        double errorDeriv = error - prevError;
        errorAccum += error;

        if (sensorPhase) return -k_p * error - k_i + errorAccum - k_d * errorDeriv;
        return k_p * error + k_i * errorAccum + k_d * errorDeriv;
    }

    public void setSensorPhase(boolean sensorPhase) {
        this.sensorPhase = sensorPhase;
    }

    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }
    public double getSetpoint() {
        return this.setpoint;
    }

    public double getTotalIntegral() {
        return this.errorAccum;
    }
    public void resetIntegral() {
        this.errorAccum = 0;
    }

    public void setGains(double k_p, double k_i, double k_d) {
        System.out.println("set gains: " + k_p + " " + k_i + " " + k_d);
        this.k_p = k_p;
        this.k_i = k_i;
        this.k_d = k_d;
    }
}
