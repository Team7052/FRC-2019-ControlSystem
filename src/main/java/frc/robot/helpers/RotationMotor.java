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

    public RotationMotor(int canID) {
        super(canID);
        this.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, this.slotIdx, this.timeoutMs);
    }

    public void initializeHome() {
        homeQuadraturePosition = this.getSensorCollection().getPulseWidthPosition();
        this.setSelectedSensorPosition(homeQuadraturePosition, slotIdx, timeoutMs);
    }


    public double getCurrentDegrees() {
        return (double) (this.getSelectedSensorPosition(slotIdx) - homeQuadraturePosition) / 4096.0 * 360.0;
    }

    public int getTarget() {
        return this.currentTargetQuadraturePositoin;
    }

    public void setDegrees(double degrees) {
        int targetPosition = (int) ((double) homeQuadraturePosition + degrees / 360 * 4096 * (positionInverted ? -1 : 1));
        this.currentTargetQuadraturePositoin = targetPosition;
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