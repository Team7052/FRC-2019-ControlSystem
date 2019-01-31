package frc.robot.helpers;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class ArmMotor extends WPI_TalonSRX {

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

    // degree positions
    public double homeDegrees = 0;
    public double maxDegrees = -1;
    public double minDegrees = -1;

    public ArmMotor(int canID, double configHomeDegrees, double configMaxDegrees, double configMinDegrees, int configExpectedHomePositionMin, int configExpectedHomePositionMax) {
        super(canID);
        this.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, this.slotIdx, this.timeoutMs);

        this.homeDegrees = configHomeDegrees;
        this.maxDegrees = configMaxDegrees;
        this.minDegrees = configMinDegrees;
        this.expectedHomePositionMax = configExpectedHomePositionMax;
        this.expectedHomePositionMin = configExpectedHomePositionMin;

        homeQuadraturePosition = this.getSensorCollection().getPulseWidthPosition();
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