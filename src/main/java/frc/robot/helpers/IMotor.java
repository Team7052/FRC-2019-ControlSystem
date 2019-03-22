package frc.robot.helpers;

public interface IMotor {
    public double getCurrent();
    public double getPercentOutput();
    public double getVoltage();
    public int getRawVelocity();
    public int getPosition();
    public boolean getInverted();
    public boolean getInvertedPosition();
    public boolean getSensorPhase();

    public void setPosition(int position);
    public void setPercentOutput(double speed);
    public void setInvertedPosition(boolean inverted);
    public void setIntegralValue(double value);

    public double get_kp();
    public double get_ki();
    public double get_kd();

    public void set_kp(double value);
    public void set_ki(double value);
    public void set_kd(double value);
}