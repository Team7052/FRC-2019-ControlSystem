package frc.robot.helpers;

public interface IRotationMotor extends IMotor {
    public double getVelocityDegrees();
    public int getTargetPosition();
    public double getTargetDegrees();
    public double getDegrees();
    public double getHomeDegrees();

    public void setDegrees(double degrees);
    public void initializeHomeDegreesFromPWM();
    public void setHomeDegrees(double degrees);
    public void setDegreesLimits(double minDegrees, double maxDegrees);
    public void homeAbsolutePosition(double degrees);

    public int degreesToPosition(double degrees);
    public double positionToDegrees(int position);
}