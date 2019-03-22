package frc.robot.helpers;

public interface ILinearMotor extends IMotor {
    public double getLinearPosition();
    public double getRotationToDistanceRatio();
    public double getHomeLinearPosition();
    public double getMinLinearPosition();
    public double getMaxLinearPosition();
    public double getLinearVelocity();

    public void setRotationToDistanceRatio(double ratio);
    public void setTargetDisplacement(double displacement);
    public void setHomeLinearPosition(double position);
    public void setDisplacementLimits(double minDegrees, double maxDegrees);

    public double positionToLinear(int position);
    public int linearToPosition(double linearPosition);
}