package frc.robot.motionProfiling;

public interface FilterOutputModifier<T> {
    public T transform(double dt, double endTime, T currentValue);
}