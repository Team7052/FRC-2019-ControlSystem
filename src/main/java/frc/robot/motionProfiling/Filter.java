package frc.robot.motionProfiling;

public interface Filter<T> {
    public T calculate(double percentage);
}