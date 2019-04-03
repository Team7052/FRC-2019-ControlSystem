package frc.robot.motionProfiling.filters;

import frc.robot.motionProfiling.FilterOutputModifier;
import frc.robot.motionProfiling.MotionTriplet;

public class LowPassFilter implements FilterOutputModifier<MotionTriplet> {

    private double targetDegrees;
    private final double tau = 30;

    public LowPassFilter(double targetDegrees) {
        this.targetDegrees = targetDegrees;
    }
    public double startTime = 0;

    @Override
    public MotionTriplet transform(double dt, double endTime, MotionTriplet currentValue) {
        double target = tau / (dt + tau) * currentValue.getPosition() + dt / (dt + tau) * this.targetDegrees;
        return new MotionTriplet(target + currentValue.getPosition(), currentValue.getVelocity(), currentValue.getAcceleration());
    }
}