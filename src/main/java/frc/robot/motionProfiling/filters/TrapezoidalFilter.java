package frc.robot.motionProfiling.filters;

import frc.robot.motionProfiling.FilterOutputModifier;
import frc.robot.motionProfiling.MotionTriplet;
import frc.robot.motionProfiling.TrapezoidShape;

public class TrapezoidalFilter implements FilterOutputModifier<MotionTriplet> {
    private TrapezoidShape shape;
    public TrapezoidalFilter(TrapezoidShape shape) {
        this.shape = shape;
    }
    @Override
    public MotionTriplet transform(double dt, double endTime, MotionTriplet currentValue) {
        double velocity = shape.getVelocityForTime(dt);
        double position = shape.getIntegralForTime(dt);
        double acceleration = shape.getDerivativeForTime(dt);
        if (currentValue != null) return new MotionTriplet(position + currentValue.getPosition(), velocity, acceleration);
        return new MotionTriplet(position, velocity, acceleration);
    }
}