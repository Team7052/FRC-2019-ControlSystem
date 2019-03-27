package frc.robot.motionProfiling;

public class MotionFilters {
    public static FilterOutputModifier<MotionTriplet> trapezoidFilter(TrapezoidShape shape) {
        return (dt, endTime, currentTriplet) -> {
            double velocity = shape.getVelocityForTime(dt);
            double position = shape.getIntegralForTime(dt);
            double acceleration = shape.getDerivativeForTime(dt);
            currentTriplet = new MotionTriplet(position + currentTriplet.getPosition(), velocity, acceleration);
            return currentTriplet;
        };
    }

    public static FilterOutputModifier<MotionTriplet> addInitialPositionFilter(double initialPosition) {
        return (dt, endTime, currentTriplet) -> {
            currentTriplet.a += initialPosition;
            return currentTriplet;
        };
    }
}