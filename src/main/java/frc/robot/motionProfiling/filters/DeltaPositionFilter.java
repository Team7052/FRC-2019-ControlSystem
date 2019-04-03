package frc.robot.motionProfiling.filters;

import frc.robot.motionProfiling.FilterOutputModifier;
import frc.robot.motionProfiling.MotionTriplet;

public class DeltaPositionFilter implements FilterOutputModifier<MotionTriplet> {
    private double initialPosition;
    public DeltaPositionFilter(double initialPosition) {
        this.initialPosition = initialPosition;
    }

    public double getInitialPosition() {
        return initialPosition;
    }

    @Override
    public MotionTriplet transform(double dt, double endTime, MotionTriplet currentValue) {
        MotionTriplet newValue = new MotionTriplet(currentValue.getPosition() + initialPosition, currentValue.getVelocity(), currentValue.getAcceleration());
        return newValue;
    }
}