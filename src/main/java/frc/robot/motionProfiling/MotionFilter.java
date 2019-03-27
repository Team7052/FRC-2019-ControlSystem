package frc.robot.motionProfiling;

import java.util.ArrayList;

import frc.robot.sequencing.DynamicEndTime;
import frc.robot.sequencing.Step;

public class MotionFilter extends Step<MotionTriplet> {
    ArrayList<FilterOutputModifier<MotionTriplet>> modifiers;
    public MotionFilter(ArrayList<FilterOutputModifier<MotionTriplet>> modifiers, DynamicEndTime endTime) {
        super(endTime);
        this.modifiers = modifiers;
    }
    public MotionFilter(FilterOutputModifier<MotionTriplet> modifier, DynamicEndTime endTime) {
        super(endTime);
        ArrayList<FilterOutputModifier<MotionTriplet>> filters = new ArrayList<>();
        filters.add(modifier);
        this.modifiers = filters;
    }

    public void addFilter(FilterOutputModifier<MotionTriplet> modifier) {
        this.modifiers.add(modifier);
    }

    @Override
    public MotionTriplet getUpdateForDeltaTime(double dt) {
         // calculate on the trapezoidal shape the current velocity
        MotionTriplet modifiedTriplet = new MotionTriplet(0.0,0.0,0.0);
        for (FilterOutputModifier<MotionTriplet> modifier: modifiers) {
            modifiedTriplet = modifier.transform(dt, this.totalRunningTime.get(), modifiedTriplet);                
        }
        return modifiedTriplet;
    }

    public static MotionFilter trapezoidalProfileFilter(TrapezoidShape shape, double initialPosition) {
        ArrayList<FilterOutputModifier<MotionTriplet>> filters = new ArrayList<>();
        filters.add(MotionFilters.trapezoidFilter(shape));
        filters.add(MotionFilters.addInitialPositionFilter(initialPosition));
        return new MotionFilter(filters, () -> shape.totalTime());
    }
}