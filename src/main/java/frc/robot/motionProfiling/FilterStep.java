package frc.robot.motionProfiling;

import java.util.ArrayList;

import frc.robot.motionProfiling.filters.DeltaPositionFilter;
import frc.robot.motionProfiling.filters.TrapezoidalFilter;
import frc.robot.sequencing.DynamicEndTime;
import frc.robot.sequencing.Step;

public class FilterStep<T> extends Step<T> {
    private ArrayList<FilterOutputModifier<T>> modifiers;
    private T initialFilterValue;
    public FilterStep(ArrayList<FilterOutputModifier<T>> modifiers, DynamicEndTime endTime) {
        super(endTime);
        this.modifiers = modifiers;
        this.initialFilterValue = null;
    }
    public FilterStep(FilterOutputModifier<T> modifier, DynamicEndTime endTime) {
        super(endTime);
        ArrayList<FilterOutputModifier<T>> filters = new ArrayList<>();
        filters.add(modifier);
        this.modifiers = filters;
        this.initialFilterValue = null;
    }

    public FilterStep(T initialFilterValue, ArrayList<FilterOutputModifier<T>> modifiers, DynamicEndTime endTime) {
        super(endTime);
        this.modifiers = modifiers;
        this.initialFilterValue = initialFilterValue;
    }
    public FilterStep(T initialFilterValue, FilterOutputModifier<T> modifier, DynamicEndTime endTime) {
        super(endTime);
        ArrayList<FilterOutputModifier<T>> filters = new ArrayList<>();
        filters.add(modifier);
        this.modifiers = filters;
        this.initialFilterValue = initialFilterValue;
    }

    public void addFilter(FilterOutputModifier<T> modifier) {
        this.modifiers.add(modifier);
    }

    @Override
    public T getUpdateForDeltaTime(double dt) {
         // calculate on the trapezoidal shape the current velocity
        T modifiedValue = initialFilterValue;
        for (FilterOutputModifier<T> modifier: modifiers) {
            modifiedValue = modifier.transform(dt, this.totalRunningTime.get(), modifiedValue);                
        }
        return modifiedValue;
    }

    public static FilterStep<MotionTriplet> trapezoidalProfileFilter(TrapezoidShape shape, double initialPosition) {
        ArrayList<FilterOutputModifier<MotionTriplet>> filters = new ArrayList<>();
        filters.add(new TrapezoidalFilter(shape));
        filters.add(new DeltaPositionFilter(initialPosition));
        return new FilterStep<>(filters, () -> shape.totalTime());
    }
}