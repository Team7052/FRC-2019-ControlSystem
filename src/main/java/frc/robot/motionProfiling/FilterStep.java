package frc.robot.motionProfiling;

import java.util.ArrayList;

import frc.robot.motionProfiling.filters.DeltaPositionFilter;
import frc.robot.motionProfiling.filters.TrapezoidalFilter;
import frc.robot.sequencing.DynamicEndTime;
import frc.robot.sequencing.Step;

/**
 * A super flexible Step that calculates output based on a series of dynamically defined {@link frc.robot.motionProfiling.FilterOutputModifier #filters}
 * @param <T>
 */
public class FilterStep<T> extends Step<T> {
    /**
     * ArrayList that keeps track of all the filter modifiers
     */
    private ArrayList<FilterOutputModifier<T>> modifiers;
    /**
     * Th initial value of the filter to be executed, assumes that it is null.
     */
    private T initialFilterValue;
    
    /**
     * Initialize based on an array of specified filters and a dynamic runtime.
     * @param modifiers a list of all the filters to be used
     * @param endTime the total time of the Step
     */
    public FilterStep(ArrayList<FilterOutputModifier<T>> modifiers, DynamicEndTime endTime) {
        super(endTime);
        this.modifiers = modifiers;
        this.initialFilterValue = null;
    }
    /**
     * Initialize based on only a single {@link frc.robot.motionProfiling.FilterOutputModifier}
     * @param modifier a single filter 
     * @param endTime the total time of the Step
     */
    public FilterStep(FilterOutputModifier<T> modifier, DynamicEndTime endTime) {
        super(endTime);
        ArrayList<FilterOutputModifier<T>> filters = new ArrayList<>();
        filters.add(modifier);
        this.modifiers = filters;
        this.initialFilterValue = null;
    }

    /**
     * Initialize based on an array of specified filters and a dynamic runtime.
     * @param initialFilterValue The value the filter should always begin calculating with
     * @param modifier a single filter 
     * @param endTime the total time of the Step
     */
    public FilterStep(T initialFilterValue, ArrayList<FilterOutputModifier<T>> modifiers, DynamicEndTime endTime) {
        super(endTime);
        this.modifiers = modifiers;
        this.initialFilterValue = initialFilterValue;
    }
    /**
     * Initialize based on only a single {@link frc.robot.motionProfiling.FilterOutputModifier}
     * @param initialFilterValue The value the filter should always begin calculating with
     * @param modifier a single filter 
     * @param endTime the total time of the Step
     */
    public FilterStep(T initialFilterValue, FilterOutputModifier<T> modifier, DynamicEndTime endTime) {
        super(endTime);
        ArrayList<FilterOutputModifier<T>> filters = new ArrayList<>();
        filters.add(modifier);
        this.modifiers = filters;
        this.initialFilterValue = initialFilterValue;
    }

    /**
     * Add a {@link frc.robot.motionProfiling.FilterOutputModifier}
     * @param modifier
     */
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

    /**
     * FilterStep that has two filters already built in: {@link frc.robot.motionProfiling.filters.DeltaPositionFilter}
     * and {@link frc.robot.motionProfiling.filters.TrapezoidalFilter} in order to create a trapezoidal motion profiling (position) filter with an initial position.
     * @param shape {@link TrapezoidShape} of the velocity profile
     * @param initialPosition initial position of the filter
     * @return
     */
    public static FilterStep<MotionTriplet> trapezoidalProfileFilter(TrapezoidShape shape, double initialPosition) {
        ArrayList<FilterOutputModifier<MotionTriplet>> filters = new ArrayList<>();
        filters.add(new TrapezoidalFilter(shape));
        filters.add(new DeltaPositionFilter(initialPosition));
        return new FilterStep<>(filters, () -> shape.totalTime());
    }
}