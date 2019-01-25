package frc.robot.motionProfiling;

// contains the function ranges, lower, upper bounds, and spacing between
public class FunctionSet {
    private PureFunction function;
    private double lower, upper, delta;
    FunctionSet next, prev;

    public FunctionSet(PureFunction function, double lower, double upper, double delta) {
        this.function = function;
        this.lower = lower;
        this.upper = upper;
        this.delta = delta;
    }

    public FunctionSet(PureFunction function) {
        this.function = function;
        this.lower = 0;
        this.upper = 100;
        this.delta = 1;
    }

    public static FunctionSet create(PureFunction function, double lower, double upper, double delta) {
        return new FunctionSet(function, lower, upper, delta);
    }

    public double getDelta() {
        return delta;
    }

    public double getLower() {
        return lower;
    }

    public double getUpper() {
        return upper;
    }

    public PureFunction getFunction() {
        return function;
    }
}
