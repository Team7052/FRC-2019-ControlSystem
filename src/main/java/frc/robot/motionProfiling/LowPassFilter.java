package frc.robot.motionProfiling;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.Timer;

public class LowPassFilter {
    private ArrayList<Point> velocityFunction = new ArrayList<>();
    private ArrayList<Point> accelerationFunction = new ArrayList<>();
    private ArrayList<Point> positionFunction = new ArrayList<>();

    private double setPointDegrees;
    private double initialDegrees;
    private final double tau = 30;

    private MotionProfileState state = MotionProfileState.IDLE;

    public LowPassFilter(double targetDegrees, double initialDegrees) {
        this.setPointDegrees = targetDegrees;
        this.initialDegrees = initialDegrees;
    }
    public double startTime = 0;
    public double prevDegrees = -500;
    public double updateFilter() {
        if (state == MotionProfileState.RUNNING) {
            double currentTime = Timer.getFPGATimestamp();
            double deltaTime = currentTime - startTime;
            if (prevDegrees == -500) prevDegrees = initialDegrees;
            double target = tau / (deltaTime + tau) * prevDegrees + deltaTime / (deltaTime + tau) * this.setPointDegrees;
            
            this.prevDegrees = target;
            return target;
        }
        return 0;
    }

    // setters and getters for motion profile

    public MotionProfileState getState() {
        return this.state;
    }

    public void stopFilter() {
        this.state = MotionProfileState.FINISHED;
    }

    public void startFilter() {
        if (this.state == MotionProfileState.IDLE) {
            this.state = MotionProfileState.RUNNING;
            this.startTime = Timer.getFPGATimestamp();
        }
    }
}