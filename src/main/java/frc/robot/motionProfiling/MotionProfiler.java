package frc.robot.motionProfiling;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.Timer;

public class MotionProfiler {
    private ArrayList<Point> velocityFunction = new ArrayList<>();
    private ArrayList<Point> accelerationFunction = new ArrayList<>();
    private ArrayList<Point> positionFunction = new ArrayList<>();

    private Point endPositionPoint;

    private MotionProfileState state = MotionProfileState.IDLE;

    public MotionProfiler() {
        
    }
    public double startTime = 0;
    public double index = 0;
    public MotionTriplet updateMotionProfile(double totalRunningTime) {
        if (state == MotionProfileState.RUNNING) {
            double currentTime = Timer.getFPGATimestamp();
            
            double percentage = (currentTime - startTime) / totalRunningTime;
            if (percentage > 1.0) {
                this.stopMotionProfile();
                return null;
            }
            
            int index = (int) Math.round(percentage * ((double) velocityFunction.size() - 1));
            
            // get velocity
            double velocityMeasurement = velocityFunction.get(index).y;
            double accelerationMeasurement = accelerationFunction.get(index).y;
            double positionMeasurement = positionFunction.get(index).y;

            return new MotionTriplet(velocityMeasurement, positionMeasurement, accelerationMeasurement);
        }
        else {
            return null;
        }
    }

    // setters and getters for motion profile

    public MotionProfileState getState() {
        return this.state;
    }

    public void stopMotionProfile() {
        this.state = MotionProfileState.FINISHED;
    }

    public void startMotionProfile() {
        if (this.state == MotionProfileState.IDLE) {
            this.state = MotionProfileState.RUNNING;
            this.startTime = Timer.getFPGATimestamp();
        }
    }

    public ArrayList<Point> getLinearInterpolation(ArrayList<Point> points, double delta) {
        ArrayList<Point> interpolatedPoints = new ArrayList<>();
        if (points.size() < 2) {
            return points;
        }
        for (int i = 0, n = points.size() - 1; i < n; i++) {
            double deltaX = points.get(i+1).x - points.get(i).x;
            interpolatedPoints.add(points.get(i));
            double slope = (points.get(i+1).y - points.get(i).y) / (points.get(i+1).x - points.get(i).x);
            double b = points.get(i).y - slope * points.get(i).x;

            interpolatedPoints.add(points.get(i));
            for (double j = delta; j < deltaX; j += delta) {
                double x = j + points.get(i).x;
                interpolatedPoints.add(new Point(x, slope * x + b));
            }
        }
        interpolatedPoints.add(points.get(points.size() - 1));
        return interpolatedPoints;
    }

    public ArrayList<Point> getCubicInterpolation(ArrayList<Point> points, double delta){
        
    }

    
    
    // setters and getters for functions
    public void setVelocityPoints(ArrayList<Point> points) {
        velocityFunction = points;
        accelerationFunction = FunctionGenerator.getDerivative(points);
        positionFunction = FunctionGenerator.getIntegral(points);

        if (positionFunction.size() > 0) {
            endPositionPoint = positionFunction.get(positionFunction.size() - 1);
        }
    }
    public void setPositionPoints(ArrayList<Point> points) {
        positionFunction = points;
        velocityFunction = FunctionGenerator.getDerivative(points);
        accelerationFunction = FunctionGenerator.getDerivative(velocityFunction);

        if (positionFunction.size() > 0) {
            endPositionPoint = positionFunction.get(positionFunction.size() - 1);
        }
    }
    public void setAccelerationFunctionPoints(ArrayList<Point> points) {
        accelerationFunction = points;
        velocityFunction = FunctionGenerator.getIntegral(points);
        positionFunction = FunctionGenerator.getIntegral(velocityFunction);

        if (positionFunction.size() > 0) {
            endPositionPoint = positionFunction.get(positionFunction.size() - 1);
        }
    }


    public ArrayList<Point> getVelocityFunction() {
        return velocityFunction;
    }
    public ArrayList<Point> getAccelerationFunction() {
        return accelerationFunction;
    }
    public ArrayList<Point> getPositionFunction() {
        return positionFunction;
    }
}