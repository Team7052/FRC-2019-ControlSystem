package frc.robot.motionProfiling;

import java.util.ArrayList;

import frc.robot.sequencing.Step;
import frc.robot.sequencing.StepState;

public class MotionProfiler extends Step<MotionTriplet> {
    private ArrayList<Point> velocityFunction = new ArrayList<>();
    private ArrayList<Point> accelerationFunction = new ArrayList<>();
    private ArrayList<Point> positionFunction = new ArrayList<>();

    public MotionProfiler() {
        super();
        this.totalRunningTime = () -> {
            if (velocityFunction.size() == 0) return 0.0;
            return velocityFunction.get(velocityFunction.size() - 1).x;
        };
    }

    public double index = 0;

    @Override
    public MotionTriplet getUpdateForDeltaTime(double dt) {
        if (velocityFunction.size() == 0) return null;
        int index = (int) Math.round(dt / this.getTotalTime() * ((double) velocityFunction.size() - 1));
        
        // get velocity
        double velocityMeasurement = velocityFunction.get(index).y;
        double accelerationMeasurement = accelerationFunction.get(index).y;
        double positionMeasurement = positionFunction.get(index).y;

        return new MotionTriplet(positionMeasurement, velocityMeasurement, accelerationMeasurement);
    }

    public void setVelocityPoints(ArrayList<Point> points) {
        this.setVelocityPoints(points, 0);
    }
    
    // setters and getters for functions
    public void setVelocityPoints(ArrayList<Point> points, double initialDisplacement) {
        velocityFunction = points;
        accelerationFunction = FunctionGenerator.getDerivative(points);
        positionFunction = this.pointsWithInitialDisplacement(FunctionGenerator.getIntegral(points), initialDisplacement);
        this.totalRunningTime = () -> {
            if (velocityFunction.size() == 0) return 0.0;
            return velocityFunction.get(velocityFunction.size() - 1).x;
        };
    }
    public void setPositionPoints(ArrayList<Point> points) {
        this.setPositionPoints(points, 0);
    }
    public void setPositionPoints(ArrayList<Point> points, double initialDisplacement) {
        positionFunction = this.pointsWithInitialDisplacement(points, initialDisplacement);
        velocityFunction = FunctionGenerator.getDerivative(points);
        accelerationFunction = FunctionGenerator.getDerivative(velocityFunction);
        this.totalRunningTime = () -> {
            if (velocityFunction.size() == 0) return 0.0;
            return velocityFunction.get(velocityFunction.size() - 1).x;
        };
    }

    public void setAccelerationFunctionPoints(ArrayList<Point> points) {
        this.setAccelerationFunctionPoints(points, 0);
    }
    public void setAccelerationFunctionPoints(ArrayList<Point> points, double initialDisplacement) {
        accelerationFunction = points;
        velocityFunction = FunctionGenerator.getIntegral(points);
        positionFunction = this.pointsWithInitialDisplacement(FunctionGenerator.getIntegral(velocityFunction), initialDisplacement);
        this.totalRunningTime = () -> {
            if (velocityFunction.size() == 0) return 0.0;
            return velocityFunction.get(velocityFunction.size() - 1).x;
        };
    }

    public double getFinalPosition() {
        return this.positionFunction.get(this.positionFunction.size() - 1).y;
    }
    

    private ArrayList<Point> pointsWithInitialDisplacement(ArrayList<Point> points, double initialDisplacement) {
        ArrayList<Point> converted = new ArrayList<>();
        for (Point point: points) {
            converted.add(new Point(point.x, point.y + initialDisplacement));
        }
        return converted;
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

    
    public void printPositions() {
        for (Point point: this.getPositionFunction()) {
            System.out.print(point.y / Math.PI * 180.0 + " ");
        }
        System.out.println();
    }
}