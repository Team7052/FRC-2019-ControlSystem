package frc.robot.motionProfiling;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.Timer;

public class MotionProfiler {
    private ArrayList<Point> velocityFunction = new ArrayList<>();
    private ArrayList<Point> accelerationFunction = new ArrayList<>();
    private ArrayList<Point> positionFunction = new ArrayList<>();


    public MotionProfiler() {
        
    }
    private boolean running = false;
    public double startTime = 0;
    public double index = 0;
    public MotionTriplet updateMotionProfile(double totalRunningTime) {
        if (running) {
            double currentTime = Timer.getFPGATimestamp();
            double percentage = (currentTime - startTime) / totalRunningTime;
            System.out.println(percentage);
            if (percentage > 1.0) {
                running = false;
                return null;
            }
            int index = (int) Math.round(percentage * (double) velocityFunction.size());

            
            
            // get velocity
            double velocityMeasurement = velocityFunction.get(index).y;
            double accelerationMeasurement = accelerationFunction.get(index).y;
            double positionMeasurement = positionFunction.get(index).y;
            return new MotionTriplet(velocityMeasurement, positionMeasurement, accelerationMeasurement);
        }
        // set
        else {
            startTime = Timer.getFPGATimestamp();
            running = true;
            return null;
        }
    }

    public void setVelocityPoints(ArrayList<Point> points) {
        velocityFunction = points;
        accelerationFunction = FunctionGenerator.getDerivative(points);
        positionFunction = FunctionGenerator.getIntegral(points);
    }
    public void setPositionPoints(ArrayList<Point> points) {
        positionFunction = points;
        velocityFunction = FunctionGenerator.getDerivative(points);
        accelerationFunction = FunctionGenerator.getDerivative(velocityFunction);
    }
    public void setAccelerationFunctionPoints(ArrayList<Point> points) {
        accelerationFunction = points;
        velocityFunction = FunctionGenerator.getIntegral(points);
        positionFunction = FunctionGenerator.getIntegral(velocityFunction);
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