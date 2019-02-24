package frc.robot.motionProfiling;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.Timer;

public class MotionProfiler {
    private ArrayList<Point> velocityFunction = new ArrayList<>();
    private ArrayList<Point> accelerationFunction = new ArrayList<>();
    private ArrayList<Point> positionFunction = new ArrayList<>();

    private Point endPositionPoint;

    private MotionProfileState state = MotionProfileState.IDLE;

    private double totalRunningTime;

    public MotionProfiler() {
        
    }
    public double startTime = 0;
    public double index = 0;
    public MotionTriplet updateMotionProfile() {
        if (state == MotionProfileState.RUNNING) {
            double currentTime = Timer.getFPGATimestamp();
            
            double percentage = (currentTime - startTime) / this.totalRunningTime;
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

    private void setTotalRunningTime() {
        if (this.positionFunction.size() > 0) {
            this.totalRunningTime = this.positionFunction.get(this.positionFunction.size() - 1).x;
        }
    }

    public void stopMotionProfile() {
        this.state = MotionProfileState.FINISHED;
    }
    public void restartMotionProfile() {
        this.state = MotionProfileState.IDLE;
    }

    public void startMotionProfile() {
        if (this.state == MotionProfileState.IDLE) {
            this.state = MotionProfileState.RUNNING;
            this.startTime = Timer.getFPGATimestamp();
        }
    }

    public static ArrayList<Point> getLinearInterpolation(ArrayList<Point> points, double delta) {
        ArrayList<Point> interpolatedPoints = new ArrayList<>();
        if (points.size() < 2) {
            return points;
        }
        for (int i = 0, n = points.size() - 1; i < n; i++) {
            double deltaX = points.get(i+1).x - points.get(i).x;
            interpolatedPoints.add(points.get(i));
            double slope = (points.get(i+1).y - points.get(i).y) / (points.get(i+1).x - points.get(i).x);
            double b = points.get(i).y - slope * points.get(i).x;

            for (double j = delta; j < deltaX; j += delta) {
                double x = j + points.get(i).x;
                interpolatedPoints.add(new Point(x, slope * x + b));
            }
        }
        interpolatedPoints.add(points.get(points.size() - 1));
        return interpolatedPoints;
    }

    public void setVelocityPoints(ArrayList<Point> points) {
        this.setVelocityPoints(points, 0);
    }
    
    // setters and getters for functions
    public void setVelocityPoints(ArrayList<Point> points, double initialDisplacement) {
        velocityFunction = points;
        accelerationFunction = FunctionGenerator.getDerivative(points);
        positionFunction = this.pointsWithInitialDisplacement(FunctionGenerator.getIntegral(points), initialDisplacement);

        if (positionFunction.size() > 0) {
            endPositionPoint = positionFunction.get(positionFunction.size() - 1);
        }
        this.setTotalRunningTime();
    }
    public void setPositionPoints(ArrayList<Point> points) {
        this.setPositionPoints(points, 0);

    }
    public void setPositionPoints(ArrayList<Point> points, double initialDisplacement) {
        positionFunction = this.pointsWithInitialDisplacement(points, initialDisplacement);
        velocityFunction = FunctionGenerator.getDerivative(points);
        accelerationFunction = FunctionGenerator.getDerivative(velocityFunction);

        if (positionFunction.size() > 0) {
            endPositionPoint = positionFunction.get(positionFunction.size() - 1);
        }
        this.setTotalRunningTime();
    }

    public void setAccelerationFunctionPoints(ArrayList<Point> points) {
        this.setAccelerationFunctionPoints(points, 0);
    }
    public void setAccelerationFunctionPoints(ArrayList<Point> points, double initialDisplacement) {
        accelerationFunction = points;
        velocityFunction = FunctionGenerator.getIntegral(points);
        positionFunction = this.pointsWithInitialDisplacement(FunctionGenerator.getIntegral(velocityFunction), initialDisplacement);

        if (positionFunction.size() > 0) {
            endPositionPoint = positionFunction.get(positionFunction.size() - 1);
        }
        this.setTotalRunningTime();
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

    public static ArrayList<Point> generateRotationMotorTrapezoidalProfile(double startPosition, double endPosition) {
        return generateTrapezoidalProfile(startPosition, endPosition, Math.PI / 4.0, Math.PI / 3.0);
    }

    public static ArrayList<Point> generateTrapezoidalProfile(double startPosition, double endPosition, double maxVelocity, double maxAcceleration) {
        // max acceleration = (pi) rad / s^2
        // max velocity = (pi / 2) rad / s
        double displacement = endPosition - startPosition;
        if (displacement < 0) {
            maxVelocity = -maxVelocity;
            maxAcceleration = -maxAcceleration;
        }
        ArrayList<Point> trapezoidalPoints = new ArrayList<>();
        trapezoidalPoints.add(new Point(0, 0));
        double accelTime = maxVelocity / maxAcceleration;
        double totalTime = displacement / maxVelocity + accelTime;

        if (accelTime >= totalTime / 2) {
            // constrained max velocity
            double halfTime = Math.sqrt(displacement / maxAcceleration);
            double newVelocity = halfTime * maxAcceleration;
            trapezoidalPoints.add(new Point(halfTime, newVelocity));
            trapezoidalPoints.add(new Point(halfTime * 2, 0));
        }
        else {
            trapezoidalPoints.add(new Point(accelTime, maxVelocity));
            trapezoidalPoints.add(new Point(totalTime - accelTime, maxVelocity));
            trapezoidalPoints.add(new Point(totalTime, 0));
        }
        return getLinearInterpolation(trapezoidalPoints, 0.01);
    }
}