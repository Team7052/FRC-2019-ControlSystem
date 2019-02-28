package frc.robot.motionProfiling;

import java.util.ArrayList;

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
            double currentTime = (double) System.currentTimeMillis() / 1000;
            
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
    public void reset() {
        this.state = MotionProfileState.IDLE;
    }

    public void startMotionProfile() {
        if (this.state == MotionProfileState.IDLE) {
            this.state = MotionProfileState.RUNNING;
            this.startTime = (double) System.currentTimeMillis() / 1000;
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

    public double getLeftSum(double desiredPoint, ArrayList<Point> points) {
        double leftSum=0;
        for(int i=0; i<desiredPoint; i++){
            if((points.get(i+1).x - points.get(i).x)<0){
                double theta = Math.atan(10*(points.get(i+1).y - points.get(i).y));
                leftSum+=Math.sin(theta)*(points.get(i+1).y - points.get(i).y);   
            }
        }
        return leftSum*4096;
    }
    public double getRightSum(double desiredPoint, ArrayList<Point> points) {
        double rightSum = 0;
        for(int i=0; i<desiredPoint; i++){
            if((points.get(i+1).y - points.get(i).y)>0){
                double theta = Math.atan(10*(points.get(i+1).y - points.get(i).y));
                rightSum+=Math.sin(theta)*points.get(i).y;
            }
            
        }
        return rightSum*4096;

    }
    public double getLeftSlope(double desiredPoint, ArrayList<Point> points, double ratio) {
        double leftSlope;
        if((points.get((int)desiredPoint+1).y - points.get((int)desiredPoint).y)>=0){
            leftSlope = 1;
        } else {
            leftSlope = (points.get((int)desiredPoint+1).y - points.get((int)desiredPoint).y)/0.1 * ratio;
        }
        return leftSlope;


    }
    public double getRightSlope(double desiredPoint, ArrayList<Point> points, double ratio) {
        double rightSlope;
        if((points.get((int)desiredPoint+1).y - points.get((int)desiredPoint).y)<=0){
            rightSlope=1;
        } else{
            rightSlope = -(points.get((int)desiredPoint+1).y - points.get((int)desiredPoint).y)/0.1 * ratio;
        }
        return rightSlope;
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

    public double getFinalPosition() {
        return this.positionFunction.get(this.positionFunction.size() - 1).y;
    }

    public double getTotalTime() {
        return this.positionFunction.get(this.positionFunction.size() - 1).x;
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
        return trapezoidalPoints;
    }

    public static ArrayList<Point> transformTrapezoidByTime(ArrayList<Point> trapezoidShape, double newTotalTime) { 
        ArrayList<Point> newTrapezoidShape = new ArrayList<>();

        double ratio = newTotalTime / MotionProfiler.totalTimeOfProfile(trapezoidShape);
        for (Point p: trapezoidShape) {
            newTrapezoidShape.add(new Point(p.x * ratio, p.y / ratio));
        }
        
        return newTrapezoidShape;
    }

    public static double totalTimeOfProfile(ArrayList<Point> points) {
        return points.get(points.size() - 1).x;
    }
    public void printPositions() {
        for (Point point: this.getPositionFunction()) {
            System.out.print(point.y / Math.PI * 180.0 + " ");
        }
        System.out.println();
    }
}